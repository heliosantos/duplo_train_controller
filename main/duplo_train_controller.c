#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"

/* =========================
   Forward declarations (FIX)
   ========================= */
void ble_app_on_sync(void);

/* =========================
   GPIO pins
   ========================= */
#define GPIO_FORWARD  GPIO_NUM_0
#define GPIO_STOP     GPIO_NUM_1
#define GPIO_BACKWARD GPIO_NUM_2

/* =========================
   BLE target UUID
   ========================= */
static const ble_uuid128_t target_char_uuid =
    BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x16,
                     0xde, 0xef, 0x12, 0x12, 0x24, 0x16, 0x00, 0x00);

/* ========================= */
#define MSG_PORT_VALUE_SINGLE 0x45

typedef enum {
    EVT_FORWARD,
    EVT_STOP,
    EVT_BACKWARD
} control_event_t;

/* =========================
   State
   ========================= */
static bool device_connected = false;
static bool ble_ready = false;

static uint8_t own_addr_type;
static uint16_t conn_handle;
static uint16_t char_val_handle;
static uint16_t ccc_handle;

/* =========================
   Command queue
   ========================= */
typedef struct {
    uint8_t data[10];
    uint8_t len;
} train_cmd_t;

static QueueHandle_t cmd_queue;
static QueueHandle_t gpio_evt_queue;

/* =========================
   ISR
   ========================= */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;

    control_event_t evt;

    if (gpio_num == GPIO_FORWARD) evt = EVT_FORWARD;
    else if (gpio_num == GPIO_STOP) evt = EVT_STOP;
    else evt = EVT_BACKWARD;

    xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
}

/* =========================
   Queue helper
   ========================= */
void enqueue_command(uint8_t *data, uint8_t len) {
    train_cmd_t cmd;
    memcpy(cmd.data, data, len);
    cmd.len = len;

    if (xQueueSend(cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW("Queue", "Queue full, dropping command");
    }
}

/* =========================
   BLE TX task
   ========================= */
void ble_tx_task(void *param) {
    train_cmd_t cmd;

    while (1) {
        if (xQueueReceive(cmd_queue, &cmd, portMAX_DELAY)) {

            if (!device_connected || !ble_ready) {
                continue;
            }

            ble_gattc_write_flat(
                conn_handle,
                char_val_handle,
                cmd.data,
                cmd.len,
                NULL,
                NULL
            );
        }
    }
}

/* =========================
   Commands
   ========================= */
void move(int throttle) {
    if (throttle < 0) throttle = 256 + throttle;

    uint8_t data[] = {0x08, 0x00, 0x81, 0x00, 0x11, 0x51, 0x00, throttle};
    enqueue_command(data, sizeof(data));
}

void setup_cmd() {
    uint8_t data[] = {0x0a, 0x00, 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01};
    enqueue_command(data, sizeof(data));
}

void play_sound(int soundId) {
    uint8_t data[] = {0x08, 0x00, 0x81, 0x01, 0x11, 0x51, 0x01, soundId};
    enqueue_command(data, sizeof(data));
}

void set_color(int colorId) {
    uint8_t data[] = {0x08, 0x00, 0x81, 0x11, 0x11, 0x51, 0x00, colorId};
    enqueue_command(data, sizeof(data));
}

void enable_speed_notifications() {
    uint8_t data[] = {0x0a, 0x00, 0x41, 0x13, 0x00, 0x50, 0x00, 0x00, 0x00, 0x01};
    enqueue_command(data, sizeof(data));
}

/* =========================
   Notify parsing
   ========================= */
void process_speed_change(uint8_t *payload, int len) {
    if (len < 2) return;

    int16_t speed = payload[0] | (payload[1] << 8);
    ESP_LOGI("Train", "Speed: %d", speed);
}

void parse_message(uint8_t *data, int len) {
    if (len < 5) return;

    if (data[2] == MSG_PORT_VALUE_SINGLE && data[3] == 0x13) {
        process_speed_change(&data[4], len - 4);
    }
}

/* =========================
   Descriptor discovery
   ========================= */
static int dsc_cb(uint16_t conn_handle,
                  const struct ble_gatt_error *error,
                  uint16_t chr_val_handle,
                  const struct ble_gatt_dsc *dsc,
                  void *arg) {

    if (error->status == 0 && ble_uuid_u16(&dsc->uuid.u) == 0x2902) {

        ccc_handle = dsc->handle;

        uint8_t enable_notify[2] = {0x01, 0x00};

        ble_gattc_write_flat(conn_handle,
                             ccc_handle,
                             enable_notify,
                             sizeof(enable_notify),
                             NULL,
                             NULL);

        ESP_LOGI("BLE", "🔔 Notifications enabled");

        enable_speed_notifications();
    }

    return 0;
}

/* =========================
   Characteristic discovery
   ========================= */
static int discover_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr,
                       void *arg) {

    if (error->status == 0 &&
        ble_uuid_cmp(&chr->uuid.u, &target_char_uuid.u) == 0) {

        char_val_handle = chr->val_handle;
        ble_ready = true;

        ESP_LOGI("BLE", "🎯 Characteristic found");

        ble_gattc_disc_all_dscs(conn_handle,
                                chr->val_handle,
                                0xffff,
                                dsc_cb,
                                NULL);
    }

    return 0;
}

void discover_characteristics() {
    ble_gattc_disc_all_chrs(conn_handle, 1, 0xffff, discover_cb, NULL);
}

/* =========================
   GAP events
   ========================= */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {

    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;

        if (ble_hs_adv_parse_fields(&fields,
                                   event->disc.data,
                                   event->disc.length_data) == 0) {

            if (fields.name && fields.name_len > 0) {

                char name[32];
                int len = fields.name_len < 31 ? fields.name_len : 31;
                memcpy(name, fields.name, len);
                name[len] = '\0';

                if (!device_connected && strcmp(name, "Train Base") == 0) {

                    ble_gap_disc_cancel();

                    ble_gap_connect(own_addr_type,
                                    &event->disc.addr,
                                    30000,
                                    NULL,
                                    ble_gap_event,
                                    NULL);
                }
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            device_connected = true;
            ble_ready = false;

            ESP_LOGI("BLE", "Connected!");
            discover_characteristics();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("BLE", "Disconnected");

        device_connected = false;
        ble_ready = false;

        ble_app_on_sync();  // reconnect

        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        struct os_mbuf *om = event->notify_rx.om;

        uint8_t data[64];
        int len = OS_MBUF_PKTLEN(om);

        os_mbuf_copydata(om, 0, len, data);

        parse_message(data, len);
        return 0;
    }

    default:
        return 0;
    }
}

/* =========================
   BLE init
   ========================= */
void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);

    struct ble_gap_disc_params scan_params = {
        .passive = 0,
        .itvl = 0x10,
        .window = 0x10,
    };

    ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                 &scan_params, ble_gap_event, NULL);
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* =========================
   GPIO init
   ========================= */
void config_gpio(void) {

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_FORWARD) |
                        (1ULL << GPIO_STOP) |
                        (1ULL << GPIO_BACKWARD),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    gpio_config(&cfg);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_FORWARD, gpio_isr_handler, (void*)GPIO_FORWARD);
    gpio_isr_handler_add(GPIO_STOP, gpio_isr_handler, (void*)GPIO_STOP);
    gpio_isr_handler_add(GPIO_BACKWARD, gpio_isr_handler, (void*)GPIO_BACKWARD);
}

int throttle_to_perc(int speed) {
    switch (speed) {
        case -3: return -100;
        case -2: return -70;
        case -1: return -40;
        case  0: return 0;
        case  1: return 40;
        case  2: return 70;
        case  3: return 100;
        default: return 0;
    }
}

/* =========================
   Control task
   ========================= */
void control_task(void *arg) {

    control_event_t evt;
    int throttle = 0;
    int prev_throttle = 0;

    while (1) {

        if (xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {

            switch (evt) {

            case EVT_FORWARD:
                throttle = (throttle < 3) ? throttle + 1 : throttle;
                break;

            case EVT_STOP:
                throttle = 0;
                break;

            case EVT_BACKWARD:
                throttle = (throttle > -3) ? throttle - 1 : throttle;
                break;
            }

            if (throttle != prev_throttle) {

                int throttle_perc = throttle_to_perc(throttle);

                ESP_LOGI("Train", "throttle: %d (%d%)", throttle, throttle_perc);

                move(throttle_perc);

                if (throttle > 0) set_color(6);
                else if (throttle < 0) set_color(3);
                else set_color(9);
            }

            if (prev_throttle == 3 && throttle == 3 && evt == EVT_FORWARD) {
                setup_cmd();
                play_sound(10);
            }

            prev_throttle = throttle;

            // drain remaining queued events
            vTaskDelay(pdMS_TO_TICKS(250));
            xQueueReset(gpio_evt_queue);

        }
    }
}

/* =========================
   MAIN
   ========================= */
void app_main(void) {

    nvs_flash_init();

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(ble_host_task);

    cmd_queue = xQueueCreate(10, sizeof(train_cmd_t));
    gpio_evt_queue = xQueueCreate(10, sizeof(control_event_t));

    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);

    config_gpio();
}