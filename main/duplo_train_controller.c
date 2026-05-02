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

static const char *TAG = "BLE_CONNECT";

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

/* =========================
   State
   ========================= */
static bool device_connected = false;
static bool ble_ready = false;
static bool scanning = false;

static uint8_t own_addr_type;
static uint16_t conn_handle;
static uint16_t char_val_handle;

/* =========================
   Command queue
   ========================= */
typedef struct {
    uint8_t data[10];
    uint8_t len;
} train_cmd_t;

static QueueHandle_t cmd_queue;

/* =========================
   GPIO event queue
   ========================= */
static QueueHandle_t gpio_evt_queue;

/* =========================
   Button state (updated by ISR task)
   ========================= */
static volatile int forward_pressed = 0;
static volatile int stop_pressed = 0;
static volatile int backward_pressed = 0;

/* =========================
   ISR
   ========================= */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/* =========================
   GPIO event task
   ========================= */
void gpio_task(void *arg) {
    uint32_t io_num;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {

            int level = gpio_get_level(io_num);

            if (io_num == GPIO_FORWARD) {
                forward_pressed = (level == 0);
            }
            else if (io_num == GPIO_STOP) {
                stop_pressed = (level == 0);
            }
            else if (io_num == GPIO_BACKWARD) {
                backward_pressed = (level == 0);
            }
        }
    }
}

/* =========================
   Queue helper
   ========================= */
void enqueue_command(uint8_t *data, uint8_t len) {
    train_cmd_t cmd;
    memcpy(cmd.data, data, len);
    cmd.len = len;

    if (xQueueSend(cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full, dropping command");
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
                ESP_LOGW(TAG, "BLE not ready, dropping command");
                continue;
            }

            int rc = ble_gattc_write_flat(
                conn_handle,
                char_val_handle,
                cmd.data,
                cmd.len,
                NULL,
                NULL
            );

            if (rc != 0) {
                ESP_LOGE(TAG, "❌ Failed to write: %d", rc);
            }
        }
    }
}

/* =========================
   Command wrappers
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

/* =========================
   BLE discovery
   ========================= */
static int discover_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr,
                       void *arg) {

    if (error->status == 0) {
        if (ble_uuid_cmp(&chr->uuid.u, &target_char_uuid.u) == 0) {
            char_val_handle = chr->val_handle;
            ble_ready = true;
            ESP_LOGI(TAG, "🎯 Characteristic found, ready!");
        }
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
                char name[32] = {0};
                memcpy(name, fields.name, fields.name_len);

                if (!device_connected && strcmp(name, "Train Base") == 0) {

                    ESP_LOGI(TAG, "Found Train Base, connecting...");

                    ble_gap_disc_cancel();
                    scanning = false;

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

            ESP_LOGI(TAG, "Connected!");
            discover_characteristics();
        } else {
            device_connected = false;
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");

        device_connected = false;
        ble_ready = false;

        return 0;

    default:
        return 0;
    }
}

/* =========================
   BLE init
   ========================= */
void ble_app_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);

    struct ble_gap_disc_params scan_params = {
        .passive = 0,
        .filter_duplicates = 1,
        .itvl = 0x0010,
        .window = 0x0010,
    };

    ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                 &scan_params, ble_gap_event, NULL);

    scanning = true;
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* =========================
   GPIO init (INTERRUPTS)
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

int speed_to_throttle(int speed) {
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
   MAIN CONTROL LOOP
   ========================= */
void control_task(void *arg) {

    int prev_speed = 0;

    while (1) {

        int forward = forward_pressed;
        int stop = stop_pressed;
        int backward = backward_pressed;

        int speed = prev_speed;

        /* =========================
           Speed control (discrete)
           ========================= */

        if (forward && !stop && !backward) {
            if (speed < 3) speed++;

        } else if (!forward && stop && !backward) {
            speed = 0;

        } else if (!forward && !stop && backward) {
            if (speed > -3) speed--;
        }

        /* =========================
           Apply change
           ========================= */

        if (speed != prev_speed) {

            int throttle = speed_to_throttle(speed);

            ESP_LOGI(TAG, "Speed: %d -> Throttle: %d", speed, throttle);

            move(throttle);

            if (speed > 0) set_color(6);       // Green
            else if (speed < 0) set_color(3);  // Blue
            else set_color(9);                 // Red
        }

        /* Max speed action */
        else if (speed == 3 && forward) {
            setup_cmd();
            play_sound(10);
        }

        prev_speed = speed;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* =========================
   MAIN
   ========================= */
void app_main(void) {

    esp_err_t ret = nvs_flash_init();
    if (ret != ESP_OK) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(ble_host_task);

    cmd_queue = xQueueCreate(10, sizeof(train_cmd_t));
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 5, NULL);
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);

    config_gpio();
}