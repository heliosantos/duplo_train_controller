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
#include "nvs.h"
#include "freertos/semphr.h"
#include "esp_sleep.h"




/* =========================
   GPIO pins
   ========================= */
#define GPIO_FORWARD  GPIO_NUM_0
#define GPIO_STOP     GPIO_NUM_1
#define GPIO_BACKWARD GPIO_NUM_2


#define NVS_NAMESPACE "train_cfg"
#define BUTTON_DEBOUNCE_MS 250

#define COOLDOWN_RED_MS    2000
#define COOLDOWN_YELLOW_MS 2000
#define COOLDOWN_GREEN_MS  2000
#define COOLDOWN_BLUE_MS   2000
#define COOLDOWN_WHITE_MS  2000
#define INACTIVITY_TIMEOUT_MS (10 * 60 * 1000)  // 10 minutes


static SemaphoreHandle_t ble_write_sem;

int throttle = 0;
int prev_throttle = 0;
int resume_throttle = 0;

int lights_on = 1;
int current_led_color = 0;

static int ble_write_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr,
                        void *arg) {

    xSemaphoreGive(ble_write_sem);
    return 0;
}

/* =========================
   Forward declarations
   ========================= */
void ble_app_on_sync(void);


/* =========================
   BLE target UUID
   ========================= */
static const ble_uuid128_t target_char_uuid = BLE_UUID128_INIT(
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x16,
    0xde, 0xef, 0x12, 0x12, 0x24, 0x16, 0x00, 0x00
);

typedef enum {
    EVT_FORWARD,
    EVT_STOP,
    EVT_BACKWARD,
    EVT_RESUME
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
static QueueHandle_t movement_evt_queue;
static QueueHandle_t color_action_queue;

static TickType_t last_forward_tick  = 0;
static TickType_t last_stop_tick     = 0;
static TickType_t last_backward_tick = 0;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    TickType_t now = xTaskGetTickCountFromISR();

    control_event_t evt;
    TickType_t *last_tick = NULL;

    if (gpio_num == GPIO_FORWARD) {
        evt = EVT_FORWARD;
        last_tick = &last_forward_tick;
    } else if (gpio_num == GPIO_STOP) {
        evt = EVT_STOP;
        last_tick = &last_stop_tick;
    } else {
        evt = EVT_BACKWARD;
        last_tick = &last_backward_tick;
    }

    if ((now - *last_tick) * portTICK_PERIOD_MS < BUTTON_DEBOUNCE_MS) {
        return;
    }

    *last_tick = now;
    xQueueSendFromISR(movement_evt_queue, &evt, NULL);
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

            xSemaphoreTake(ble_write_sem, portMAX_DELAY);

            ble_gattc_write_flat(
                conn_handle,
                char_val_handle,
                cmd.data,
                cmd.len,
                ble_write_cb,
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

int beeper_mode = 0;

void set_beeper_mode(int mode) {
    uint8_t data[] = {0x0a, 0x00, 0x41, 0x01, mode, 0x01, 0x00, 0x00, 0x00, 0x01};
    enqueue_command(data, sizeof(data));
}

void set_beeper_tone_mode(void) {
    if (beeper_mode == 0) return;
    beeper_mode = 0;
    set_beeper_mode(beeper_mode);
}

void set_beeper_sound_mode(void) {
    if (beeper_mode == 1) return;
    beeper_mode = 1;
    set_beeper_mode(beeper_mode);
}

/*
    3: LOW (brake)
    9: MED (whistle)
    10: HIGH (horn)
*/
void beeper_play_tone(int soundId) {
    set_beeper_tone_mode();
    uint8_t data[] = {0x08, 0x00, 0x81, 0x01, 0x11, 0x51, 0x0, soundId};
    enqueue_command(data, sizeof(data));
}

/*
    break: 3
    start: 5
    water: 7
    whistle: 9
    horn: 10
*/
void beeper_play_sound(int soundId) {
    set_beeper_sound_mode();
    uint8_t data[] = {0x08, 0x00, 0x81, 0x01, 0x11, 0x51, 0x01, soundId};
    enqueue_command(data, sizeof(data));
}

/*
    NONE: 0,
    MAGENTA: 2,
    BLUE: 3,
    GREEN: 6,
    YELLOW: 7,
    ORANGE: 8,
    RED: 9,
*/
void set_color(int colorId) {
    current_led_color = colorId;
    int color = lights_on ? current_led_color : 0;
    uint8_t data[] = {0x08, 0x00, 0x81, 0x11, 0x11, 0x51, 0x00, color};
    enqueue_command(data, sizeof(data));
}

void enable_speed_notifications(void) {
    uint8_t data[] = {0x0a, 0x00, 0x41, 0x13, 0x00, 0x50, 0x00, 0x00, 0x00, 0x01};
    enqueue_command(data, sizeof(data));
}

void enable_color_sensor_notifications(void) {
    uint8_t data[] = {0x0a, 0x00, 0x41, 0x12, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01};
    enqueue_command(data, sizeof(data));
}

void shutdown_hub(void) {
    // Hub Action command: 0x02 = Switch Off Hub
    uint8_t data[] = {0x04, 0x00, 0x02, 0x01};
    enqueue_command(data, sizeof(data));
}

/* =========================
   NVS helpers (per-MAC sound)
   ========================= */

static void addr_to_key(const ble_addr_t *addr, char *out, size_t out_len) {
    // Key: 12 hex chars from the 6 MAC bytes, e.g. "aabbccddeeff"
    snprintf(out, out_len, "%02x%02x%02x%02x%02x%02x",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

static int load_default_sound(const ble_addr_t *addr) {
    nvs_handle_t h;
    int sound = 10; // factory default: horn

    char key[16];
    addr_to_key(addr, key, sizeof(key));

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        int32_t val;
        if (nvs_get_i32(h, key, &val) == ESP_OK) {
            sound = (int)val;
            ESP_LOGI("NVS", "Loaded sound %d for %s", sound, key);
        }
        nvs_close(h);
    }
    return sound;
}

static void save_default_sound(const ble_addr_t *addr, int sound) {
    nvs_handle_t h;
    char key[16];
    addr_to_key(addr, key, sizeof(key));

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, key, (int32_t)sound);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI("NVS", "Saved sound %d for %s", sound, key);
    }
}

/* =========================
   Sound selection mode
   ========================= */

// All valid sound IDs in cycling order
static const int SOUND_LIST[]  = {3, 5, 7, 9, 10, 0};
#define SOUND_LIST_LEN (sizeof(SOUND_LIST) / sizeof(SOUND_LIST[0]))

static int default_sound      = 10;  // loaded from NVS on connect
static int sound_select_mode  = 0;   // 1 = active
static int sound_select_index = 0;   // index into SOUND_LIST
static int stop_press_count   = 0;   // consecutive stop presses

static ble_addr_t connected_addr;    // MAC of currently connected train

static int sound_to_index(int sound) {
    for (int i = 0; i < (int)SOUND_LIST_LEN; i++) {
        if (SOUND_LIST[i] == sound) return i;
    }
    return 0;
}

/* =========================
   Notify parsing
   ========================= */
void process_speed_change(uint8_t *payload, int len) {
    if (len < 2) return;
    int16_t speed = payload[0] | (payload[1] << 8);
    ESP_LOGI("Train", "Speed: %d", speed);

    if (speed > 0) {
        set_color(6);
    } else if (speed < 0) {
        set_color(3);
    } else {
        // xQueueSend(movement_evt_queue, &(control_event_t){EVT_STOP}, 0);
        throttle = 0;
        set_color(9);
    }
}

/* =========================
   Color sensor cooldowns (ms)
   ========================= */

static TickType_t last_red_tick    = 0;
static TickType_t last_yellow_tick = 0;
static TickType_t last_green_tick  = 0;
static TickType_t last_blue_tick   = 0;
static TickType_t last_white_tick  = 0;

#define IS_COOLING_DOWN(last_tick, cooldown_ms) \
    ((xTaskGetTickCount() - (last_tick)) * portTICK_PERIOD_MS < (cooldown_ms))

/* =========================
   Color action queue
   ========================= */
typedef enum {
    COLOR_ACTION_RED,
    COLOR_ACTION_YELLOW,
    COLOR_ACTION_GREEN,
    COLOR_ACTION_BLUE,
    COLOR_ACTION_WHITE
} color_action_t;


/* =========================
   Inactivity shutdown
   ========================= */

static volatile TickType_t last_activity_tick = 0;

void update_activity(void) {
    last_activity_tick = xTaskGetTickCount();
}

void watchdog_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(15000));  // check every 15 seconds

        TickType_t elapsed = (xTaskGetTickCount() - last_activity_tick) * portTICK_PERIOD_MS;

        if (last_activity_tick != 0 && elapsed >= INACTIVITY_TIMEOUT_MS) {
            ESP_LOGI("Watchdog", "Inactivity timeout — shutting down");

            while(device_connected) {
                ESP_LOGI("Watchdog", "Send hub shutdown command");

                xQueueReset(cmd_queue);
                xQueueReset(movement_evt_queue);
                xQueueReset(color_action_queue);

                shutdown_hub();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            // // Disconnect BLE
            // if (device_connected) {
            //     ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            //     vTaskDelay(pdMS_TO_TICKS(500));
            // }

            // Deep sleep forever (wake on reset button only)
            esp_deep_sleep_start();
        }
    }
}

/* =========================
   Color action task
   ========================= */
void color_action_task(void *arg)
{
    color_action_t action;

    while (1) {
        if (xQueueReceive(color_action_queue, &action, portMAX_DELAY)) {

            switch (action) {

            case COLOR_ACTION_RED:
                ESP_LOGI("COLOR", "RED → stop");
                xQueueSend(movement_evt_queue, &(control_event_t){EVT_STOP}, 0);
                beeper_play_sound(3);
                break;

            case COLOR_ACTION_YELLOW:
                ESP_LOGI("COLOR", "YELLOW → horn");
                beeper_play_sound(default_sound);
                break;

            case COLOR_ACTION_GREEN:
                ESP_LOGI("COLOR", "GREEN → reverse");
                resume_throttle = -throttle;
                xQueueSend(movement_evt_queue, &(control_event_t){EVT_STOP}, 0);
                beeper_play_sound(3);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xQueueSend(movement_evt_queue, &(control_event_t){EVT_RESUME}, 0);
                beeper_play_sound(default_sound);
                break;

            case COLOR_ACTION_BLUE:
                ESP_LOGI("COLOR", "BLUE → refuel");
                resume_throttle = throttle;
                xQueueSend(movement_evt_queue, &(control_event_t){EVT_STOP}, 0);
                beeper_play_sound(3);
                vTaskDelay(pdMS_TO_TICKS(1000));
                beeper_play_sound(7);
                vTaskDelay(pdMS_TO_TICKS(1100));
                beeper_play_sound(7);
                vTaskDelay(pdMS_TO_TICKS(1100));
                beeper_play_sound(7);
                vTaskDelay(pdMS_TO_TICKS(1100));
                beeper_play_sound(7);
                vTaskDelay(pdMS_TO_TICKS(2000));
                xQueueSend(movement_evt_queue, &(control_event_t){EVT_RESUME}, 0);
                break;

            case COLOR_ACTION_WHITE:
                lights_on = lights_on == 1 ? 0 : 1;
                set_color(current_led_color);
                beeper_play_tone(3);
                break;
            }
        }
    }
}

/* =========================
   Color sensor processing
   ========================= */
void process_color_sensor(uint8_t *payload, int len)
{
    uint8_t c = payload[len - 1];

    switch (c) {
        case 0x09: // RED
            if (IS_COOLING_DOWN(last_red_tick, COOLDOWN_RED_MS)) return;
            last_red_tick = xTaskGetTickCount();
            xQueueSend(color_action_queue, &(color_action_t){COLOR_ACTION_RED}, 0);
            break;

        case 0x07: // YELLOW
            if (IS_COOLING_DOWN(last_yellow_tick, COOLDOWN_YELLOW_MS)) return;
            last_yellow_tick = xTaskGetTickCount();
            xQueueSend(color_action_queue, &(color_action_t){COLOR_ACTION_YELLOW}, 0);
            break;

        case 0x05: // GREEN
            if (IS_COOLING_DOWN(last_green_tick, COOLDOWN_GREEN_MS)) return;
            last_green_tick = xTaskGetTickCount();
            xQueueSend(color_action_queue, &(color_action_t){COLOR_ACTION_GREEN}, 0);
            break;

        case 0x03: // BLUE
            if (IS_COOLING_DOWN(last_blue_tick, COOLDOWN_BLUE_MS)) return;
            last_blue_tick = xTaskGetTickCount();
            xQueueSend(color_action_queue, &(color_action_t){COLOR_ACTION_BLUE}, 0);
            break;

        case 0x0A: // WHITE
            if (IS_COOLING_DOWN(last_white_tick, COOLDOWN_WHITE_MS)) return;
            last_white_tick = xTaskGetTickCount();
            xQueueSend(color_action_queue, &(color_action_t){COLOR_ACTION_WHITE}, 0);
            break;

        default:
            break;
    }
}

void parse_message(uint8_t *data, int len) {
    if (len < 5) return;

    if (data[2] == 0x45 && data[3] == 0x13) {
        process_speed_change(&data[4], len - 4);
    } else if (data[2] == 0x45 && data[3] == 0x12) {
        process_color_sensor(&data[4], len - 4);
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
        ble_gattc_write_flat(conn_handle, ccc_handle, enable_notify, sizeof(enable_notify), NULL, NULL);
        enable_speed_notifications();
        enable_color_sensor_notifications();
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

        ble_gattc_disc_all_dscs(conn_handle, chr->val_handle, 0xffff, dsc_cb, NULL);
    }

    return 0;
}

void discover_characteristics(void) {
    ble_gattc_disc_all_chrs(conn_handle, 1, 0xffff, discover_cb, NULL);
}

/* =========================
   GAP events
   ========================= */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {

    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;

        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
            if (fields.name && fields.name_len > 0) {
                char name[32];
                int len = fields.name_len < 31 ? fields.name_len : 31;
                memcpy(name, fields.name, len);
                name[len] = '\0';

                if (!device_connected && strcmp(name, "Train Base") == 0) {
                    ble_gap_disc_cancel();
                    ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
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

            // Store MAC and load per-device sound preference
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(conn_handle, &desc) == 0) {
                connected_addr = desc.peer_id_addr;
            }
            default_sound = load_default_sound(&connected_addr);

            // Reset sound-select state on each new connection
            sound_select_mode = 0;
            stop_press_count  = 0;

            ESP_LOGI("BLE", "Connected! Default sound: %d", default_sound);
            discover_characteristics();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("BLE", "Disconnected");
        device_connected = false;
        ble_ready = false;
        sound_select_mode = 0;
        stop_press_count  = 0;
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
    gpio_isr_handler_add(GPIO_FORWARD,  gpio_isr_handler, (void*)GPIO_FORWARD);
    gpio_isr_handler_add(GPIO_STOP,     gpio_isr_handler, (void*)GPIO_STOP);
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

    while (1) {

        if (!xQueueReceive(movement_evt_queue, &evt, portMAX_DELAY)) continue;

        update_activity();

        /* ------------------------------------------------
           Sound-selection mode
           Forward  → preview next sound (cycles through list)
           Backward → commit as new default and exit
           Stop     → exit with no changes
           ------------------------------------------------ */
        if (sound_select_mode) {

            switch (evt) {

                case EVT_FORWARD:
                    sound_select_index = (sound_select_index + 1) % SOUND_LIST_LEN;
                    beeper_play_sound(SOUND_LIST[sound_select_index]);
                    ESP_LOGI("SoundSel", "Preview sound %d",
                             SOUND_LIST[sound_select_index]);
                    break;

                case EVT_BACKWARD:
                    default_sound = SOUND_LIST[sound_select_index];
                    save_default_sound(&connected_addr, default_sound);
                    beeper_play_sound(default_sound);
                    ESP_LOGI("SoundSel", "Committed sound %d", default_sound);
                    sound_select_mode = 0;
                    stop_press_count  = 0;
                    break;

                case EVT_STOP:
                    ESP_LOGI("SoundSel", "Cancelled, keeping sound %d",
                             default_sound);
                    sound_select_mode = 0;
                    stop_press_count  = 0;
                    break;

                default:
                    break;
            }

            // All motor commands are suppressed while in selection mode
            continue;
        }

        /* ------------------------------------------------
           Normal mode
           ------------------------------------------------ */
        switch (evt) {

            case EVT_FORWARD:
                stop_press_count = 0;
                throttle = (throttle < 3) ? throttle + 1 : throttle;
                break;

            case EVT_STOP:
                throttle = 0;
                stop_press_count++;

                if (stop_press_count >= 10) {
                    sound_select_mode  = 1;
                    sound_select_index = sound_to_index(default_sound);
                    stop_press_count   = 0;
                    beeper_play_tone(3);
                    // beeper_play_sound(default_sound);
                    ESP_LOGI("SoundSel",
                             "Entered sound-select mode (current sound=%d)",
                             default_sound);
                    prev_throttle = throttle;
                    continue; // skip motor command
                }
                break;

            case EVT_BACKWARD:
                stop_press_count = 0;
                throttle = (throttle > -3) ? throttle - 1 : throttle;
                break;

            case EVT_RESUME:
                prev_throttle = throttle;
                throttle = resume_throttle;
                break;
        }

        if (throttle != prev_throttle) {
            int throttle_perc = throttle_to_perc(throttle);
            ESP_LOGI("Train", "throttle: %d (%d%%)", throttle, throttle_perc);
            move(throttle_perc);
        }

        // Play default sound when already at max speed and forward is pressed again
        if (prev_throttle == 3 && throttle == 3 && evt == EVT_FORWARD) {
            beeper_play_sound(default_sound);
        }

        prev_throttle = throttle;
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

    cmd_queue          = xQueueCreate(10, sizeof(train_cmd_t));
    movement_evt_queue = xQueueCreate(10, sizeof(control_event_t));
    color_action_queue = xQueueCreate(10, sizeof(color_action_t));

    ble_write_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(ble_write_sem);

    xTaskCreate(ble_tx_task,       "ble_tx",       4096, NULL, 5, NULL);
    xTaskCreate(control_task,      "control",      4096, NULL, 5, NULL);
    xTaskCreate(color_action_task, "color_action", 4096, NULL, 5, NULL);
    xTaskCreate(watchdog_task,     "watchdog",     2048, NULL, 3, NULL);

    config_gpio();
}