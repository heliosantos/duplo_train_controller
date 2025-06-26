#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

static const char *TAG = "BLE_CONNECT";
static uint8_t own_addr_type;

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected to Train Base!");
        } else {
            ESP_LOGE(TAG, "Failed to connect: %d", event->connect.status);
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        ESP_LOGI(TAG, "Disconnected");
    }
    return 0;
}

void ble_app_on_sync(void) {
    ble_addr_t addr = {
        .type = 0,
        .val = {0x16, 0xF9, 0x51, 0x65, 0x6C, 0xA0} 
    };
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_gap_connect(own_addr_type, &addr, 30000, NULL, ble_gap_event, NULL);
    ESP_LOGI(TAG, "Connecting to Train Base...");
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    nvs_flash_init();
    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

