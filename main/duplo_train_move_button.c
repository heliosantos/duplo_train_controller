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

static bool device_connected = false;
static uint8_t own_addr_type;
static bool scanning = false;

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            struct ble_hs_adv_fields fields;
            if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
                if (fields.name && fields.name_len > 0) {
                    char name[32] = {0};
                    memcpy(name, fields.name, fields.name_len);
                    name[fields.name_len] = '\0';

                    ESP_LOGI(TAG, "Discovered device: %s", name);

                    if (!device_connected && strcmp(name, "Train Base") == 0) {
                        ESP_LOGI(TAG, "Found Train Base, connecting...");
                        
                        ble_gap_disc_cancel(); // cancel the scan
                                               //
                        ble_addr_t *addr = &event->disc.addr;

                        int rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, ble_gap_event, NULL);

                        if (rc != 0) {
                            ESP_LOGE(TAG, "Failed to connect: %d", rc);
                        } else {
                            device_connected = true;
                            scanning = false;
                        }
                    }
                }
            }
            return 0;
        }

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Connected to Train Base!");
                scanning = false;
            } else {
                ESP_LOGE(TAG, "Connection failed; status=%d", event->connect.status);
                device_connected = false;
                // Optionally restart scanning here if needed
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
            device_connected = false;
            // Restart scanning after disconnect
            if (!scanning) {
                struct ble_gap_disc_params scan_params = {
                    .passive = 0,
                    .filter_duplicates = 1,
                    .itvl = 0x0010,
                    .window = 0x0010,
                    .filter_policy = 0,
                    .limited = 0,
                };
                int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &scan_params, ble_gap_event, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Error restarting scan; rc=%d", rc);
                } else {
                    scanning = true;
                    ESP_LOGI(TAG, "Restarted scanning after disconnect");
                }
            }
            return 0;

        default:
            return 0;
    }
}

void ble_app_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);

    ESP_LOGI(TAG, "BLE Host synced, own_addr_type=%d", own_addr_type);

    struct ble_gap_disc_params scan_params = {
        .passive = 0,
        .filter_duplicates = 1,
        .itvl = 0x0010,
        .window = 0x0010,
        .filter_policy = 0,
        .limited = 0,
    };

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &scan_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting scan; rc=%d", rc);
    } else {
        scanning = true;
        ESP_LOGI(TAG, "Started scanning");
    }
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(ble_host_task);
}

