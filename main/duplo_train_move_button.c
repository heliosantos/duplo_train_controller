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

// 00001624-1212-efde-1623-785feabcd123
static const ble_uuid128_t target_char_uuid = BLE_UUID128_INIT(
    0x23, 0xd1, 0xbc, 0xea,
    0x5f, 0x78,
    0x23, 0x16,
    0xde, 0xef,
    0x12, 0x12, 0x24, 0x16, 0x00, 0x00
    );

static bool device_connected = false;
static uint8_t own_addr_type;
static bool scanning = false;

char uuid_str[BLE_UUID_STR_LEN];
char target_uuid_str[BLE_UUID_STR_LEN];


static uint16_t conn_handle;
static uint16_t char_val_handle;

static int discover_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
    const struct ble_gatt_chr *chr, void *arg) {
  if (error->status == 0) {
    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_str);

    ESP_LOGI(TAG, "🔍 Discovered characteristic: UUID=%s, handle=0x%04x", uuid_str, chr->val_handle);

    if (ble_uuid_cmp(&chr->uuid.u, &target_char_uuid.u) == 0) {
      ESP_LOGI(TAG, "🎯 Found target char! Saving handle: 0x%04x", chr->val_handle);
      char_val_handle = chr->val_handle;


      uint8_t data2[] = {0x08, 0x00, 0x81, 0x00, 0x11, 0x51, 0x00, 50};
      //uint8_t data2[] = {0x08, 0x00, 0x81, 0x11, 0x11, 0x51, 0x00, 7}; //  light

      int rc = ble_gattc_write_flat(conn_handle, char_val_handle, data2, sizeof(data2), NULL, NULL);

      if (rc != 0) {
        ESP_LOGE(TAG, "❌ Failed to write: %d", rc);
      } else {
        ESP_LOGI(TAG, "✅ Write successful");
      }



    }
  }

  return 0;
}

void discover_characteristics() {
  ble_gattc_disc_all_chrs(conn_handle, 1, 0xffff, discover_cb, NULL);
}


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
                                     scanning = false;

                                     ble_addr_t *addr = &event->disc.addr;

                                     int rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, ble_gap_event, NULL);

                                     if (rc != 0) {
                                       ESP_LOGE(TAG, "Failed to connect: %d", rc);
                                     } else {
                                       device_connected = true;
                                     }
                                   }
                                 }
                               }
                               return 0;
                             }

    case BLE_GAP_EVENT_CONNECT:
                             if (event->connect.status == 0) {
                               conn_handle = event->connect.conn_handle; 
                               ESP_LOGI(TAG, "Connected to Train Base!");
                               scanning = false;


                               discover_characteristics();
                               /* 
                                  uint8_t data[] = {0x0a, 0x00, 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01}; // setup

                                  int rc = ble_gattc_write_flat(conn_handle, char_val_handle, data, sizeof(data), NULL, NULL);

                                  if (rc != 0) {
                                  ESP_LOGE(TAG, "❌ Failed to write: %d", rc);
                                  } else {
                                  ESP_LOGI(TAG, "✅ Write successful");
                                  }

                                  vTaskDelay(5000 / portTICK_PERIOD_MS);

                                  uint8_t data2[] = {0x08, 0x00, 0x81, 0x00, 0x11, 0x51, 0x00, 50};
                               //uint8_t data2[] = {0x08, 0x00, 0x81, 0x11, 0x11, 0x51, 0x00, 7}; //  light

                               rc = ble_gattc_write_flat(conn_handle, char_val_handle, data2, sizeof(data2), NULL, NULL);


                               if (rc != 0) {
                               ESP_LOGE(TAG, "❌ Failed to write: %d", rc);
                               } else {
                               ESP_LOGI(TAG, "✅ Write successful");
                               }

                               vTaskDelay(5000 / portTICK_PERIOD_MS);
                               */
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

