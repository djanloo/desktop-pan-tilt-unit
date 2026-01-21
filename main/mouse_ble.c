#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "mouse_ble.h"

#define TAG "BLE_MOUSE"
#define MOTOR_PIN 25
#define TARGET_DEVICE_NAME "MS2119M"
#define MAX_NOTIFY_CHARS 4

static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_bd_addr_t mouse_addr;
static bool connect = false;
static bool get_service = false;
static uint16_t conn_id = 0;
static esp_gatt_if_t gl_profile_tab = ESP_GATT_IF_NONE;
static uint16_t service_start_handle = 0;
static uint16_t service_end_handle = 0;
static bool is_connected = false;

// Track notification-enabled characteristics
typedef struct {
    uint16_t char_handle;
    uint16_t cccd_handle;
    bool enabled;
} notify_char_t;

static notify_char_t notify_chars[MAX_NOTIFY_CHARS];
static int notify_char_count = 0;
static int notifications_enabled_count = 0;
static int current_enabling_index = 0;
static bool all_notifications_ready = false;


static void parse_mouse_report(uint8_t *data, uint16_t len) {
    if (len < 7) {
        return;
    }
    
    uint8_t buttons = data[0];
    int8_t x_move = (int8_t)data[1];
    int8_t y_move = (int8_t)data[3];
    int8_t wheel = (int8_t)data[5];
    
    bool left = buttons & 0x01;
    bool right = buttons & 0x02;
    bool middle = buttons & 0x04;
    
    if (buttons != 0 || x_move != 0 || y_move != 0 || wheel != 0) {
        ESP_LOGI(TAG, "Mouse: Btn=0x%02x %s%s%s X=%+3d Y=%+3d Wheel=%+2d", 
                 buttons,
                 left ? "[L]" : "",
                 right ? "[R]" : "",
                 middle ? "[M]" : "",
                 x_move, y_move, wheel);
    }else{
        ESP_LOGI(TAG, "Mouse Release");
        on_release();
    }
    
    static int total_x = 0;
    static int total_y = 0;
    total_x += x_move;
    total_y += y_move;
    
    // static int motor_speed = 128;
    // motor_speed += y_move;
    // if (motor_speed < 0) motor_speed = 0;
    // if (motor_speed > 255) motor_speed = 255;
    
    // if (y_move != 0) {
    //     // set_motor_speed(motor_speed);
    //     ESP_LOGI(TAG, "Position: (%d, %d), Motor: %d", total_x, total_y, motor_speed);
    // }

    if (wheel != 0){
        on_wheel_move(wheel);
    }

    if (left) {
        on_left_button();
    }

    if (right) {
        on_right_button();
    }

    if (middle){
        on_middle_button();
    }
}

static void reset_notification_state(void) {
    notify_char_count = 0;
    notifications_enabled_count = 0;
    current_enabling_index = 0;
    all_notifications_ready = false;
    memset(notify_chars, 0, sizeof(notify_chars));
}

static void enable_next_notification(esp_gatt_if_t gattc_if) {
    if (current_enabling_index >= notify_char_count) {
        ESP_LOGI(TAG, "All notifications enabled!");
        return;
    }
    
    ESP_LOGI(TAG, "Enabling [%d/%d]: Char=0x%04x", 
             current_enabling_index + 1, notify_char_count,
             notify_chars[current_enabling_index].char_handle);
    
    esp_err_t ret = esp_ble_gattc_register_for_notify(gattc_if, mouse_addr, 
                                                      notify_chars[current_enabling_index].char_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register notify failed: %s", esp_err_to_name(ret));
        // Try next one
        current_enabling_index++;
        enable_next_notification(gattc_if);
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Starting scan...");
            esp_ble_gap_start_scanning(0);
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Scanning for %s", TARGET_DEVICE_NAME);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (is_connected || connect) {
                    break;
                }
                
                uint8_t *adv_name = NULL;
                uint8_t adv_name_len = 0;
                
                adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_CMPL,
                                                    &adv_name_len);
                
                if (adv_name == NULL) {
                    adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                        ESP_BLE_AD_TYPE_NAME_SHORT,
                                                        &adv_name_len);
                }
                
                if (adv_name != NULL) {
                    char dev_name[32] = {0};
                    memcpy(dev_name, adv_name, adv_name_len < 31 ? adv_name_len : 31);
                    
                    if (strcmp(dev_name, TARGET_DEVICE_NAME) == 0) {
                        ESP_LOGI(TAG, "Found %s, connecting...", dev_name);
                        
                        esp_ble_gap_stop_scanning();
                        memcpy(mouse_addr, param->scan_rst.bda, sizeof(esp_bd_addr_t));
                        connect = true;
                        
                        esp_err_t ret = esp_ble_gattc_open(gl_profile_tab, 
                                                           param->scan_rst.bda, 
                                                           param->scan_rst.ble_addr_type, 
                                                           true);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Connect failed: %s", esp_err_to_name(ret));
                            connect = false;
                            esp_ble_gap_start_scanning(0);
                        }
                    }
                }
            }
            break;
            
        case ESP_GAP_BLE_SEC_REQ_EVT:
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
            
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(TAG, "Authentication OK");
            }
            break;
            
        default:
            break;
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, 
                                       esp_gatt_if_t gattc_if, 
                                       esp_ble_gattc_cb_param_t *param) {
    esp_err_t ret;
    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(TAG, "GATT client registered");
            gl_profile_tab = gattc_if;
            
            // Claude
            // esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
            // Chatgpt
            esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;


            esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
            uint8_t key_size = 16;
            uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
            uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
            
            esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
            esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
            esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
            esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
            esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
            
            ret = esp_ble_gap_set_scan_params(&(esp_ble_scan_params_t){
                .scan_type = BLE_SCAN_TYPE_ACTIVE,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
                .scan_interval = 0x50,
                .scan_window = 0x30,
                .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
            });
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Set scan params failed");
            }
            break;
            
        case ESP_GATTC_CONNECT_EVT:
            //ChatGPT
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);


            ESP_LOGI(TAG, "========================================");
            if (all_notifications_ready) {
                ESP_LOGI(TAG, "RECONNECTED - re-enabling notifications");
                ESP_LOGI(TAG, "========================================");
            } else {
                ESP_LOGI(TAG, "CONNECTED - first time");
                ESP_LOGI(TAG, "========================================");
            }
            
            conn_id = param->connect.conn_id;
            is_connected = true;
            
            // Update connection parameters
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x10;
            conn_params.max_int = 0x20;
            conn_params.latency = 0;
            conn_params.timeout = 400;
            esp_ble_gap_update_conn_params(&conn_params);
            
            // If we've already discovered services before, re-enable notifications
            if (all_notifications_ready && notify_char_count > 0) {
                // Reset for re-enabling
                notifications_enabled_count = 0;
                current_enabling_index = 0;
                for (int i = 0; i < notify_char_count; i++) {
                    notify_chars[i].enabled = false;
                }
                
                // Start enabling notifications one by one
                enable_next_notification(gattc_if);
            }
            break;
            
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Open failed: %d", param->open.status);
                is_connected = false;
                connect = false;
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_ble_gap_start_scanning(0);
                break;
            }
            
            conn_id = param->open.conn_id;
            is_connected = true;
            
            // If this is the first connection, discover services
            if (!all_notifications_ready) {
                ESP_LOGI(TAG, "First connection - discovering services...");
                get_service = false;
                esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
            }
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT:
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
                param->search_res.srvc_id.uuid.uuid.uuid16 == 0x1812) {
                ESP_LOGI(TAG, "Found HID Service");
                service_start_handle = param->search_res.start_handle;
                service_end_handle = param->search_res.end_handle;
                get_service = true;
            }
            break;
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (!get_service) {
                ESP_LOGE(TAG, "HID Service not found!");
                break;
            }
            
            ESP_LOGI(TAG, "Service discovery complete");
            
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if, conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    service_start_handle,
                                                                    service_end_handle,
                                                                    0, &count);
            
            if (status != ESP_GATT_OK || count == 0) {
                ESP_LOGE(TAG, "No characteristics found");
                break;
            }
            
            char_elem_result = malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (!char_elem_result) {
                ESP_LOGE(TAG, "Malloc failed");
                break;
            }
            
            status = esp_ble_gattc_get_all_char(gattc_if, conn_id, service_start_handle,
                                               service_end_handle, char_elem_result, &count, 0);
            
            if (status != ESP_GATT_OK) {
                free(char_elem_result);
                char_elem_result = NULL;
                break;
            }
            
            // Reset notification tracking
            reset_notification_state();
            
            ESP_LOGI(TAG, "Finding HID Report characteristics...");
            
            // Find all HID Report characteristics with NOTIFY
            for (int i = 0; i < count && notify_char_count < MAX_NOTIFY_CHARS; i++) {
                if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 &&
                    char_elem_result[i].uuid.uuid.uuid16 == 0x2A4D &&
                    (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                    
                    uint16_t char_handle = char_elem_result[i].char_handle;
                    
                    // Get CCCD descriptor
                    uint16_t desc_count = 0;
                    esp_ble_gattc_get_attr_count(gattc_if, conn_id, ESP_GATT_DB_DESCRIPTOR,
                                                service_start_handle, service_end_handle,
                                                char_handle, &desc_count);
                    
                    if (desc_count > 0) {
                        esp_gattc_descr_elem_t *descr_result = malloc(sizeof(esp_gattc_descr_elem_t) * desc_count);
                        if (descr_result) {
                            esp_ble_gattc_get_all_descr(gattc_if, conn_id, char_handle,
                                                       descr_result, &desc_count, 0);
                            
                            for (int j = 0; j < desc_count; j++) {
                                if (descr_result[j].uuid.uuid.uuid16 == 0x2902) {
                                    // Found CCCD - save it
                                    notify_chars[notify_char_count].char_handle = char_handle;
                                    notify_chars[notify_char_count].cccd_handle = descr_result[j].handle;
                                    notify_chars[notify_char_count].enabled = false;
                                    
                                    ESP_LOGI(TAG, "HID Report [%d]: Char=0x%04x, CCCD=0x%04x",
                                            notify_char_count, char_handle, descr_result[j].handle);
                                    
                                    notify_char_count++;
                                    break;
                                }
                            }
                            
                            free(descr_result);
                        }
                    }
                }
            }
            
            free(char_elem_result);
            char_elem_result = NULL;
            
            ESP_LOGI(TAG, "Found %d HID Report characteristics", notify_char_count);
            
            // Now enable notifications one by one
            if (notify_char_count > 0) {
                current_enabling_index = 0;
                enable_next_notification(gattc_if);
            }
            break;

        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            if (param->reg_for_notify.status == ESP_GATT_OK) {
                // Find which characteristic this is for
                for (int i = 0; i < notify_char_count; i++) {
                    if (notify_chars[i].char_handle == param->reg_for_notify.handle) {
                        ESP_LOGI(TAG, "Register notify OK for 0x%04x", notify_chars[i].char_handle);
                        
                        // Special handling for characteristic 0x0019 - read it first
                        if (notify_chars[i].char_handle == 0x0019) {
                            ESP_LOGI(TAG, "Reading 0x0019 before enabling notifications...");
                            ret = esp_ble_gattc_read_char(gattc_if, conn_id, 
                                                        notify_chars[i].char_handle,
                                                        ESP_GATT_AUTH_REQ_NONE);
                            if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "Read char failed: %s", esp_err_to_name(ret));
                                // Try to write CCCD anyway
                                goto write_cccd;
                            }
                            // Will write CCCD after read completes in ESP_GATTC_READ_CHAR_EVT
                        } else {
        write_cccd:
                            ESP_LOGI(TAG, "Writing CCCD for 0x%04x...", notify_chars[i].char_handle);
                            
                            uint16_t notify_en = 1;
                            ret = esp_ble_gattc_write_char_descr(gattc_if, conn_id, 
                                                                notify_chars[i].cccd_handle,
                                                                sizeof(notify_en), 
                                                                (uint8_t *)&notify_en,
                                                                ESP_GATT_WRITE_TYPE_RSP, 
                                                                ESP_GATT_AUTH_REQ_NONE);
                            if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "Write CCCD failed: %s", esp_err_to_name(ret));
                                current_enabling_index++;
                                enable_next_notification(gattc_if);
                            }
                        }
                        break;
                    }
                }
            } else {
                ESP_LOGE(TAG, "Register for notify failed for 0x%04x", 
                        param->reg_for_notify.handle);
                current_enabling_index++;
                enable_next_notification(gattc_if);
            }
            break;
            
        case ESP_GATTC_WRITE_DESCR_EVT:
            ESP_LOGI(TAG, "Write CCCD event - handle: 0x%04x, status: %d", 
                    param->write.handle, param->write.status);
            
            if (param->write.status == ESP_GATT_OK) {
                // Find which characteristic was enabled
                for (int i = 0; i < notify_char_count; i++) {
                    if (notify_chars[i].cccd_handle == param->write.handle) {
                        notify_chars[i].enabled = true;
                        notifications_enabled_count++;
                        
                        ESP_LOGI(TAG, "✓ Enabled [%d/%d]: 0x%04x", 
                                notifications_enabled_count, notify_char_count,
                                notify_chars[i].char_handle);
                        
                        // Check if all are enabled
                        if (notifications_enabled_count >= notify_char_count) {
                            ESP_LOGI(TAG, "========================================");
                            ESP_LOGI(TAG, "✓✓✓ ALL NOTIFICATIONS ENABLED ✓✓✓");
                            ESP_LOGI(TAG, "🖱️  MOUSE IS READY!");
                            ESP_LOGI(TAG, "========================================");
                            all_notifications_ready = true;
                        } else {
                            // Enable next notification
                            current_enabling_index++;
                            enable_next_notification(gattc_if);
                        }
                        break;
                    }
                }
            } else {
                // Log the actual error status
                ESP_LOGE(TAG, "Write CCCD FAILED for 0x%04x - Error status: 0x%02x", 
                        param->write.handle, param->write.status);
                
                // Common GATT error codes:
                // 0x01 = Invalid Handle
                // 0x02 = Read Not Permitted
                // 0x03 = Write Not Permitted
                // 0x04 = Invalid PDU
                // 0x05 = Insufficient Authentication
                // 0x06 = Request Not Supported
                // 0x07 = Invalid Offset
                // 0x08 = Insufficient Authorization
                // 0x0D = Invalid Attribute Value Length
                // 0x0E = Unlikely Error
                // 0x0F = Insufficient Encryption
                
                if (param->write.status == 0x03) {
                    ESP_LOGE(TAG, "Error: Write Not Permitted - mouse may not allow this");
                } else if (param->write.status == 0x05) {
                    ESP_LOGE(TAG, "Error: Insufficient Authentication");
                } else if (param->write.status == 0x08) {
                    ESP_LOGE(TAG, "Error: Insufficient Authorization");
                }
                
                // Try next one anyway
                current_enabling_index++;
                enable_next_notification(gattc_if);
            }
            break;
        
        case ESP_GATTC_READ_CHAR_EVT:
            ESP_LOGI(TAG, "Read char event, handle: 0x%04x, status: %d", 
                    param->read.handle, param->read.status);
            
            if (param->read.status == ESP_GATT_OK && param->read.handle == 0x0019) {
                ESP_LOGI(TAG, "Read 0x0019 successful, now writing CCCD...");
                
                // Find the CCCD handle for 0x0019
                for (int i = 0; i < notify_char_count; i++) {
                    if (notify_chars[i].char_handle == 0x0019) {
                        uint16_t notify_en = 1;
                        ret = esp_ble_gattc_write_char_descr(gattc_if, conn_id, 
                                                            notify_chars[i].cccd_handle,
                                                            sizeof(notify_en), 
                                                            (uint8_t *)&notify_en,
                                                            ESP_GATT_WRITE_TYPE_RSP, 
                                                            ESP_GATT_AUTH_REQ_NONE);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Write CCCD failed: %s", esp_err_to_name(ret));
                            current_enabling_index++;
                            enable_next_notification(gattc_if);
                        }
                        break;
                    }
                }
            } else {
                ESP_LOGE(TAG, "Read char failed, trying to write CCCD anyway...");
                // Try to write anyway
                for (int i = 0; i < notify_char_count; i++) {
                    if (notify_chars[i].char_handle == param->read.handle) {
                        uint16_t notify_en = 1;
                        ret = esp_ble_gattc_write_char_descr(gattc_if, conn_id, 
                                                            notify_chars[i].cccd_handle,
                                                            sizeof(notify_en), 
                                                            (uint8_t *)&notify_en,
                                                            ESP_GATT_WRITE_TYPE_RSP, 
                                                            ESP_GATT_AUTH_REQ_NONE);
                        if (ret != ESP_OK) {
                            current_enabling_index++;
                            enable_next_notification(gattc_if);
                        }
                        break;
                    }
                }
            }
            break;
            
        case ESP_GATTC_NOTIFY_EVT:
            parse_mouse_report(param->notify.value, param->notify.value_len);
            break;
            
        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "Disconnected (reason: %d)", param->disconnect.reason);
            ESP_LOGI(TAG, "========================================");
            
            is_connected = false;
            connect = false;
            // Keep all_notifications_ready and notify_chars[] - we'll reuse them
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "Restarting scan...");
            esp_ble_gap_start_scanning(0);
            break;
            
        default:
            break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, 
                        esp_ble_gattc_cb_param_t *param) {
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gattc_profile_event_handler(event, gattc_if, param);
        }
    } else {
        gattc_profile_event_handler(event, gattc_if, param);
    }
}

void init_ble(){
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "BLE Mouse Controller");
    ESP_LOGI(TAG, "Target: %s", TARGET_DEVICE_NAME);
    ESP_LOGI(TAG, "========================================");

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(0));

    ESP_LOGI(TAG, "Ready!");
}

