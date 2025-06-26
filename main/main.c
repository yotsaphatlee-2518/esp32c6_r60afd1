#ifndef DEVICE_ID
#define DEVICE_ID "falldetector"
#endif
char g_device_id[32] = DEVICE_ID; // Default Device ID (สามารถ override ได้ตอน build)
#define RESET_BUTTON_GPIO    0      // ตัวอย่าง: ใช้ GPIO0 (เปลี่ยนตามที่ต่อจริง)
#define RESET_HOLD_TIME_MS   3000   // ตัวอย่าง: กดค้าง 3 วินาที (3000 ms)
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "cJSON.h"
#include <inttypes.h>
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "wifiManager.h"
#include "wifiManager_private.h"
#include "ota_update.h" // เพิ่ม include OTA
#include <time.h>       // <-- เพิ่มบรรทัดนี้
#include "driver/gpio.h" // เพิ่มสำหรับใช้งาน GPIO

// เพิ่ม extern สำหรับ certificate
extern const uint8_t server1_crt_start[] asm("_binary_server1_crt_start");
extern const uint8_t server1_crt_end[]   asm("_binary_server1_crt_end");

// Add these function prototypes before mqtt_event_handler
void update_installation_angles(int16_t angle_x, int16_t angle_y, int16_t angle_z);
void update_installation_height(uint16_t new_height);
void update_fall_detection_sensitivity(uint8_t new_sensitivity);
void update_fall_duration(uint32_t new_duration);
void update_fall_breaking_height(uint16_t new_height);
void update_sitting_still_distance(uint16_t new_distance);
void update_moving_distance(uint16_t new_distance);
void update_stay_still_switch(bool enable);
void update_stay_still_duration(uint32_t new_duration);
void enable_human_presence_detection(bool enable);
void wifi_reset_button_task(void *arg);

// Add these function prototypes after the other prototypes
void save_settings_to_nvs(void);
void load_settings_from_nvs(void);

// Add this function prototype with the other prototypes at the top of the file
void reboot_device(void);

// Forward declaration of the send_query function
void send_query(uint8_t control, uint8_t command, uint8_t data_payload);

// Add forward declaration for mqtt_publish_settings
void mqtt_publish_settings(void);

// Function prototypes for MQTT publishing functions
void mqtt_publish_product_info(void);

// Add these prototypes with the other function declarations at the top of the file
void update_height_accumulation_time(uint32_t seconds);
void update_non_presence_time(uint32_t seconds);

// Add this function prototype with the other prototypes at the top
void init_nvs(void);

// Add function prototypes for NVS device ID management
void save_device_id_to_nvs(void);
void load_device_id_from_nvs(void);

// ---------------------- UART and Frame Definitions ----------------------
#define UART_PORT_NUM      UART_NUM_1
#define BUF_SIZE           1024

#define FRAME_HEADER0      0x53
#define FRAME_HEADER1      0x59
#define FRAME_TAIL0        0x54
#define FRAME_TAIL1        0x43

// ---------------------- Device Identification ----------------------
#define DEVICE_TYPE     "R60AFD1"

// ---------------------- Global Variables ----------------------

// Live Data (change frequently)
volatile bool    g_presence         = false;   // Human presence
volatile bool    g_fall_alarm       = false;   // Fall detection
volatile bool    g_stay_still_alarm = false;   // Stay-still alarm
volatile uint8_t g_movement_state   = 0;       // 0: No movement, 1: Static, 2: Active
volatile uint8_t g_body_movement_param = 0;     // Body movement parameter
volatile uint8_t g_heartbeat        = 0;       // Heartbeat value

// Trajectory points (each coordinate is 2 bytes)
volatile int16_t g_traj_x = 0;
volatile int16_t g_traj_y = 0;

// Height Proportion Report values (live measurement)
volatile uint16_t g_total_height_count = 0;     
volatile uint8_t  g_height_prop_0_0_5  = 0;
volatile uint8_t  g_height_prop_0_5_1  = 0;
volatile uint8_t  g_height_prop_1_1_5  = 0;
volatile uint8_t  g_height_prop_1_5_2  = 0;

// Non-presence time (in seconds)
volatile uint32_t g_non_presence_time = 0;

// Settings (configuration parameters, change infrequently)
// Working status report (spontaneous or query reply)
volatile uint8_t g_working_status   = 0;
// Scenario Report
volatile uint8_t g_scenario         = 0;
// Installation configuration reports:
volatile int16_t g_installation_angle_x = 0;
volatile int16_t g_installation_angle_y = 0;
volatile int16_t g_installation_angle_z = 0;
volatile uint16_t g_installation_height   = 0;

// Fall detection parameters
volatile uint8_t g_fall_detection_sensitivity = 0;
volatile uint32_t g_fall_duration = 0;
volatile uint16_t g_fall_breaking_height = 0;

// ---------------------- New: Distance Settings ----------------------
// Sitting-still and Moving Horizontal Distance (both in cm)
volatile uint16_t g_sitting_still_distance = 0;
volatile uint16_t g_moving_distance        = 0;

// ---------------------- New: Operating Time ----------------------
volatile uint32_t g_operating_time = 0; // Operating time in seconds (queried from device)

// ---------------------- Product Information ----------------------
// Global arrays to store queried product info (filled in by the device)
#define PRODUCT_STR_LEN 32
char g_product_model[PRODUCT_STR_LEN]    = {0};
char g_product_id[PRODUCT_STR_LEN]       = {0};
char g_hardware_model[PRODUCT_STR_LEN]   = {0};
char g_firmware_version[PRODUCT_STR_LEN] = {0};

// Add this with the other global variables in the "Settings" section
volatile bool g_stay_still_switch = true;
volatile uint32_t g_stay_still_duration = 0;

// Add this with the other global variables in the "Live Data" section
volatile uint32_t g_height_accumulation_time = 0;  // Default value

// Add this with the other global variables in the "Settings" section
volatile bool g_fall_detection_switch = true;  // Default to enabled

// Add this function prototype with the other prototypes at the top
void update_fall_detection_switch(bool enable);

// Add this function implementation with the other command functions
void update_fall_detection_switch(bool enable) {
    // Frame structure (10 bytes total):
    // Header (2) + Control (1) + Command (1) + Length (2) + Payload (1) + Check (1) + Tail (2)
    uint8_t frame[10];
    frame[0] = FRAME_HEADER0;   // 0x53
    frame[1] = FRAME_HEADER1;   // 0x59
    frame[2] = 0x83;            // Control for fall detection settings
    frame[3] = 0x01;            // Command for fall detection switch
    frame[4] = 0x00;            // Length high byte
    frame[5] = 0x01;            // Length low byte (1 byte payload)
    frame[6] = enable ? 0x01 : 0x00; // Payload: 0x01 to enable, 0x00 to disable

    // Calculate checksum
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += frame[i];
    }
    frame[7] = sum & 0xFF;

    // Append tail
    frame[8] = FRAME_TAIL0;    // 0x54
    frame[9] = FRAME_TAIL1;    // 0x43

    // Send frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Fall Detection %s\n", enable ? "ENABLED" : "DISABLED");

    // Update global variable
    g_fall_detection_switch = enable;
}

// Helper function to convert movement state to a readable string
const char* movement_state_str(uint8_t state) {
    switch(state) {
        case 0: return "No movement";
        case 1: return "Static";
        case 2: return "Active";
        default: return "Unknown";
    }
}

// ---------------------- MQTT Configurations ----------------------
#define MQTT_BROKER_URI      "mqtts://dev-connect.datatamer.ai:8884"
#define MQTT_USERNAME        "client"
#define MQTT_PASSWORD        "Apollo1999!"
// send reading live data
char mqtt_topic_live[64];
// request reading product info
char mqtt_topic_info_device_id[64];
// send reading product info
#define MQTT_TOPIC_INFO      "R60AFD1/info"

// OTA update topic
char mqtt_topic_ota_update[64];

// update settings
char mqtt_topic_settings_update[64];
// request reading settings
char mqtt_topic_settings_state_device_id[64];
// send reading settings
#define MQTT_TOPIC_SETTINGS_STATE "R60AFD1/settings_state"

static const char *MQTT_TAG = "mqtt_client";
esp_mqtt_client_handle_t mqtt_client = NULL;

// Forward declaration for the get_live_json_payload_str function
char* get_live_json_payload_str(void);

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT Connected to broker");
            // Subscribe to settings topic when connected
            int msg_id = esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_settings_update, 0);
            ESP_LOGI(MQTT_TAG, "Subscribed to %s, msg_id=%d", mqtt_topic_settings_update, msg_id);
            msg_id = esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_info_device_id, 0);
            ESP_LOGI(MQTT_TAG, "Subscribed to %s, msg_id=%d", mqtt_topic_info_device_id, msg_id);
            msg_id = esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_settings_state_device_id, 0);
            ESP_LOGI(MQTT_TAG, "Subscribed to %s, msg_id=%d", mqtt_topic_settings_state_device_id, msg_id);
            // Subscribe to the OTA topic
            msg_id = esp_mqtt_client_subscribe(mqtt_client, mqtt_topic_ota_update, 0);
            ESP_LOGI(MQTT_TAG, "Subscribed to %s, msg_id=%d", mqtt_topic_ota_update, msg_id);
            
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT Disconnected from broker");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(MQTT_TAG, "MQTT Message published successfully, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(MQTT_TAG, "MQTT Data received:");
            ESP_LOGI(MQTT_TAG, "Topic: %.*s", event->topic_len, event->topic);
            ESP_LOGI(MQTT_TAG, "Data: %.*s", event->data_len, event->data);
            
            // If the message is on the settings topic, parse and update settings.
            if (event->topic_len == strlen(mqtt_topic_settings_update) &&
                strncmp(event->topic, mqtt_topic_settings_update, event->topic_len) == 0) {
                
                printf("Received settings update: %.*s\n", event->data_len, event->data);
                cJSON *json = cJSON_Parse(event->data);
                if(json == NULL) {
                    ESP_LOGE(MQTT_TAG, "Error parsing settings JSON");
                } else {
                    cJSON *item = NULL;
                    
                    // Add ability to set a new device ID
                    item = cJSON_GetObjectItem(json, "set_device_id");
                    if(item && cJSON_IsString(item) && (item->valuestring != NULL)) {
                        printf("Received new device ID: %s\n", item->valuestring);
                        strncpy(g_device_id, item->valuestring, sizeof(g_device_id) - 1);
                        g_device_id[sizeof(g_device_id) - 1] = '\0'; // Ensure null termination
                        save_device_id_to_nvs();
                        printf("New device ID saved. Rebooting in 3 seconds...\n");
                        cJSON_Delete(json);
                        vTaskDelay(pdMS_TO_TICKS(3000));
                        esp_restart();
                        return;
                    }
                    
                    // Check for reboot command first
                    item = cJSON_GetObjectItem(json, "reboot");
                    if(item && cJSON_IsBool(item) && item->valueint) {
                        printf("Reboot command received\n");
                        cJSON_Delete(json);
                        reboot_device();
                        return;
                    }
                    
                    // Remove the first duplicate non-presence time update here
                    // item = cJSON_GetObjectItem(json, "non_presence_time");
                    // if(item && cJSON_IsNumber(item)) {
                    //     g_non_presence_time = (uint32_t)item->valueint;
                    //     printf("Updated non-presence time to: %" PRIu32 " seconds\n", g_non_presence_time);
                    // }
                    
                    // Update installation angles
                    int16_t angle_x = 0, angle_y = 0, angle_z = 0;
                    item = cJSON_GetObjectItem(json, "installation_angle_x");
                    if(item && cJSON_IsNumber(item)) {
                        angle_x = item->valueint;
                    }
                    item = cJSON_GetObjectItem(json, "installation_angle_y");
                    if(item && cJSON_IsNumber(item)) {
                        angle_y = item->valueint;
                    }
                    item = cJSON_GetObjectItem(json, "installation_angle_z");
                    if(item && cJSON_IsNumber(item)) {
                        angle_z = item->valueint;
                    }
                    update_installation_angles(angle_x, angle_y, angle_z);
                    
                    // Update installation height
                    printf("Checking installation height update\n");
                    item = cJSON_GetObjectItem(json, "installation_height");
                    if(item && cJSON_IsNumber(item)) {
                        uint16_t new_height = (uint16_t)item->valueint;
                        printf("Updating installation height to: %d cm\n", new_height);
                        update_installation_height(new_height);
                    } else {
                        printf("Installation height update skipped: invalid or missing value\n");
                    }
                    
                    // Update fall detection sensitivity
                    item = cJSON_GetObjectItem(json, "fall_detection_sensitivity");
                    if(item && cJSON_IsNumber(item)) {
                        update_fall_detection_sensitivity((uint8_t)item->valueint);
                    }
                    
                    // Update fall duration
                    item = cJSON_GetObjectItem(json, "fall_duration");
                    if(item && cJSON_IsNumber(item)) {
                        update_fall_duration((uint32_t)item->valueint);
                    }
                    
                    // Update fall breaking height
                    item = cJSON_GetObjectItem(json, "fall_breaking_height");
                    if(item && cJSON_IsNumber(item)) {
                        update_fall_breaking_height((uint16_t)item->valueint);
                    }
                    
                    // Update sitting-still horizontal distance
                    item = cJSON_GetObjectItem(json, "sitting_still_distance");
                    if(item && cJSON_IsNumber(item)) {
                        update_sitting_still_distance((uint16_t)item->valueint);
                    }
                    
                    // Update moving horizontal distance
                    item = cJSON_GetObjectItem(json, "moving_distance");
                    if(item && cJSON_IsNumber(item)) {
                        update_moving_distance((uint16_t)item->valueint);
                    }
                    
                    // Update stay-still alarm switch
                    item = cJSON_GetObjectItem(json, "stay_still_switch");
                    if(item && cJSON_IsBool(item)) {
                        update_stay_still_switch(item->valueint ? true : false);
                    }
                    
                    // Update stay-still duration
                    item = cJSON_GetObjectItem(json, "stay_still_duration");
                    if(item && cJSON_IsNumber(item)) {
                        update_stay_still_duration((uint32_t)item->valueint);
                    }
                    
                    // Add fall detection switch update
                    item = cJSON_GetObjectItem(json, "fall_detection_switch");
                    if(item && cJSON_IsBool(item)) {
                        update_fall_detection_switch(item->valueint ? true : false);
                    }
                    // height accumulation time
                    item = cJSON_GetObjectItem(json, "height_accumulation_time");
                    if(item && cJSON_IsNumber(item)) {
                        update_height_accumulation_time((uint32_t)item->valueint);
                    }
                    // non-presence time (keep this one, which is already further down in the function)
                    item = cJSON_GetObjectItem(json, "non_presence_time");
                    if(item && cJSON_IsNumber(item)) {
                        update_non_presence_time((uint32_t)item->valueint);
                    }
                    cJSON_Delete(json);
                    save_settings_to_nvs();
                    mqtt_publish_settings();
                }
            }
            // Check if the topic is MQTT_TOPIC_SETTINGS_STATE_DEVICE_ID
            else if (event->topic_len == strlen(mqtt_topic_settings_state_device_id) &&
                     strncmp(event->topic, mqtt_topic_settings_state_device_id, event->topic_len) == 0) {
                
                printf("Received request for settings state on device-specific topic\n");
                mqtt_publish_settings();
            }
            // Check if the topic is MQTT_TOPIC_INFO_DEVICE_ID
            else if (event->topic_len == strlen(mqtt_topic_info_device_id) &&
                     strncmp(event->topic, mqtt_topic_info_device_id, event->topic_len) == 0) {
                
                printf("Received request for product info on device-specific topic\n");
                mqtt_publish_product_info();
            }
            // เพิ่ม trigger OTA ผ่าน MQTT topic
            else if (event->topic_len == strlen(mqtt_topic_ota_update) &&
                     strncmp(event->topic, mqtt_topic_ota_update, event->topic_len) == 0) {
                // รับ URL OTA จาก payload
                char url[128] = {0};
                int len = event->data_len < 127 ? event->data_len : 127;
                strncpy(url, event->data, len);
                url[len] = '\0';

                // --- เพิ่มโค้ดนี้เพื่อตัด \n, \r, space ข้างหน้าและข้างหลังออก ---
                char *start = url;
                while (*start == '\n' || *start == '\r' || *start == ' ') start++;
                char *end = start + strlen(start) - 1;
                while (end > start && (*end == '\n' || *end == '\r' || *end == ' ')) {
                    *end = '\0';
                    end--;
                }
                memmove(url, start, strlen(start) + 1); // ขยับ string ไปต้น buffer
                // -------------------------------------------------------------

                printf("[OTA] Trigger OTA update from MQTT: %s\n", url);
                printf("OTA URL raw: [%s]\n", url);
                for (int i = 0; i < strlen(url); i++) {
                    printf("%02X ", (unsigned char)url[i]);
                }
                printf("\n");
                ota_update_start(url);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(MQTT_TAG, "MQTT Error occurred");
            break;
        default:
            break;
    }
}

// Initialize MQTT client
void mqtt_init(void)
{
    esp_log_level_set("mbedtls", ESP_LOG_VERBOSE); // Add this line for detailed TLS logs
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
            },
            .verification = {
                .certificate = (const char *)server1_crt_start,
            },
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication = {
                .password = MQTT_PASSWORD,
            },
        },
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// Publish live data to MQTT
void mqtt_publish_live_data(void)
{
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    // printf("[DEBUG] mqtt_publish_live_data() called at %04d-%02d-%02d %02d:%02d:%02d\n",
    //        timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
    //        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    char *json_str = get_live_json_payload_str();
    if (json_str) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, mqtt_topic_live, json_str, 0, 1, 0);
        if (msg_id != -1) {
            printf("Published live data to %s\n", mqtt_topic_live);
        } else {
            printf("Failed to publish live data\n");
        }
        free(json_str);
    }
}

// Get live JSON payload as a string (caller must free)
char* get_live_json_payload_str(void)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        printf("Error creating live JSON object\n");
        return NULL;
    }

    // Add device identification
    cJSON_AddStringToObject(json, "device_id", g_device_id);
    cJSON_AddStringToObject(json, "device_type", DEVICE_TYPE);

    cJSON_AddBoolToObject(json, "presence", g_presence);
    cJSON_AddBoolToObject(json, "fall_alarm", g_fall_alarm);
    cJSON_AddBoolToObject(json, "stay_still_alarm", g_stay_still_alarm);
    cJSON_AddNumberToObject(json, "movement_state", g_movement_state);
    cJSON_AddNumberToObject(json, "body_movement_param", g_body_movement_param);
    cJSON_AddNumberToObject(json, "heartbeat", g_heartbeat);
    cJSON_AddNumberToObject(json, "trajectory_x", g_traj_x);
    cJSON_AddNumberToObject(json, "trajectory_y", g_traj_y);
    cJSON_AddNumberToObject(json, "total_height_count", g_total_height_count);
    cJSON_AddNumberToObject(json, "height_prop_0_0_5", g_height_prop_0_0_5);
    cJSON_AddNumberToObject(json, "height_prop_0_5_1", g_height_prop_0_5_1);
    cJSON_AddNumberToObject(json, "height_prop_1_1_5", g_height_prop_1_1_5);
    cJSON_AddNumberToObject(json, "height_prop_1_5_2", g_height_prop_1_5_2);
    cJSON_AddNumberToObject(json, "non_presence_time", g_non_presence_time);

    char *json_str = cJSON_Print(json);
    cJSON_Delete(json);
    return json_str;
}

// Modified print_live_json_payload to use the get_live_json_payload_str function
void print_live_json_payload(void)
{
    char *json_str = get_live_json_payload_str();
    if(json_str) {
        printf("Live JSON Payload: %s\n", json_str);
        free(json_str);
    }
}

// Task to publish MQTT messages periodically
void mqtt_publish_task(void *arg)
{
    while(1) {
        if (mqtt_client != NULL) {
            mqtt_publish_live_data();
        }
        vTaskDelay(pdMS_TO_TICKS(60000)); // Publish every 60 seconds (1 นาที)
    }
}

// Get settings JSON payload as a string (caller must free)
char* get_settings_json_payload_str(void)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        printf("Error creating settings JSON object\n");
        return NULL;
    }
    
    // Add device identification
    cJSON_AddStringToObject(json, "device_id", g_device_id);
    cJSON_AddStringToObject(json, "device_type", DEVICE_TYPE);
    
    cJSON_AddNumberToObject(json, "working_status", g_working_status);
    cJSON_AddNumberToObject(json, "scenario", g_scenario);
    cJSON_AddNumberToObject(json, "installation_angle_x", g_installation_angle_x);
    cJSON_AddNumberToObject(json, "installation_angle_y", g_installation_angle_y);
    cJSON_AddNumberToObject(json, "installation_angle_z", g_installation_angle_z);
    cJSON_AddNumberToObject(json, "installation_height", g_installation_height);
    
    // Add fall detection parameters
    cJSON_AddNumberToObject(json, "fall_detection_sensitivity", g_fall_detection_sensitivity);
    cJSON_AddNumberToObject(json, "fall_duration", g_fall_duration);
    cJSON_AddNumberToObject(json, "fall_breaking_height", g_fall_breaking_height);
    
    // Distance Settings
    cJSON_AddNumberToObject(json, "sitting_still_distance", g_sitting_still_distance);
    cJSON_AddNumberToObject(json, "moving_distance", g_moving_distance);
    
    // Add stay-still parameters
    cJSON_AddBoolToObject(json, "stay_still_switch", g_stay_still_switch);
    cJSON_AddNumberToObject(json, "stay_still_duration", g_stay_still_duration);
    cJSON_AddNumberToObject(json, "height_accumulation_time", g_height_accumulation_time);
    cJSON_AddBoolToObject(json, "fall_detection_switch", g_fall_detection_switch);
    cJSON_AddNumberToObject(json, "non_presence_time", g_non_presence_time);
    
    char *json_str = cJSON_Print(json);
    cJSON_Delete(json);
    return json_str;
}

// Function to publish settings to MQTT
void mqtt_publish_settings(void)
{
    char *json_str = get_settings_json_payload_str();
    if (json_str) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_SETTINGS_STATE, json_str, 0, 1, 0);
        if (msg_id != -1) {
            printf("Published settings to %s\n", MQTT_TOPIC_SETTINGS_STATE);
        } else {
            printf("Failed to publish settings\n");
        }
        free(json_str);
    }
}

// Modified print_settings_json_payload to use the get_settings_json_payload_str function
void print_settings_json_payload(void)
{
    char *json_str = get_settings_json_payload_str();
    if(json_str) {
        printf("******************************************************** \n");
        printf("Settings JSON Payload: %s\n", json_str);
        free(json_str);
        printf("********************************************************\n");
    }
}

// Product Information JSON payload
void print_product_info_payload(void)
{
    cJSON *json = cJSON_CreateObject();
    if(json == NULL) {
        printf("Error creating product info JSON object\n");
        return;
    }
    
    // Add device identification
    cJSON_AddStringToObject(json, "device_id", g_device_id);
    cJSON_AddStringToObject(json, "device_type", DEVICE_TYPE);
    
    cJSON_AddStringToObject(json, "product_model", g_product_model);
    cJSON_AddStringToObject(json, "product_id", g_product_id);
    cJSON_AddStringToObject(json, "hardware_model", g_hardware_model);
    cJSON_AddStringToObject(json, "firmware_version", g_firmware_version);

    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("========================================================\n");
        printf("Product Information JSON Payload: %s\n", json_str);
        printf("========================================================\n");
        free(json_str);
    }
    cJSON_Delete(json);
}

// Usage/Operating Time JSON payload: usage or operating time of the device.
void print_usage_json_payload(void)
{
    cJSON *json = cJSON_CreateObject();
    if(json == NULL) {
        printf("Error creating usage JSON object\n");
        return;
    }
    
    // Add device identification
    cJSON_AddStringToObject(json, "device_id", g_device_id);
    cJSON_AddStringToObject(json, "device_type", DEVICE_TYPE);
    
    cJSON_AddNumberToObject(json, "operating_time", g_operating_time);
    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("Usage/Operating Time JSON Payload: %s\n", json_str);
        free(json_str);
    }
    cJSON_Delete(json);
}

// Initialize UART (update GPIO numbers as needed)
void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    // Set TX and RX pins (adjust these GPIO numbers for your hardware)
    uart_set_pin(UART_PORT_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// This task continuously reads UART data and parses incoming frames
void uart_read_task(void *arg)
{
    uint8_t data[BUF_SIZE];
    uint32_t last_height_prop_request = 0; // Time (in ms) when we last requested height proportion data

    while(1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if(len > 0) {
            int i = 0;
            while(i < len) {
                // Look for the frame header
                if(data[i] == FRAME_HEADER0 && (i + 1) < len && data[i+1] == FRAME_HEADER1) {
                    // Ensure at least 6 bytes are available (header + control + command + length)
                    if(i + 6 > len) {
                        break;  // Incomplete header; wait for more data
                    }
                    uint8_t control = data[i+2];
                    uint8_t command = data[i+3];
                    // Length is 2 bytes (big-endian)
                    uint16_t payload_len = ((uint16_t)data[i+4] << 8) | data[i+5];
                    // Total frame length = header(2) + control(1) + command(1) + length(2) + payload + check(1) + tail(2)
                    int frame_total_length = payload_len + 9;
                    if(i + frame_total_length > len) {
                        break;  // Incomplete frame; wait for more data
                    }
                    // Verify tail bytes
                    if(data[i + frame_total_length - 2] != FRAME_TAIL0 ||
                       data[i + frame_total_length - 1] != FRAME_TAIL1) {
                        // Invalid tail, skip one byte and continue searching
                        i++;
                        continue;
                    }
                    
                    // Print raw frame data for debugging
                    printf("Raw frame: ");
                    for (int k = i; k < i + frame_total_length; k++) {
                        printf("%02X ", data[k]);
                    }
                    printf("\n");
                    
                    // Calculate check digit: sum of header, control, command, length, and payload bytes
                    uint32_t sum = 0;
                    for (int j = 0; j < 6 + payload_len; j++) {
                        sum += data[i + j];
                    }
                    uint8_t calc_check = sum & 0xFF;
                    // The check digit is immediately after the payload
                    uint8_t recv_check = data[i + 6 + payload_len];
                    if(calc_check != recv_check) {
                        printf("Checksum mismatch: calc 0x%02X, recv 0x%02X\n", calc_check, recv_check);
                        i += frame_total_length;
                        continue;
                    }
                    
                    // Process the frame based on control and command values
                    if(control == 0x80 && command == 0x01 && payload_len == 1) {
                        // Human presence report
                        g_presence = (data[i+6] == 0x01);
                        printf("Parsed Presence: %d\n", g_presence);
                    }
                    else if(control == 0x05 && command == 0x01 && payload_len == 1) {
                        // Working status report
                        g_working_status = data[i + 6];
                        printf("Parsed Working Status: 0x%02X\n", g_working_status);
                    }
                    else if(control == 0x05 && command == 0x81 && payload_len == 1) {
                        // Working status query acknowledgement
                        uint8_t ack = data[i + 6];
                        g_working_status = ack;  // Optionally update the global variable if it carries working status info
                        printf("Parsed Working Status Query Ack: 0x%02X\n", ack);
                    }
                    else if(control == 0x80 && command == 0x02 && payload_len == 1) {
                        // Movement information report
                        uint8_t movement_value = data[i + 6];
                        g_movement_state = movement_value; // 0: No movement, 1: Static, 2: Active
                        printf("Parsed Movement State: %d\n", g_movement_state);
                    }
                    else if(control == 0x80 && command == 0x03 && payload_len == 1) {
                        // Body movement parameter report
                        uint8_t body_movement_value = data[i + 6];
                        g_body_movement_param = body_movement_value;
                        printf("Parsed Body Movement Param: %d\n", g_body_movement_param);
                    }
                    else if(control == 0x83 && command == 0x01 && payload_len == 1) {
                        uint8_t fall_value = data[i + 6];
                        g_fall_alarm = (fall_value == 0x01);
                        printf("Parsed Fall Alarm: %d\n", g_fall_alarm);
                    }
                    else if(control == 0x83 && command == 0x05 && payload_len == 1) {
                        uint8_t still_value = data[i + 6];
                        g_stay_still_alarm = (still_value == 0x01);
                        printf("Parsed Stay-still Alarm: %d\n", g_stay_still_alarm);
                    }
                    else if(control == 0x80 && command == 0x04 && payload_len == 1) {
                        // Heartbeat report
                        uint8_t heartbeat_value = data[i + 6];
                        g_heartbeat = heartbeat_value;
                        printf("Parsed Heartbeat: %d\n", g_heartbeat);
                    }
                    else if(control == 0x01 && command == 0x01 && payload_len == 1) {
                        // Heartbeat packet: payload contains the heartbeat value.
                        g_heartbeat = data[i+6];
                        printf("Parsed Heartbeat: %d\n", g_heartbeat);
                    }
                    else if(control == 0x80 && command == 0x10 && payload_len == 4) {
                        // Trajectory point report (4 bytes: 2 for X, 2 for Y)
                        // Extract X coordinate (first 2 bytes, little-endian)
                        g_traj_x = (int16_t)((data[i + 7] << 8) | data[i + 6]);
                        // Extract Y coordinate (next 2 bytes, little-endian)
                        g_traj_y = (int16_t)((data[i + 9] << 8) | data[i + 8]);
                        printf("Parsed Trajectory: X=%d, Y=%d\n", g_traj_x, g_traj_y);
                    }
                    else if(control == 0x83 && command == 0x12 && payload_len == 4) {
                        // Trajectory point report: 2 bytes X, 2 bytes Y (big-endian)
                        g_traj_x = (int16_t)((data[i+6] << 8) | data[i+7]);
                        g_traj_y = (int16_t)((data[i+8] << 8) | data[i+9]);
                        printf("Parsed Trajectory: X=%d, Y=%d\n", g_traj_x, g_traj_y);
                    }
                    else if(control == 0x83 && command == 0x0E && payload_len == 6) {
                        // Height Proportion Report:
                        // Byte 0-1: Total height count (16-bit)
                        // Byte 2: Proportion for 0-0.5 m
                        // Byte 3: Proportion for 0.5-1 m
                        // Byte 4: Proportion for 1-1.5 m
                        // Byte 5: Proportion for 1.5-2 m
                        
                        // Add debug output to see raw bytes
                        printf("Height Proportion Raw Data: ");
                        for (int j = 0; j < payload_len; j++) {
                            printf("%02X ", data[i+6+j]);
                        }
                        printf("\n");
                        
                        uint16_t total = ((uint16_t)data[i+6] << 8) | data[i+7];
                        uint8_t prop0 = data[i+8];
                        uint8_t prop1 = data[i+9];
                        uint8_t prop2 = data[i+10];
                        uint8_t prop3 = data[i+11];
                        g_total_height_count = total;
                        g_height_prop_0_0_5 = prop0;
                        g_height_prop_0_5_1 = prop1;
                        g_height_prop_1_1_5 = prop2;
                        g_height_prop_1_5_2 = prop3;
                        
                        // Improved debug output with clear formatting and percentage calculation
                        printf("Height Proportion Report:\n");
                        printf("  Total Count: %d\n", total);
                        printf("  0-0.5m: %d (%d%%)\n", prop0, total > 0 ? (prop0 * 100 / total) : 0);
                        printf("  0.5-1m: %d (%d%%)\n", prop1, total > 0 ? (prop1 * 100 / total) : 0);
                        printf("  1-1.5m: %d (%d%%)\n", prop2, total > 0 ? (prop2 * 100 / total) : 0);
                        printf("  1.5-2m: %d (%d%%)\n", prop3, total > 0 ? (prop3 * 100 / total) : 0);
                        
                        // Verify that proportions add up to total (for debugging)
                        uint16_t sum = prop0 + prop1 + prop2 + prop3;
                        if (sum != total) {
                            printf("  WARNING: Sum of proportions (%d) doesn't match total (%d)\n", sum, total);
                        }
                    }
                    else if(control == 0x80 && command == 0x0A && payload_len == 4) {
                        // Non-presence time report (4 bytes, in seconds)
                        // Extract the 32-bit value (little-endian)
                        g_non_presence_time = ((uint32_t)data[i+9] << 24) | 
                                             ((uint32_t)data[i+8] << 16) | 
                                             ((uint32_t)data[i+7] << 8) | 
                                              data[i+6];
                        printf("Parsed Non-presence Time: %" PRIu32 " seconds\n", g_non_presence_time);
                    }
                    else if(control == 0x80 && command == 0x12 && payload_len == 4) {
                        // Non-presence time report (4 bytes, in seconds, transmitted in big-endian)
                        g_non_presence_time = ((uint32_t)data[i+6] << 24) |
                                              ((uint32_t)data[i+7] << 16) |
                                              ((uint32_t)data[i+8] << 8)  |
                                               data[i+9];
                        printf("Parsed Non-presence Time: %" PRIu32 " seconds\n", g_non_presence_time);
                    }
                    else if(control == 0x05 && command == 0x07 && payload_len == 1) {
                        // Scenario Report
                        g_scenario = data[i + 6];
                        printf("Parsed Scenario Report: %d\n", g_scenario);
                    }
                    else if(control == 0x06 && command == 0x01 && payload_len == 6) {
                        // Installation Angle Report: 3 x 16-bit values (X, Y, Z)
                        g_installation_angle_x = (int16_t)((data[i+7] << 8) | data[i+6]);
                        g_installation_angle_y = (int16_t)((data[i+9] << 8) | data[i+8]);
                        g_installation_angle_z = (int16_t)((data[i+11] << 8) | data[i+10]);
                        printf("Parsed Installation Angle: X=%d, Y=%d, Z=%d\n", 
                               g_installation_angle_x, g_installation_angle_y, g_installation_angle_z);
                    }
                    else if(control == 0x06 && command == 0x02 && payload_len == 2) {
                        // Installation Height Report
                        g_installation_height = ((uint16_t)data[i+6] << 8) | data[i+7];
                        printf("Parsed Installation Height: %d cm\n", g_installation_height);
                    }
                    else if(control == 0x83 && command == 0x02 && payload_len == 7) {
                        // Fall Detection Parameters
                        g_fall_detection_sensitivity = data[i + 6];
                        g_fall_duration = ((uint32_t)data[i+10] << 24) | 
                                         ((uint32_t)data[i+9] << 16) | 
                                         ((uint32_t)data[i+8] << 8) | 
                                          data[i+7];
                        g_fall_breaking_height = ((uint16_t)data[i+12] << 8) | data[i+11];
                        printf("Parsed Fall Detection Parameters: Sensitivity=%d, Duration=%"PRIu32" s, Breaking Height=%d cm\n",
                               g_fall_detection_sensitivity, g_fall_duration, g_fall_breaking_height);
                    }
                    else if(control == 0x83 && command == 0x0D && payload_len == 1) {
                        // Fall Detection Sensitivity Report
                        g_fall_detection_sensitivity = data[i+6];
                        printf("Parsed Fall Detection Sensitivity: %d\n", g_fall_detection_sensitivity);
                    }
                    else if(control == 0x83 && command == 0x0C && payload_len == 4) {
                        // Fall Duration Report (4 bytes, big-endian)
                        g_fall_duration = ((uint32_t)data[i+6] << 24) |
                                          ((uint32_t)data[i+7] << 16) |
                                          ((uint32_t)data[i+8] << 8)  | data[i+9];
                        printf("Parsed Fall Duration: %" PRIu32 " seconds\n", g_fall_duration);
                    }
                    else if(control == 0x83 && command == 0x11 && payload_len == 2) {
                        // Fall Breaking Height Report (2 bytes, big-endian)
                        g_fall_breaking_height = ((uint16_t)data[i+6] << 8) | data[i+7];
                        printf("Parsed Fall Breaking Height: %d cm\n", g_fall_breaking_height);
                    }
                    else if(control == 0x80 && command == 0x0D && payload_len == 2) {
                        // Sitting-still Horizontal Distance (big-endian)
                        g_sitting_still_distance = ((uint16_t)data[i+6] << 8) | data[i+7];
                        printf("Parsed Sitting-still Horizontal Distance: %d cm\n", g_sitting_still_distance);
                    }
                    else if(control == 0x80 && command == 0x0E && payload_len == 2) {
                        // Moving Horizontal Distance (big-endian)
                        g_moving_distance = ((uint16_t)data[i+6] << 8) | data[i+7];
                        printf("Parsed Moving Horizontal Distance: %d cm\n", g_moving_distance);
                    }
                    else if(control == 0x02 && command == 0xA1 && payload_len >= 1) {
                        int len_str = payload_len;
                        if(len_str > PRODUCT_STR_LEN - 1) len_str = PRODUCT_STR_LEN - 1;
                        memcpy(g_product_model, &data[i+6], len_str);
                        g_product_model[len_str] = '\0';
                        printf("Parsed Product Model: %s\n", g_product_model);
                    }
                    else if(control == 0x02 && command == 0xA2 && payload_len >= 1) {
                        int len_str = payload_len;
                        if(len_str > PRODUCT_STR_LEN - 1) len_str = PRODUCT_STR_LEN - 1;
                        memcpy(g_product_id, &data[i+6], len_str);
                        g_product_id[len_str] = '\0';
                        printf("Parsed Product ID: %s\n", g_product_id);
                    }
                    else if(control == 0x02 && command == 0xA3 && payload_len >= 1) {
                        int len_str = payload_len;
                        if(len_str > PRODUCT_STR_LEN - 1) len_str = PRODUCT_STR_LEN - 1;
                        memcpy(g_hardware_model, &data[i+6], len_str);
                        g_hardware_model[len_str] = '\0';
                        printf("Parsed Hardware Model: %s\n", g_hardware_model);
                    }
                    else if(control == 0x02 && command == 0xA4 && payload_len >= 1) {
                        int len_str = payload_len;
                        if(len_str > PRODUCT_STR_LEN - 1) len_str = PRODUCT_STR_LEN - 1;
                        memcpy(g_firmware_version, &data[i+6], len_str);
                        g_firmware_version[len_str] = '\0';
                        printf("Parsed Firmware Version: %s\n", g_firmware_version);
                    }
                    else if(control == 0x03 && command == 0xB0 && payload_len == 4) {
                        // Operating Time Report (4 bytes, big-endian)
                        g_operating_time = ((uint32_t)data[i+6] << 24) |
                                           ((uint32_t)data[i+7] << 16) |
                                           ((uint32_t)data[i+8] << 8)  | data[i+9];
                        printf("Parsed Operating Time: %" PRIu32 " seconds\n", g_operating_time);
                    }
                    else if(control == 0x83 && command == 0x0B && payload_len == 1) {
                        // Stay-still switch response
                        g_stay_still_switch = (data[i+6] == 0x01);
                        printf("Parsed Stay-still Switch: %s\n", g_stay_still_switch ? "Enabled" : "Disabled");
                    }
                    else if(control == 0x83 && command == 0x0A && payload_len == 4) {
                        // Stay-still duration response (4 bytes, big-endian)
                        g_stay_still_duration = ((uint32_t)data[i+6] << 24) |
                                               ((uint32_t)data[i+7] << 16) |
                                               ((uint32_t)data[i+8] << 8)  |
                                                data[i+9];
                        printf("Parsed Stay-still Duration: %" PRIu32 " seconds\n", g_stay_still_duration);
                    }
                    else if(control == 0x83 && command == 0x8F && payload_len == 4) {
                        // Height cumulation time query reply (4 bytes, big-endian)
                        g_height_accumulation_time= ((uint32_t)data[i+6] << 24) |
                                                   ((uint32_t)data[i+7] << 16) |
                                                   ((uint32_t)data[i+8] << 8)  |
                                                   data[i+9];
                        printf("Parsed Height Cumulation Time: %" PRIu32 " seconds\n", g_height_accumulation_time);
                    }
                    else if (control == 0x82 && command == 0x02 && payload_len == 11) {
                        // New hypothesis: This frame contains height measurement data.
                        uint16_t height_value = ((uint16_t)data[i+1]); // Take byte [1] as height in cm
                        
                        // Store the value in an appropriate global variable
                        g_total_height_count = height_value;

                        printf("✅ Parsed Height Measurement Frame: Height = %d cm\n", g_total_height_count);
                    }
                    // Inside the while(i < len) loop, after all known frame branches
                    else if(payload_len == 6 && !(control == 0x83 && command == 0x0E)) {
                        // Assume this 6-byte payload is a height proportion report
                        uint16_t total = ((uint16_t)data[i+6] << 8) | data[i+7];
                        uint8_t prop0 = data[i+8];
                        uint8_t prop1 = data[i+9];
                        uint8_t prop2 = data[i+10];
                        uint8_t prop3 = data[i+11];
                        // Update global variables
                        g_total_height_count = total;
                        g_height_prop_0_0_5 = prop0;
                        g_height_prop_0_5_1 = prop1;
                        g_height_prop_1_1_5 = prop2;
                        g_height_prop_1_5_2 = prop3;
                        
                        // Print a clear debug message with calculated percentages
                        printf("Height Proportion Report (fallback):\n");
                        printf("  Total Count: %d\n", total);
                        printf("  0-0.5m: %d (%d%%)\n", prop0, total > 0 ? (prop0 * 100 / total) : 0);
                        printf("  0.5-1m: %d (%d%%)\n", prop1, total > 0 ? (prop1 * 100 / total) : 0);
                        printf("  1-1.5m: %d (%d%%)\n", prop2, total > 0 ? (prop2 * 100 / total) : 0);
                        printf("  1.5-2m: %d (%d%%)\n", prop3, total > 0 ? (prop3 * 100 / total) : 0);
                        
                        // Advance the index past this frame and continue with next frame.
                        i += frame_total_length;
                        continue;
                    }
                    else {
                        // Print unknown frame data for debugging
                        printf("🚨🚨 Unknown Frame Data 🚨🚨: ");
                        for (int j = 0; j < payload_len; j++) {
                            printf("%02X ", data[i + 6 + j]);  // Print each byte in HEX
                        }
                        printf("\n");
                    }

                    
                    // Move index past this frame
                    i += frame_total_length;
                } else {
                    i++;
                }
            }
        }
        
        // Increase the frequency of height proportion data requests to get more updates
        uint32_t current_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if(current_tick - last_height_prop_request > 2000) {  // Changed from 5000 to 2000 ms
            // Send both types of height queries to ensure we get data
            send_query(0x83, 0x8E, 0x0F); // Height proportion over a period
            vTaskDelay(pdMS_TO_TICKS(50));
            send_query(0x83, 0x0E, 0x0F); // Direct height proportion query
            printf("Requested Height Proportion Queries\n");
            last_height_prop_request = current_tick;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task to print live data JSON payload
void live_json_print_task(void *arg)
{
    while(1) {
        print_live_json_payload();
        vTaskDelay(pdMS_TO_TICKS(30000));  // ปรับ refresh rate เป็น 30 วินาที
    }
}

// Task to print settings JSON payload
void settings_json_print_task(void *arg)
{
    while(1) {
        print_settings_json_payload();
        vTaskDelay(pdMS_TO_TICKS(15000));  // Adjust refresh rate as needed
    }
}

// Task to print product information JSON payload
void product_info_json_print_task(void *arg)
{
    while(1) {
        print_product_info_payload();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// Task to print Usage/Operating Time JSON payload periodically
void usage_json_print_task(void *arg)
{
    while(1) {
        print_usage_json_payload();
        vTaskDelay(pdMS_TO_TICKS(10000));  // Adjust update interval as needed
    }
}

// Task that monitors console input (from UART0) for key presses.
// When the user types the letter 'r' or 'R', it prints the working status report.
void key_read_task(void *arg)
{
    int ch;
    while(1) {
        // getchar() will block until a character is received on the console (UART0).
        ch = getchar();
        if(ch == 'r' || ch == 'R'){
            printf("Working Status Report (on demand): 0x%02X\n", g_working_status);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// This task periodically sends a working status query to the radar module.
void settings_read_task(void *arg)
{
    while(1) {
        send_query(0x05, 0x81, 0x0F);
        printf("Sent Working Status Query\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Helper function to build and send a query frame.
// The frame format is: HEADER (2 bytes), control (1 byte), command (1 byte),
// length (2 bytes), data (payload), check (1 byte), tail (2 bytes).
void send_query(uint8_t control, uint8_t command, uint8_t data_payload) {
    uint8_t frame[10];
    frame[0] = FRAME_HEADER0;
    frame[1] = FRAME_HEADER1;
    frame[2] = control;
    frame[3] = command;
    frame[4] = 0x00;
    frame[5] = 0x01; // 1 byte payload
    frame[6] = data_payload;
    uint32_t sum = 0;
    for (int j = 0; j < 7; j++) {
        sum += frame[j];
    }
    frame[7] = sum & 0xFF;
    frame[8] = FRAME_TAIL0;
    frame[9] = FRAME_TAIL1;
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
}

// Function to update the installation height setting on the radar device
void update_installation_height(uint16_t new_height) {
    // Frame structure: HEADER (2) + CONTROL (1) + COMMAND (1) +
    // LENGTH (2) + PAYLOAD (2) + CHECK (1) + TAIL (2) = 11 bytes total.
    uint8_t frame[11];
    frame[0] = FRAME_HEADER0;        // 0x53
    frame[1] = FRAME_HEADER1;        // 0x59
    frame[2] = 0x06;                 // Control for installation configuration
    frame[3] = 0x02;                 // Command for installation height
    frame[4] = 0x00;                 // Length high byte
    frame[5] = 0x02;                 // Length low byte (2 bytes payload)
    // Payload: height in centimeters, big-endian
    frame[6] = (new_height >> 8) & 0xFF;
    frame[7] = new_height & 0xFF;
    
    // Calculate check digit: sum of all bytes from header to payload.
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += frame[i];
    }
    frame[8] = sum & 0xFF;
    
    // Append fixed tail
    frame[9]  = FRAME_TAIL0;         // 0x54
    frame[10] = FRAME_TAIL1;         // 0x43

    // Write frame to UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent installation height update: %d cm\n", new_height);

    // Optionally update the global variable if you want to reflect the change immediately.
    g_installation_height = new_height;
    
    // // Publish updated settings
    // mqtt_publish_settings();
}

void update_height_accumulation_time(uint32_t seconds) {
    uint8_t frame[13];
    frame[0] = FRAME_HEADER0;   // 0x53
    frame[1] = FRAME_HEADER1;   // 0x59
    frame[2] = 0x83;            // Control byte
    frame[3] = 0x8F;            // <-- Corrected command for height accumulation time
    frame[4] = 0x00;            // Payload length high byte
    frame[5] = 0x04;            // Payload length (4 bytes)

    // Payload: seconds (big-endian)
    frame[6] = (seconds >> 24) & 0xFF;
    frame[7] = (seconds >> 16) & 0xFF;
    frame[8] = (seconds >> 8) & 0xFF;
    frame[9] = seconds & 0xFF;

    // Checksum calculation
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) sum += frame[i];
    frame[10] = sum & 0xFF;

    // Append tail
    frame[11] = FRAME_TAIL0;    // 0x54
    frame[12] = FRAME_TAIL1;    // 0x43

    // Send frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Height Accumulation Time updated to: %" PRIu32 " seconds\n", seconds);

    g_height_accumulation_time = seconds;
}


void heartbeat_task(void *arg)
{
    while (1) {
        // Send heartbeat query frame
        uint8_t frame[10];
        frame[0] = FRAME_HEADER0;   // 0x53
        frame[1] = FRAME_HEADER1;   // 0x59
        frame[2] = 0x01;            // Control byte for heartbeat
        frame[3] = 0x01;            // Command for heartbeat query
        frame[4] = 0x00;            // Length high byte
        frame[5] = 0x01;            // Length low byte (1 byte payload)
        frame[6] = 0x0F;            // Payload (query value)

        // Calculate checksum
        uint32_t sum = 0;
        for (int i = 0; i < 7; i++) {
            sum += frame[i];
        }
        frame[7] = sum & 0xFF;

        // Append tail
        frame[8] = FRAME_TAIL0;     // 0x54
        frame[9] = FRAME_TAIL1;     // 0x43

        // Send frame via UART
        uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
        printf("Sent heartbeat query\n");

        // Wait for 10 seconds before next heartbeat
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


void update_non_presence_time(uint32_t seconds) {
    // Frame structure (13 bytes total):
    // Header (2) + Control (1) + Command (1) + Length (2) + Payload (4) + Check (1) + Tail (2)
    uint8_t frame[13];
    frame[0] = FRAME_HEADER0;   // 0x53
    frame[1] = FRAME_HEADER1;   // 0x59
    frame[2] = 0x83;            // Control byte
    frame[3] = 0x0B;            // Command for non-presence time
    frame[4] = 0x00;            // Length high byte
    frame[5] = 0x04;            // Length low byte (4 bytes payload)
    
    // Convert seconds to bytes (little-endian)
    frame[6] = seconds & 0xFF;
    frame[7] = (seconds >> 8) & 0xFF;
    frame[8] = (seconds >> 16) & 0xFF;
    frame[9] = (seconds >> 24) & 0xFF;

    // Calculate checksum
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += frame[i];
    }
    frame[10] = sum & 0xFF;

    // Append tail
    frame[11] = FRAME_TAIL0;    // 0x54
    frame[12] = FRAME_TAIL1;    // 0x43

    // Send frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Non-presence Time updated to: %" PRIu32 " seconds\n", seconds);

    // Update global variable
    g_non_presence_time = seconds;
}

// Function to update the fall detection sensitivity setting on the radar device
void update_fall_detection_sensitivity(uint8_t new_sensitivity) {
    // Enforce valid range [0, 3]
    if(new_sensitivity > 3) {
        printf("Sensitivity value %d out of range. Clamping to 3.\n", new_sensitivity);
        new_sensitivity = 3;
    }
    // Frame structure:
    // Header (2 bytes) + Control (1 byte) + Command (1 byte) +
    // Length (2 bytes) + Payload (1 byte) + Check (1 byte) + Tail (2 bytes) = 10 bytes total.
    uint8_t frame[10];
    frame[0] = FRAME_HEADER0;          // e.g., 0x53
    frame[1] = FRAME_HEADER1;          // e.g., 0x59
    frame[2] = 0x83;                   // Control for fall detection
    frame[3] = 0x0D;                   // Command for updating fall detection sensitivity
    frame[4] = 0x00;                   // Payload length high byte (1 byte payload)
    frame[5] = 0x01;                   // Payload length low byte
    frame[6] = new_sensitivity;        // New sensitivity value (0 to 3)
    
    // Calculate check digit (sum of header, control, command, length, and payload bytes)
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += frame[i];
    }
    frame[7] = sum & 0xFF;
    frame[8] = FRAME_TAIL0;            // e.g., 0x54
    frame[9] = FRAME_TAIL1;            // e.g., 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Fall Detection Sensitivity update: %d\n", new_sensitivity);

    // Update the global variable to reflect the new setting
    g_fall_detection_sensitivity = new_sensitivity;
    
    // Publish updated settings
    // mqtt_publish_settings();
}

// Function to update the fall duration setting on the radar device
void update_fall_duration(uint32_t new_duration) {
    // Enforce valid range [5, 180] seconds
    if(new_duration < 5) {
        printf("Duration value %" PRIu32 " s is below minimum. Setting to 5 s.\n", new_duration);
        new_duration = 5;
    }
    if(new_duration > 180) {
        printf("Duration value %" PRIu32 " s exceeds maximum. Setting to 180 s.\n", new_duration);
        new_duration = 180;
    }
    
    // Frame structure:
    // Header (2 bytes) + Control (1 byte) + Command (1 byte) + Length (2 bytes)
    // + Payload (4 bytes) + Check (1 byte) + Tail (2 bytes) = 13 bytes total.
    uint8_t frame[13];
    frame[0] = FRAME_HEADER0;          // 0x53
    frame[1] = FRAME_HEADER1;          // 0x59
    frame[2] = 0x83;                   // Control for fall detection parameters
    frame[3] = 0x0C;                   // Command for updating fall duration
    frame[4] = 0x00;                   // Length high byte (payload is 4 bytes)
    frame[5] = 0x04;                   // Length low byte
    
    // Payload: new_duration (4 bytes, big-endian)
    frame[6] = (new_duration >> 24) & 0xFF;
    frame[7] = (new_duration >> 16) & 0xFF;
    frame[8] = (new_duration >> 8)  & 0xFF;
    frame[9] = new_duration & 0xFF;
    
    // Calculate check digit: sum bytes from header through payload
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += frame[i];
    }
    frame[10] = sum & 0xFF;
    
    // Append tail bytes
    frame[11] = FRAME_TAIL0;           // 0x54
    frame[12] = FRAME_TAIL1;           // 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Fall Duration update: %" PRIu32 " seconds\n", new_duration);

    // Optionally, update the global variable to reflect the new value.
    g_fall_duration = new_duration;
    
    // Publish updated settings
    // mqtt_publish_settings();
}

// Function to update the fall breaking height setting on the radar device
void update_fall_breaking_height(uint16_t new_height) {
    // Enforce valid range: 0 to 150 cm
    if(new_height > 150) {
        printf("Height value %d cm exceeds maximum. Clamping to 150 cm.\n", new_height);
        new_height = 150;
    }
    
    // Frame structure:
    // HEADER (2 bytes) + CONTROL (1 byte) + COMMAND (1 byte) + LENGTH (2 bytes)
    // + PAYLOAD (2 bytes) + CHECK (1 byte) + TAIL (2 bytes) = 11 bytes total.
    uint8_t frame[11];
    frame[0] = FRAME_HEADER0;          // e.g., 0x53
    frame[1] = FRAME_HEADER1;          // e.g., 0x59
    frame[2] = 0x83;                   // Control for fall detection parameters
    frame[3] = 0x11;                   // Command for fall breaking height update
    frame[4] = 0x00;                   // Length high byte (payload length is 2)
    frame[5] = 0x02;                   // Length low byte
    
    // Payload: new_height (2 bytes, big-endian)
    frame[6] = (new_height >> 8) & 0xFF;
    frame[7] = new_height & 0xFF;
    
    // Calculate check digit: sum of header, control, command, length, and payload bytes (bytes 0 to 7)
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += frame[i];
    }
    frame[8] = sum & 0xFF;
    
    // Append tail bytes
    frame[9]  = FRAME_TAIL0;           // e.g., 0x54
    frame[10] = FRAME_TAIL1;           // e.g., 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Fall Breaking Height update: %d cm\n", new_height);

    // Optionally update the global variable
    g_fall_breaking_height = new_height;
    
    // Publish updated settings
    // mqtt_publish_settings();
}

// Function to update the sitting-still horizontal distance setting on the radar device
void update_sitting_still_distance(uint16_t new_distance) {
    // Enforce the valid range: 0 to 300 cm
    if(new_distance > 300) {
        printf("Sitting-Still Distance value %d cm exceeds maximum. Clamping to 300 cm.\n", new_distance);
        new_distance = 300;
    }
    
    // Frame structure:
    // HEADER (2 bytes) + CONTROL (1 byte) + COMMAND (1 byte) + LENGTH (2 bytes)
    // + PAYLOAD (2 bytes) + CHECK (1 byte) + TAIL (2 bytes) = 11 bytes total.
    uint8_t frame[11];
    frame[0] = FRAME_HEADER0;          // e.g., 0x53
    frame[1] = FRAME_HEADER1;          // e.g., 0x59
    frame[2] = 0x80;                   // Control for sensor data settings
    frame[3] = 0x0D;                   // Command for setting sitting-still horizontal distance
    frame[4] = 0x00;                   // Length high byte (2 bytes payload)
    frame[5] = 0x02;                   // Length low byte
    // Payload: new_distance in cm, represented in big-endian format.
    frame[6] = (new_distance >> 8) & 0xFF;
    frame[7] = new_distance & 0xFF;
    
    // Calculate check digit: sum bytes 0 to 7, then take the lower 8 bits.
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += frame[i];
    }
    frame[8] = sum & 0xFF;
    
    // Append tail bytes
    frame[9]  = FRAME_TAIL0;           // e.g., 0x54
    frame[10] = FRAME_TAIL1;           // e.g., 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Sitting-Still Distance update: %d cm\n", new_distance);
    
    // Optionally update the global variable to reflect the new setting
    g_sitting_still_distance = new_distance;
    
    // Publish updated settings
    // mqtt_publish_settings();
}

// Function to update the moving horizontal distance setting on the radar device
void update_moving_distance(uint16_t new_distance) {
    // Enforce valid range: 0 to 300 cm
    if(new_distance > 300) {
        printf("Moving Horizontal Distance value %d cm exceeds maximum. Clamping to 300 cm.\n", new_distance);
        new_distance = 300;
    }
    
    // Frame structure:
    // HEADER (2 bytes) + CONTROL (1 byte) + COMMAND (1 byte) +
    // LENGTH (2 bytes) + PAYLOAD (2 bytes) + CHECK (1 byte) + TAIL (2 bytes)
    // Total: 11 bytes.
    uint8_t frame[11];
    frame[0] = FRAME_HEADER0;  // e.g., 0x53
    frame[1] = FRAME_HEADER1;  // e.g., 0x59
    frame[2] = 0x80;           // Control for sensor data settings
    frame[3] = 0x0E;           // Command for setting moving horizontal distance
    frame[4] = 0x00;           // Payload length high byte (2 bytes)
    frame[5] = 0x02;           // Payload length low byte
    
    // Payload: new_distance in big-endian format
    frame[6] = (new_distance >> 8) & 0xFF;
    frame[7] = new_distance & 0xFF;
    
    // Calculate check digit: sum of bytes 0 to 7
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += frame[i];
    }
    frame[8] = sum & 0xFF;
    
    // Append tail bytes
    frame[9]  = FRAME_TAIL0;  // e.g., 0x54
    frame[10] = FRAME_TAIL1;  // e.g., 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Moving Horizontal Distance update: %d cm\n", new_distance);

    // Optionally update the global variable
    g_moving_distance = new_distance;
    
    // Publish updated settings
    // mqtt_publish_settings();
}

// Function to update the stay-still alarm switch on the radar device
void update_stay_still_switch(bool enable) {
    // Frame structure (10 bytes total):
    // Header (2) + Control (1) + Command (1) + Length (2) + Payload (1) + Check (1) + Tail (2)
    uint8_t frame[10];
    frame[0] = FRAME_HEADER0;   // 0x53
    frame[1] = FRAME_HEADER1;   // 0x59
    frame[2] = 0x83;            // Control for fall detection / stay-still
    frame[3] = 0x0B;            // Command for stay-still switch
    frame[4] = 0x00;            // Length high byte
    frame[5] = 0x01;            // Length low byte (1 byte payload)
    frame[6] = enable ? 0x01 : 0x00; // Payload: 0x01 to enable, 0x00 to disable

    // Calculate checksum: sum of bytes [0..6], take lower 8 bits
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += frame[i];
    }
    frame[7] = sum & 0xFF;

    // Append tail
    frame[8] = FRAME_TAIL0;    // 0x54
    frame[9] = FRAME_TAIL1;    // 0x43

    // Send frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Stay-Still Alarm %s\n", enable ? "ENABLED" : "DISABLED");
    
    // Update global variable
    g_stay_still_switch = enable;
}

// Function to update the stay-still duration setting on the radar device
void update_stay_still_duration(uint32_t new_duration) {
    // Enforce valid range [60, 3600] seconds
    if(new_duration < 60) {
        printf("Duration value %" PRIu32 " s is below minimum. Setting to 60 s.\n", new_duration);
        new_duration = 60;
    }
    if(new_duration > 3600) {
        printf("Duration value %" PRIu32 " s exceeds maximum. Setting to 3600 s.\n", new_duration);
        new_duration = 3600;
    }

    uint8_t frame[13] = {
        FRAME_HEADER0, FRAME_HEADER1,
        0x83, 0x0A, 0x00, 0x04,
        (new_duration >> 24) & 0xFF,
        (new_duration >> 16) & 0xFF,
        (new_duration >> 8) & 0xFF,
        new_duration & 0xFF,
        0x00, // placeholder for checksum
        FRAME_TAIL0,
        FRAME_TAIL1
    };

    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) sum += frame[i];
    frame[10] = sum & 0xFF;

    // Update global variable before sending UART command
    g_stay_still_duration = new_duration;

    // Send UART command
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Stay-still Duration update: %" PRIu32 " seconds\n", new_duration);

    // Add a small delay to ensure UART transmission is complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check if MQTT client is initialized before publishing
    if (mqtt_client != NULL) {
        mqtt_publish_settings();
    }
}

// This task periodically sends product information queries.
void product_info_query_task(void *arg)
{
    while(1) {
        // Query Product Model (control 0x02, command 0xA1)
        send_query(0x02, 0xA1, 0x0F);
        printf("Sent Product Model Query\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Query Product ID (control 0x02, command 0xA2)
        send_query(0x02, 0xA2, 0x0F);
        printf("Sent Product ID Query\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Query Hardware Model (control 0x02, command 0xA3)
        send_query(0x02, 0xA3, 0x0F);
        printf("Sent Hardware Model Query\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Query Firmware Version (control 0x02, command 0xA4)
        send_query(0x02, 0xA4, 0x0F);
        printf("Sent Firmware Version Query\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ---------------------- New: Height Proportion Query Task ----------------------
// This task periodically sends a query for height proportion data.
// Expected response: control 0x83, command 0x0E, payload length 6.
void height_proportion_query_task(void *arg)
{
    while(1) {
        send_query(0x83, 0x0E, 0x0F); // Adjust data payload as per device spec
        printf("Sent Height Proportion Query\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ---------------------- New: Height Proportion Period Query Task ----------------------
// This task periodically sends a query for height proportion over a period of time.
// Expected query: control 0x83, command 0x8E, with 1-byte payload (example: 0x0F).
void height_proportion_period_query_task(void *arg)
{
    while(1) {
        // Send query frame with control 0x83 and command 0x8E.
        send_query(0x83, 0x8E, 0x0F); // Adjust the payload (0x0F) as per your device specification.
        printf("Sent Height Proportion Period Query\n");
        vTaskDelay(pdMS_TO_TICKS(5000));  // Adjust the query interval as needed
    }
}

// ---------------------- Wi-Fi Configuration ----------------------
#define WIFI_SSID      "Home_2.4G"
#define WIFI_PASS      "11112222"
#define MAXIMUM_RETRY  5

static const char *TAG = "wifi_connect";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Wi-Fi event handler function
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to AP...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Failed to connect to the AP");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    // Remove the NVS initialization code from here
    
    s_wifi_event_group = xEventGroupCreate();

    // Initialize the TCP/IP stack and the event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(sta_netif, "Falldetector7");
    

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Configure Wi-Fi connection settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// New: Function to update installation angles.
void update_installation_angles(int16_t angle_x, int16_t angle_y, int16_t angle_z) {
    // Frame structure:
    // HEADER (2 bytes) + CONTROL (1) + COMMAND (1) + LENGTH (2) + PAYLOAD (6 bytes) + CHECK (1) + TAIL (2) = 15 bytes.
    uint8_t frame[15];
    frame[0] = FRAME_HEADER0;
    frame[1] = FRAME_HEADER1;
    frame[2] = 0x06;   // Control for installation configuration.
    frame[3] = 0x01;   // Command for installation angle update.
    frame[4] = 0x00;
    frame[5] = 0x06;   // 6-byte payload.
    // Payload: 16-bit signed values (big-endian)
    frame[6]  = (angle_x >> 8) & 0xFF;
    frame[7]  = angle_x & 0xFF;
    frame[8]  = (angle_y >> 8) & 0xFF;
    frame[9]  = angle_y & 0xFF;
    frame[10] = (angle_z >> 8) & 0xFF;
    frame[11] = angle_z & 0xFF;
    // Calculate checksum over bytes 0 to 11.
    uint32_t sum = 0;
    for (int i = 0; i < 12; i++) {
        sum += frame[i];
    }
    frame[12] = sum & 0xFF;
    frame[13] = FRAME_TAIL0;
    frame[14] = FRAME_TAIL1;
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Installation Angles update: X=%d, Y=%d, Z=%d\n", angle_x, angle_y, angle_z);
    
    // Optionally update the global variables.
    g_installation_angle_x = angle_x;
    g_installation_angle_y = angle_y;
    g_installation_angle_z = angle_z;
    
    // Publish updated settings
    // mqtt_publish_settings();
}

// Function to publish product information to MQTT
void mqtt_publish_product_info(void)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        printf("Error creating product info JSON object\n");
        return;
    }
    
    // Add device identification
    cJSON_AddStringToObject(json, "device_id", g_device_id);
    cJSON_AddStringToObject(json, "device_type", DEVICE_TYPE);
    
    // Add product information
    cJSON_AddStringToObject(json, "product_model", g_product_model);
    cJSON_AddStringToObject(json, "product_id", g_product_id);
    cJSON_AddStringToObject(json, "hardware_model", g_hardware_model);
    cJSON_AddStringToObject(json, "firmware_version", g_firmware_version);
    
    // Add operating time
    cJSON_AddNumberToObject(json, "operating_time", g_operating_time);
    
    char *json_str = cJSON_Print(json);
    if (json_str) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_INFO, json_str, 0, 1, 0);
        if (msg_id != -1) {
            printf("Published product info to %s\n", MQTT_TOPIC_INFO);
        } else {
            printf("Failed to publish product info\n");
        }
        free(json_str);
    }
    cJSON_Delete(json);
}

// Function to initialize default settings on the radar device
void init_default_settings(void) {
    printf("Initializing default settings...\n");
    
    // Enable human presence detection first
    enable_human_presence_detection(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default installation angles (0, 0, 0)
    update_installation_angles(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200)); // Small delay between commands
    
    // Set default installation height (200 cm)
    update_installation_height(200);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default fall detection sensitivity (3)
    update_fall_detection_sensitivity(3);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default fall duration (5 seconds)
    update_fall_duration(5);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default fall breaking height (20 cm)
    update_fall_breaking_height(20);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default sitting still distance (300 cm)
    update_sitting_still_distance(300);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default moving distance (30 cm)
    update_moving_distance(30);
    
    // Set default stay-still duration (300 seconds = 5 minutes)
    update_stay_still_duration(60);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Enable fall detection by default
    update_fall_detection_switch(true);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Set default non-presence time (300 seconds = 5 minutes)
    update_non_presence_time(5);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Set default height accumulation time (60 seconds = 1 minute)
    update_height_accumulation_time(60);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    printf("Default settings initialized\n");
}

// Function to save current settings to NVS
void save_settings_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("radar_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        return;
    }

    // Individual error checking for each setting
    err = nvs_set_i16(nvs_handle, "angle_x", g_installation_angle_x);
    if (err != ESP_OK) printf("Error saving angle_x: %s\n", esp_err_to_name(err));
    
    err = nvs_set_i16(nvs_handle, "angle_y", g_installation_angle_y);
    if (err != ESP_OK) printf("Error saving angle_y: %s\n", esp_err_to_name(err));
    
    err = nvs_set_i16(nvs_handle, "angle_z", g_installation_angle_z);
    if (err != ESP_OK) printf("Error saving angle_z: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u16(nvs_handle, "inst_height", g_installation_height);
    if (err != ESP_OK) printf("Error saving inst_height: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u8(nvs_handle, "fall_sens", g_fall_detection_sensitivity);
    if (err != ESP_OK) printf("Error saving fall_sens: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u32(nvs_handle, "fall_dur", g_fall_duration);
    if (err != ESP_OK) printf("Error saving fall_dur: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u16(nvs_handle, "fall_height", g_fall_breaking_height);
    if (err != ESP_OK) printf("Error saving fall_height: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u16(nvs_handle, "still_dist", g_sitting_still_distance);
    if (err != ESP_OK) printf("Error saving still_dist: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u16(nvs_handle, "move_dist", g_moving_distance);
    if (err != ESP_OK) printf("Error saving move_dist: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u8(nvs_handle, "still_switch", g_stay_still_switch ? 1 : 0);
    if (err != ESP_OK) printf("Error saving still_switch: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u32(nvs_handle, "still_dur", g_stay_still_duration);
    if (err != ESP_OK) printf("Error saving still_dur: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u8(nvs_handle, "fall_switch", g_fall_detection_switch ? 1 : 0);
    if (err != ESP_OK) printf("Error saving fall_switch: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u32(nvs_handle, "non_p_time", g_non_presence_time);
    if (err != ESP_OK) printf("Error saving non_presence_time: %s\n", esp_err_to_name(err));
    
    err = nvs_set_u32(nvs_handle, "h_acc_t", g_height_accumulation_time);
    if (err != ESP_OK) printf("Error saving height_accumulation_time: %s\n", esp_err_to_name(err));

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        printf("Error committing NVS changes: %s\n", esp_err_to_name(err));
    } else {
        printf("Settings saved to NVS successfully\n");
    }

    nvs_close(nvs_handle);
}

// Improved function to load settings from NVS
void load_settings_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("radar_config", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        printf("Using default settings instead\n");
        init_default_settings();
        return;
    }

    bool at_least_one_failed = false;
    
    // Temp variables with defaults pre-set
    int16_t angle_x = 0, angle_y = 0, angle_z = 0;
    uint16_t inst_height = 200, fall_height = 20, still_dist = 300, move_dist = 30;
    uint8_t fall_sens = 3, still_switch = 0, fall_switch = 1;
    uint32_t fall_dur = 5, still_dur = 60, non_p_time = 5, h_acc_t = 60;
    
    // Load all values with individual error checking
    err = nvs_get_i16(nvs_handle, "angle_x", &angle_x);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading angle_x: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_i16(nvs_handle, "angle_y", &angle_y);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading angle_y: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    // ... repeat for all other parameters with similar error checking ...
    
    err = nvs_get_i16(nvs_handle, "angle_z", &angle_z);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading angle_z: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u16(nvs_handle, "inst_height", &inst_height);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading inst_height: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u8(nvs_handle, "fall_sens", &fall_sens);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading fall_sens: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u32(nvs_handle, "fall_dur", &fall_dur);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading fall_dur: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u16(nvs_handle, "fall_height", &fall_height);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading fall_height: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u16(nvs_handle, "still_dist", &still_dist);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading still_dist: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u16(nvs_handle, "move_dist", &move_dist);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading move_dist: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u8(nvs_handle, "still_switch", &still_switch);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading still_switch: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u32(nvs_handle, "still_dur", &still_dur);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading still_dur: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u8(nvs_handle, "fall_switch", &fall_switch);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading fall_switch: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u32(nvs_handle, "non_p_time", &non_p_time);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading non_p_time: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }
    
    err = nvs_get_u32(nvs_handle, "h_acc_t", &h_acc_t);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("Error reading h_acc_t: %s\n", esp_err_to_name(err));
        at_least_one_failed = true;
    }

    nvs_close(nvs_handle);
    
    if (at_least_one_failed) {
        printf("Some settings could not be loaded from NVS, using available values\n");
    } else {
        printf("Settings loaded successfully from NVS\n");
    }

    // Apply loaded settings to the radar
    printf("Applying settings...\n");
    
    // Store values in global variables
    g_non_presence_time = non_p_time;
    g_height_accumulation_time = h_acc_t;
    
    // Apply settings to device with small delays between commands
    enable_human_presence_detection(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_installation_angles(angle_x, angle_y, angle_z);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // ... apply other settings similarly ...
    update_installation_height(inst_height);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_fall_detection_sensitivity(fall_sens);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_fall_duration(fall_dur);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_fall_breaking_height(fall_height);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_sitting_still_distance(still_dist);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_moving_distance(move_dist);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_stay_still_switch(still_switch);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_stay_still_duration(still_dur);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_fall_detection_switch(fall_switch);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_height_accumulation_time(h_acc_t);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    update_non_presence_time(non_p_time);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    printf("Settings applied successfully\n");
}

// Add this function implementation before app_main()
void reboot_device(void) {
    // Frame structure (10 bytes total):
    // Header (2) + Control (1) + Command (1) + Length (2) + Payload (1) + Check (1) + Tail (2)
    // uint8_t frame[10];
    // frame[0] = FRAME_HEADER0;   // 0x53
    // frame[1] = FRAME_HEADER1;   // 0x59
    // frame[2] = 0x02;           // Control for system commands
    // frame[3] = 0xA0;           // Command for reboot
    // frame[4] = 0x00;           // Length high byte
    // frame[5] = 0x01;           // Length low byte (1 byte payload)
    // frame[6] = 0x0F;           // Payload (typical query value)

    // // Calculate checksum
    // uint32_t sum = 0;
    // for (int i = 0; i < 7; i++) {
    //     sum += frame[i];
    // }
    // frame[7] = sum & 0xFF;

    // // Append tail
    // frame[8] = FRAME_TAIL0;    // 0x54
    // frame[9] = FRAME_TAIL1;    // 0x43

    // // Send frame via UART
    // uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sending reboot command to device\n");
    esp_restart();
    printf("Sent reboot command to device\n");
}

// Add this function implementation
void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("NVS needs to be erased - this will delete saved settings\n");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            printf("Error erasing NVS flash: %s\n", esp_err_to_name(ret));
            return;
        }
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        printf("Failed to initialize NVS: %s\n", esp_err_to_name(ret));
        return;
    }
    
    printf("NVS initialized successfully\n");
}

// Add this function after init_default_settings() to specifically initialize height-related settings
void init_height_detection(void) {
    printf("Initializing height detection parameters...\n");
    
    // Send a smaller value first to get faster results
    update_height_accumulation_time(30);  // 30 seconds instead of 60
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Force a height proportion request immediately
    send_query(0x83, 0x0E, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(100));
    send_query(0x83, 0x8E, 0x0F);
    
    printf("Height detection parameters initialized\n");
}

// Function to save the device ID to NVS
void save_device_id_to_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_set_str(my_handle, "device_id", g_device_id);
        if (err != ESP_OK) {
            printf("Error (%s) writing device_id to NVS!\n", esp_err_to_name(err));
        }
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            printf("Error (%s) committing updates to NVS!\n", esp_err_to_name(err));
        } else {
            printf("Device ID saved to NVS: %s\n", g_device_id);
        }
        nvs_close(my_handle);
    }
}

// Function to load the device ID from NVS
void load_device_id_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        size_t required_size = sizeof(g_device_id);
        err = nvs_get_str(my_handle, "device_id", g_device_id, &required_size);
        switch (err) {
            case ESP_OK:
                printf("Successfully loaded device_id from NVS: %s\n", g_device_id);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The device_id is not initialized yet!\n");
                printf("Saving default device_id to NVS.\n");
                // The default g_device_id is already set, so we just save it.
                save_device_id_to_nvs();
                break;
            default:
                printf("Error (%s) reading device_id from NVS!\n", esp_err_to_name(err));
        }
        nvs_close(my_handle);
    }
}

// Main entry point
void app_main(void)
{
    // Initialize NVS first
    init_nvs();
    printf("Release 1.0.3\n");
    // Load device ID from NVS, or use default and save it
    load_device_id_from_nvs();

    // Construct dynamic MQTT topics
    snprintf(mqtt_topic_settings_update, sizeof(mqtt_topic_settings_update), "%s/settings_update", g_device_id);
    snprintf(mqtt_topic_info_device_id, sizeof(mqtt_topic_info_device_id), "%s/info", g_device_id);
    snprintf(mqtt_topic_settings_state_device_id, sizeof(mqtt_topic_settings_state_device_id), "%s/settings_state", g_device_id);
    snprintf(mqtt_topic_ota_update, sizeof(mqtt_topic_ota_update), "%s/ota_update", g_device_id);
    snprintf(mqtt_topic_live, sizeof(mqtt_topic_live), "R60AFD1/live", g_device_id);

    // Initialize UART for communication with the radar module
    init_uart();
    
    // Load settings from NVS (or use defaults if not found)
    load_settings_from_nvs();
    
    // Start WiFiManager (AP + Web Portal)
    wifiManager_init();
    
    // รอจนกว่าจะเชื่อมต่อ Wi-Fi สำเร็จ
    xEventGroupWaitBits(wm_wifi_event_group, WM_EVENTG_WIFI_CONNECTED, pdFALSE, pdFALSE, portMAX_DELAY);
    
    // เมื่อเชื่อมต่อ Wi-Fi ได้แล้ว ค่อยเริ่ม MQTT และ sensor task
    mqtt_init();
    mqtt_publish_product_info();
    mqtt_publish_settings();
    
    // Create tasks: one for reading/parsing UART data, one for printing live data JSON, one for settings JSON, one for product info JSON, and one for settings read
    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 10, NULL);
    xTaskCreate(live_json_print_task, "live_json_print_task", 4096, NULL, 10, NULL);
    xTaskCreate(settings_json_print_task, "settings_json_print_task", 4096, NULL, 10, NULL);
    xTaskCreate(product_info_json_print_task, "product_info_json_print_task", 4096, NULL, 10, NULL);
    xTaskCreate(usage_json_print_task, "usage_json_print_task", 4096, NULL, 10, NULL);
    xTaskCreate(settings_read_task, "settings_read_task", 2048, NULL, 10, NULL);
    xTaskCreate(product_info_query_task, "product_info_query_task", 2048, NULL, 10, NULL);
    xTaskCreate(height_proportion_query_task, "height_proportion_query_task", 2048, NULL, 10, NULL);
    xTaskCreate(height_proportion_period_query_task, "height_proportion_period_query_task", 2048, NULL, 10, NULL);
    
    // Add the new MQTT publish task
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 10, NULL);
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 10, NULL);  // Add heartbeat task
    
    // Initialize height detection separately after other settings
    init_height_detection();
    
    // Create and start height proportion query task
    xTaskCreate(height_proportion_query_task, "height_prop_query", 2048, NULL, 5, NULL);
    
    // If your app_main doesn't already have this task creation, add it
    xTaskCreate(height_proportion_period_query_task, "height_period_query", 2048, NULL, 5, NULL);

    // ตัวอย่างการเรียก OTA (แนะนำให้เรียกเมื่อได้รับคำสั่งจาก MQTT หรือปุ่ม ไม่ควรเรียกทันทีหลังบูต)
    // ota_update_start("http://192.168.1.58:8000/esp32-R60AFD1.bin");
    // รอ Wi-Fi เชื่อมต่อสำเร็จ (ถ้ามี)
    // ota_update_start("http://192.168.1.58:8000/esp32-R60AFD1.bin");

    // ใน app_main() ให้เพิ่มการสร้าง task นี้
    xTaskCreate(wifi_reset_button_task, "wifi_reset_button_task", 2048, NULL, 5, NULL);
}

// Add this function implementation with the other command functions
void enable_human_presence_detection(bool enable) {
    // Frame structure (10 bytes total):
    // Header (2) + Control (1) + Command (1) + Length (2) + Payload (1) + Check (1) + Tail (2)
    uint8_t frame[10];
    frame[0] = FRAME_HEADER0;   // 0x53
    frame[1] = FRAME_HEADER1;   // 0x59
    frame[2] = 0x80;            // Control for sensor data settings
    frame[3] = 0x00;            // Command for human presence detection
    frame[4] = 0x00;            // Length high byte
    frame[5] = 0x01;            // Length low byte (1 byte payload)
    frame[6] = enable ? 0x01 : 0x00; // Payload: 0x01 to enable, 0x00 to disable

    // Calculate checksum
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += frame[i];
    }
    frame[7] = sum & 0xFF;

    // Append tail
    frame[8] = FRAME_TAIL0;    // 0x54
    frame[9] = FRAME_TAIL1;    // 0x43

    // Send frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Human Presence Detection %s\n", enable ? "ENABLED" : "DISABLED");
}

// เพิ่ม task สำหรับตรวจสอบปุ่ม reset Wi-Fi
void wifi_reset_button_task(void *arg) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    int hold_time = 0;
    while (1) {
        int level = gpio_get_level(RESET_BUTTON_GPIO);
        printf("DEBUG: GPIO%d level = %d\n", RESET_BUTTON_GPIO, level);
        if (level == 0) { // ปุ่มถูกกด (active low)
            hold_time += 100;
            if (hold_time >= RESET_HOLD_TIME_MS) {
                printf("[RESET] Button held, resetting Wi-Fi config...\n");
                nvs_flash_erase(); // ลบ Wi-Fi config ทั้งหมด
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart(); // รีสตาร์ทบอร์ด
            }
        } else {
            hold_time = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

