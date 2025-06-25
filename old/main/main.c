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

// Add these function prototypes before mqtt_event_handler
void update_installation_angles(int16_t angle_x, int16_t angle_y, int16_t angle_z);
void update_installation_height(uint16_t new_height);
void update_fall_detection_sensitivity(uint8_t new_sensitivity);
void update_fall_duration(uint32_t new_duration);
void update_fall_breaking_height(uint16_t new_height);
void update_sitting_still_distance(uint16_t new_distance);
void update_moving_distance(uint16_t new_distance);

// Forward declaration of the send_query function
void send_query(uint8_t control, uint8_t command, uint8_t data_payload);

// ---------------------- UART and Frame Definitions ----------------------
#define UART_PORT_NUM      UART_NUM_1
#define BUF_SIZE           1024

#define FRAME_HEADER0      0x53
#define FRAME_HEADER1      0x59
#define FRAME_TAIL0        0x54
#define FRAME_TAIL1        0x43

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
#define MQTT_BROKER_URI      "mqtt://mqtt.datatamer.ai"
#define MQTT_USERNAME        "raspi5nr"
#define MQTT_PASSWORD        "12345"
#define MQTT_TOPIC_LIVE      "R60AFD1/live"
#define MQTT_TOPIC_SETTINGS  "R60AFD1/settings"
#define MQTT_TOPIC_INFO      "R60AFD1/info"

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
            int msg_id = esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_SETTINGS, 0);
            ESP_LOGI(MQTT_TAG, "Subscribed to %s, msg_id=%d", MQTT_TOPIC_SETTINGS, msg_id);
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
            if (event->topic_len == strlen(MQTT_TOPIC_SETTINGS) &&
                strncmp(event->topic, MQTT_TOPIC_SETTINGS, event->topic_len) == 0) {
                
                printf("Received settings update: %.*s\n", event->data_len, event->data);
                cJSON *json = cJSON_Parse(event->data);
                if(json == NULL) {
                    ESP_LOGE(MQTT_TAG, "Error parsing settings JSON");
                } else {
                    cJSON *item = NULL;
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
                    item = cJSON_GetObjectItem(json, "installation_height");
                    if(item && cJSON_IsNumber(item)) {
                        update_installation_height((uint16_t)item->valueint);
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
                    cJSON_Delete(json);
                }
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
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
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
    char *json_str = get_live_json_payload_str();
    if (json_str) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_LIVE, json_str, 0, 1, 0);
        if (msg_id != -1) {
            printf("Published live data to %s\n", MQTT_TOPIC_LIVE);
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

    cJSON_AddBoolToObject(json, "presence", g_presence);
    cJSON_AddBoolToObject(json, "fall_alarm", g_fall_alarm);
    cJSON_AddBoolToObject(json, "stay_still_alarm", g_stay_still_alarm);
    cJSON_AddNumberToObject(json, "movement_state", g_movement_state);
    cJSON_AddStringToObject(json, "movement_state_readable", movement_state_str(g_movement_state));
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
        vTaskDelay(pdMS_TO_TICKS(2000)); // Publish every 2 seconds
    }
}

// Settings JSON payload: configuration parameters.
void print_settings_json_payload(void)
{
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        printf("Error creating settings JSON object\n");
        return;
    }
    
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
    
    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("******************************************************** \n");
        printf("Settings JSON Payload: %s\n", json_str);
        free(json_str);
        printf("********************************************************\n");
    }
    cJSON_Delete(json);
}

// Product Information JSON payload
void print_product_info_payload(void)
{
    cJSON *json = cJSON_CreateObject();
    if(json == NULL) {
        printf("Error creating product info JSON object\n");
        return;
    }
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
                    else if(control == 0x80 && command == 0x10 && payload_len == 4) {
                        // Trajectory point report (4 bytes: 2 for X, 2 for Y)
                        // Extract X coordinate (first 2 bytes, little-endian)
                        g_traj_x = (int16_t)((data[i + 7] << 8) | data[i + 6]);
                        // Extract Y coordinate (next 2 bytes, little-endian)
                        g_traj_y = (int16_t)((data[i + 9] << 8) | data[i + 8]);
                        printf("Parsed Trajectory: X=%d, Y=%d\n", g_traj_x, g_traj_y);
                    }
                    else if(control == 0x83 && command == 0x0E && payload_len == 6) {
                        // Height Proportion Report:
                        // Byte 0-1: Total height count (16-bit)
                        // Byte 2: Proportion for 0-0.5 m
                        // Byte 3: Proportion for 0.5-1 m
                        // Byte 4: Proportion for 1-1.5 m
                        // Byte 5: Proportion for 1.5-2 m
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
                        printf("Parsed Height Proportion: Total=%d, 0-0.5=%d, 0.5-1=%d, 1-1.5=%d, 1.5-2=%d\n",
                               total, prop0, prop1, prop2, prop3);
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
                        g_installation_height = ((uint16_t)data[i+7] << 8) | data[i+6];
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
                    else {
                        // Other frames could be handled here if needed.
                        printf("Received unknown frame: control=0x%02X, command=0x%02X, payload_len=%d\n",
                               control, command, payload_len);
                    }
                    
                    // Move index past this frame
                    i += frame_total_length;
                } else {
                    i++;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task to print live data JSON payload
void live_json_print_task(void *arg)
{
    while(1) {
        print_live_json_payload();
        vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust refresh rate as needed
    }
}

// Task to print settings JSON payload
void settings_json_print_task(void *arg)
{
    while(1) {
        print_settings_json_payload();
        vTaskDelay(pdMS_TO_TICKS(5000));  // Adjust refresh rate as needed
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
    frame[7] = sum & 0xFF;             // Check digit
    frame[8] = FRAME_TAIL0;            // e.g., 0x54
    frame[9] = FRAME_TAIL1;            // e.g., 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Fall Detection Sensitivity update: %d\n", new_sensitivity);

    // Update the global variable to reflect the new setting
    g_fall_detection_sensitivity = new_sensitivity;
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
    
    // Append tail
    frame[11] = FRAME_TAIL0;           // 0x54
    frame[12] = FRAME_TAIL1;           // 0x43

    // Send the frame via UART
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
    printf("Sent Fall Duration update: %" PRIu32 " seconds\n", new_duration);

    // Optionally, update the global variable to reflect the new value.
    g_fall_duration = new_duration;
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
    // g_sitting_still_distance = new_distance;
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
    // Initialize NVS — it is used by Wi-Fi.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    // Initialize the TCP/IP stack and the event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

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
}

// Main entry point
void app_main(void)
{
    // Initialize Wi-Fi
    wifi_init_sta();
    
    // Initialize UART for communication with the radar module
    init_uart();
    
    // Initialize MQTT after Wi-Fi is connected
    mqtt_init();

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
}
