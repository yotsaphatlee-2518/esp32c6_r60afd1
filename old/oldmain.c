#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_event.h"
#include "freertos/event_groups.h"

// Include your libraries here
// #include <WiFi.h>
// #include <WiFiManager.h>
// #include <PubSubClient.h>

// Define your constants and variables
#define WIFI_RESET_PIN GPIO_NUM_0
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16
#define UART_NUM UART_NUM_1

// MQTT configurations
const char* mqtt_server = "192.168.1.67";
const int mqtt_port = 1883;
const char* mqtt_user = "raspi5nr";
const char* mqtt_password = "12345";
const char* topic = "Fulldetector";

esp_mqtt_client_handle_t client;

// Add sensor variables
int presenceDetected = 1;
int motionDetected = 0; // 0: no motion, 1: low motion, 2: high motion
int activityLevel = 1;
int fallDetected = 0;
int stationaryState = 0; // 0: no stationary object, 1: stationary object present
float heightPercentage[4] = {0}; // height percentage in each range
int fallSensitivity = 0; // fall detection sensitivity (0-3)
int fallTime = 0; // fall detection time (5-180 seconds)
int residenceTime = 0; // stationary detection time (60-3600 seconds)
int residenceSwitch = 0; // stationary detection switch (0: off, 1: on)
int heightAccumulationTime = 0; // height accumulation time (0-300 seconds)

// Buffer for received data
static uint8_t buffer[128];
static int buffer_index = 0;

// Define the event group and connection bit
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// Function prototypes
void setupWiFiManager();
void setupMQTT();
void reconnectMQTT();
void sendMQTTData();
void getUserInput();
void processPacket();
void querySensorData();

// Add prototypes for the functions that are used before their definitions
void queryHeightPercentage();
void queryStationaryState();
void queryFallStatus();
void queryFallTime();
void queryResidenceTime();
void queryResidenceSwitch();
void queryFallSensitivity();
void queryHeightAccumulationTime();

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the event group
    s_wifi_event_group = xEventGroupCreate();

    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize WiFi
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set WiFi mode to STA (Station)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Setup WiFi Manager
    // setupWiFiManager();

    // Get user input
    getUserInput();

    // Setup MQTT
    setupMQTT();

    // Initialize UART for sensor communication
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);

    // Register the event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    // Main loop
    while (1) {
        // Ensure MQTT is connected - modified to avoid using esp_mqtt_client_is_connected
        // reconnectMQTT();

        // Query sensor data
        querySensorData();

        // Process and send MQTT data
        processPacket();
        sendMQTTData();

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
}

// Implement your functions here

void setupWiFiManager() {
    // Configure WiFi reset button
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WIFI_RESET_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Check if reset button is pressed
    if (gpio_get_level(WIFI_RESET_PIN) == 0) {
        ESP_LOGI("WIFI", "Wi-Fi reset button pressed. Resetting WiFi settings...");
        // Reset Wi-Fi settings logic here if needed
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for reset to complete
    }

    // Connect to WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Fulldetec",
            .password = "", // Add your WiFi password here
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdTRUE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WIFI", "Connected to WiFi");
    } else {
        ESP_LOGI("WIFI", "Failed to connect to WiFi");
    }

    // Get IP address
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
    ESP_LOGI("WIFI", "IP Address: " IPSTR, IP2STR(&ip_info.ip));
}

void setupMQTT() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.1.67", // Use the correct field for URI
        .credentials.username = mqtt_user,
        .credentials.authentication.password = mqtt_password,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void reconnectMQTT() {
    esp_mqtt_client_reconnect(client);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}

void sendMQTTData() {
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{ \"device_id\": \"Fulldetector2\", \"device_type\": \"R60AFD1\", \"presence\": %d, \"motion\": %d, \"activity_level\": %d, \"fall_detected\": %d, \"stationary_state\": %d,  \"height0\": %.2f,  \"height1\": %.2f,  \"height2\": %.2f,  \"height3\": %.2f}",
             presenceDetected, motionDetected, activityLevel, fallDetected, stationaryState, 
             heightPercentage[0], heightPercentage[1], heightPercentage[2], heightPercentage[3]);
    ESP_LOGI("MQTT", "Payload: %s", payload);
    // int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
    // if (msg_id != -1) {
    //     ESP_LOGI("MQTT", "Message sent successfully: %s", payload);
    // } else {
    //     ESP_LOGE("MQTT", "Failed to send message");
    // }
}

void getUserInput() {
    ESP_LOGI("CONFIG", "Please enter values for R60AFD1 sensor:");
    
    // In a real implementation, you would get these values from a user interface
    // For now, we'll set default values
    fallDetected = 0;
    fallSensitivity = 2;
    fallTime = 30;
    residenceTime = 300;
    residenceSwitch = 1;
    heightAccumulationTime = 60;
    
    ESP_LOGI("CONFIG", "Configuration complete with default values");
}

void processPacket() {
    uint8_t data[128];
    size_t length = 0;
    
    // Read data from UART
    uart_get_buffered_data_len(UART_NUM, &length);
    if (length > 0) {
        length = uart_read_bytes(UART_NUM, data, length, 0);
        
        // Process the data
        for (size_t i = 0; i < length; i++) {
            buffer[buffer_index++] = data[i];
            
            // Check for frame tail (0x54 0x43)
            if (buffer_index >= 7 && buffer[buffer_index - 2] == 0x54 && buffer[buffer_index - 1] == 0x43) {
                // Log raw data
                ESP_LOGI("SENSOR", "Raw Data from Sensor:");
                for (int j = 0; j < buffer_index; j++) {
                    ESP_LOGI("SENSOR", "%02X ", buffer[j]);
                }
                
                // Process the packet
                if (buffer[0] == 0x53 && buffer[1] == 0x59) {
                    switch (buffer[2]) {
                        case 0x80: // General data type
                            switch (buffer[3]) {
                                case 0x01: // presenceDetected
                                    presenceDetected = buffer[6];
                                    break;
                                case 0x02: // motionDetected
                                    motionDetected = buffer[6];
                                    ESP_LOGI("SENSOR", "Updated Motion Detected: %d", motionDetected);
                                    break;
                                case 0x03: // activityLevel
                                    activityLevel = buffer[6];
                                    fallDetected = (activityLevel >= 85) ? 1 : 0;
                                    break;
                            }
                            break;
                        case 0x83: // Height data type
                            if (buffer[3] == 0x85) { // Query Stationary Residence State
                                stationaryState = buffer[6];
                                ESP_LOGI("SENSOR", "Stationary Residence State: %s", 
                                         stationaryState ? "Occupied" : "Unoccupied");
                            }
                            if (buffer[3] == 0x8E) { // Query Height Percentage
                                heightPercentage[0] = buffer[6] * 0.01; // 0-0.5m
                                heightPercentage[1] = buffer[7] * 0.01; // 0.5-1m
                                heightPercentage[2] = buffer[8] * 0.01; // 1-1.5m
                                heightPercentage[3] = buffer[9] * 0.01; // 1.5-2m
                                
                                ESP_LOGI("SENSOR", "Height Percentages: %.2f %.2f %.2f %.2f", 
                                         heightPercentage[0], heightPercentage[1], 
                                         heightPercentage[2], heightPercentage[3]);
                            }
                            break;
                    }
                }
                
                // Reset buffer index for new packet
                buffer_index = 0;
            }
        }
    }
}

void querySensorData() {
    static unsigned long lastQueryTime = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (current_time - lastQueryTime > 10000) {
        queryFallStatus();
        queryFallTime();
        queryResidenceTime();
        queryResidenceSwitch();
        queryFallSensitivity();
        queryHeightPercentage();
        queryStationaryState();
        queryHeightAccumulationTime();
        lastQueryTime = current_time;
    }
}

// Add checksum calculation function
uint8_t calculateChecksum(uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

// Add sensor query functions
void queryHeightPercentage() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x8E, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Height Percentage...");
}

void queryStationaryState() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x85, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Stationary Residence State...");
}

void queryFallStatus() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x81, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Fall Status...");
}

void queryFallTime() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x8C, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Fall Time...");
}

void queryResidenceTime() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x8A, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Residence Time...");
}

void queryResidenceSwitch() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x8B, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Residence Switch...");
}

void queryFallSensitivity() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x8D, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Fall Sensitivity...");
}

void queryHeightAccumulationTime() {
    uint8_t command[10] = {0x53, 0x59, 0x83, 0x8F, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
    command[7] = calculateChecksum(command, 7);
    uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    ESP_LOGI("SENSOR", "Querying Height Accumulation Time...");
} 