#include "ota_update.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_crt_bundle.h"

#define TAG "OTA_UPDATE"

static void ota_task(void *pvParameter) {
    const char *url = (const char *)pvParameter;
    ESP_LOGI(TAG, "Starting OTA from URL: %s", url);
    esp_http_client_config_t http_config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA Failed");
    }
    vTaskDelete(NULL);
}

void ota_update_start(const char *url) {
    xTaskCreate(&ota_task, "ota_task", 8192, (void *)url, 5, NULL);
} 