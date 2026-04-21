#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "System start");

    if (lora_init() != ESP_OK){
        ESP_LOGE(TAG, "LoRa init failed");
        return;
    }

    if (lora_configure() != ESP_OK){
        ESP_LOGE(TAG, "LoRa config failed");
        return;
    }

    if (lora_rx_start() != ESP_OK){
        ESP_LOGE(TAG, "LoRa RX start failed");
        return;
    }

    ESP_LOGI(TAG, "LoRa running");
}