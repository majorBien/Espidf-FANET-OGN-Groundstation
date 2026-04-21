/*
 * lora.c
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */

#include "lora.h"
#include "ra01s.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "decoder.h"

static const char *TAG = "LORA";

static TaskHandle_t rx_task_handle = NULL;

/**
 * =========================
 * RX TASK
 * =========================
 */
static void lora_rx_task(void *arg){
    ESP_LOGI(TAG, "RX task started");

    uint8_t buf[255];

    while (1){
        uint8_t rxLen = LoRaReceive(buf, sizeof(buf));

        if (rxLen > 0)
        {
            int8_t rssi, snr;
            GetPacketStatus(&rssi, &snr);
            fanet_decoder_decode(buf, rxLen, rssi, snr);

            ESP_LOGI(TAG, "%d byte packet:[%.*s]", rxLen, rxLen, buf);
            ESP_LOGI(TAG, "rssi=%d snr=%d", rssi, snr);
        }

        vTaskDelay(1);
    }
}

/**
 * =========================
 * PUBLIC API
 * =========================
 */

esp_err_t lora_init(void){
    ESP_LOGI(TAG, "LoRa init");

    LoRaInit();

    int8_t txPower = 22;
    uint32_t freq = 868000000;

    float tcxo = 3.3;
    bool ldo = true;

    ESP_LOGW(TAG, "Enable TCXO");

    if (LoRaBegin(freq, txPower, tcxo, ldo) != 0){
        ESP_LOGE(TAG, "LoRa init failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t lora_configure(void){
    uint8_t sf = 7;
    uint8_t bw = 4;
    uint8_t cr = 1;
    uint16_t preamble = 8;
    uint8_t payload = 0;
    bool crc = true;
    bool invert = false;

    ESP_LOGI(TAG, "LoRa config");

    LoRaConfig(sf, bw, cr, preamble, payload, crc, invert);

    return ESP_OK;
}

esp_err_t lora_rx_start(void){
    if (rx_task_handle)
        return ESP_ERR_INVALID_STATE;

    xTaskCreate(
        lora_rx_task,
        "lora_rx",
        4096,
        NULL,
        5,
        &rx_task_handle
    );

    return ESP_OK;
}

esp_err_t lora_rx_stop(void){
    if (!rx_task_handle)
        return ESP_OK;

    vTaskDelete(rx_task_handle);
    rx_task_handle = NULL;

    return ESP_OK;
}


