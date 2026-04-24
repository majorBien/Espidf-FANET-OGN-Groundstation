/*
 * lora.c
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */

#include "rf.h"
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
        if (rxLen > 0){
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

esp_err_t rf_init_module(int8_t txPower, uint32_t freq){
    ESP_LOGI(TAG, "LoRa init");

    LoRaInit();
    float tcxo = 3.3;
    bool ldo = true;
    ESP_LOGW(TAG, "Enable TCXO");

    if (LoRaBegin(freq, txPower, tcxo, ldo) != 0){
        ESP_LOGE(TAG, "LoRa init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t rf_configure(uint8_t mode, void *config){
    if (mode == RF_MODE_FANET){
        // =========================
        // FANET → LoRa
        // =========================
        lora_receiver_t *cfg = (lora_receiver_t *)config;

        cfg->sf = 7;
        cfg->bw = SX126X_LORA_BW_125_0;
        cfg->cr = SX126X_LORA_CR_4_5;
        cfg->preamble = 8;
        cfg->payload = 0;
        cfg->crc = true;
        cfg->invert = false;

        ESP_LOGI(TAG, "RF: FANET (LoRa)");

        LoRaConfig(
            cfg->sf,
            cfg->bw,
            cfg->cr,
            cfg->preamble,
            cfg->payload,
            cfg->crc,
            cfg->invert
        );
    }
    else if (mode == RF_MODE_OGN){
        // =========================
        // OGN → GFSK
        // =========================
        gfsk_receiver_t *cfg = (gfsk_receiver_t *)config;

        cfg->bitrate = 100000;
        cfg->fdev = 50000;
        cfg->rxBw = SX126X_GFSK_RX_BW_156_2;
        cfg->preambleLength = 32;
        cfg->payloadLen = 0;
        cfg->crcOn = true;
        cfg->whiteningOn = false;

        ESP_LOGI(TAG, "RF: OGN (GFSK)");

        GFSKConfig(
            cfg->bitrate,
            cfg->fdev,
            cfg->rxBw,
            cfg->preambleLength,
            cfg->payloadLen,
            cfg->crcOn,
            cfg->whiteningOn
        );
    }
    else{
        ESP_LOGE(TAG, "Invalid RF mode");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "RF configured");
    return ESP_OK;
}

esp_err_t rf_rx_start(void){
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

esp_err_t rf_rx_stop(void){
    if (!rx_task_handle)
        return ESP_OK;

    vTaskDelete(rx_task_handle);
    rx_task_handle = NULL;

    return ESP_OK;
}


