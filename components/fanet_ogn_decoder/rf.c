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
#include "multi_decoder.h"
#include "ogn_types.h"

static const char *TAG = "LORA";

static TaskHandle_t rx_task_handle = NULL;



/**
 * =========================
 * RX TASK
 * =========================
 */
/**
 * @brief Helper to log raw packet bytes as hex (if ESP_LOG_BUFFER_HEX not available)
 */
static void log_hex(const char *tag, const uint8_t *buf, int len) {
    char hex_str[255 * 3 + 1];
    int pos = 0;
    for (int i = 0; i < len && pos < (int)sizeof(hex_str) - 3; i++) {
        pos += sprintf(hex_str + pos, "%02X ", buf[i]);
    }
    if (pos > 0) hex_str[pos - 1] = '\0'; // remove trailing space
    ESP_LOGI(tag, "RAW HEX: %s", hex_str);
}

static void lora_rx_task(void *arg) {
    ESP_LOGI(TAG, "RX task started");
    uint8_t buf[255];

    while (1) {
        uint8_t rxLen = LoRaReceive(buf, sizeof(buf));
        if (rxLen > 0) {
            int8_t rssi, snr;
            GetPacketStatus(&rssi, &snr);

            // --- Debug output: raw packet bytes ---
            ESP_LOGI(TAG, "Received %d bytes, RSSI=%d, SNR=%d", rxLen, rssi, snr);
            log_hex(TAG, buf, rxLen);               // print hex dump
            // Optional: print first 4 bytes as ASCII (if printable)
            if (rxLen >= 4) {
                ESP_LOGI(TAG, "First 4 bytes: 0x%02X%02X%02X%02X",
                         buf[0], buf[1], buf[2], buf[3]);
            }
            // --- End debug ---

            // Decode as OGN only if it looks like a valid OGN packet (at least 20 bytes)
            if (rxLen >= 20) {
				//fanet_decoder_decode(buf, rxLen, rssi, snr);
                ogn_decoder_decode(buf, rxLen, rssi, snr);
            } else {
                ESP_LOGW(TAG, "Packet too short for OGN (%d bytes)", rxLen);
            }
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

