/*
 * lora.h
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */

#ifndef LORA_H
#define LORA_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LoRa driver (radio + SPI)
 */
esp_err_t lora_init(void);

/**
 * @brief Configure LoRa parameters (SF, BW, CR, etc.)
 */
esp_err_t lora_configure(void);

/**
 * @brief Start RX task (background thread)
 */
esp_err_t lora_rx_start(void);

/**
 * @brief Stop RX task
 */
esp_err_t lora_rx_stop(void);

#ifdef __cplusplus
}
#endif

#endif
