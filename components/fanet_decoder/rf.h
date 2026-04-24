/*
 * lora.h
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */

#ifndef LORA_H
#define LORA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RF_MODE_FANET 0
#define RF_MODE_OGN   1


/*
* @brief lora config structure (SF, BW, CR, etc.)
*/
typedef struct {
    uint8_t sf;
    uint8_t bw;
    uint8_t cr;
    uint16_t preamble;
    uint8_t payload;
    bool crc;
    bool invert;
} lora_receiver_t;

/*
* @brief gfsk config structure (SF, BW, CR, etc.)
*/
typedef struct {
	uint32_t bitrate;
	uint32_t fdev;
	uint8_t rxBw;
	uint16_t preambleLength;
	uint8_t payloadLen;
	bool crcOn;
	bool whiteningOn;
} gfsk_receiver_t;
/**
 * @brief Initialize LoRa driver (radio + SPI)
 */
esp_err_t rf_init_module(int8_t txPower, uint32_t freq);

/**
 * @brief Configure LoRa parameters (SF, BW, CR, etc.)
 */
esp_err_t rf_configure(uint8_t mode, void *config);

/**
 * @brief Start RX task (background thread)
 */
esp_err_t rf_rx_start(void);

/**
 * @brief Stop RX task
 */
esp_err_t rf_rx_stop(void);

#ifdef __cplusplus
}
#endif

#endif
