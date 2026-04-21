/*
 * decoder.h
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize FANET decoder (optional future use)
 */
void fanet_decoder_init(void);

/**
 * @brief Decode raw FANET packet
 *
 * @param data RX buffer from LoRa PHY
 * @param len  buffer length
 * @param rssi received signal strength
 * @param snr  signal-to-noise ratio
 */
void fanet_decoder_decode(const uint8_t *data, int len, int rssi, int snr);

#ifdef __cplusplus
}
#endif