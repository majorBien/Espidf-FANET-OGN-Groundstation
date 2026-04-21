/*
 * decoder.c
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */



#include "decoder.h"
#include "types.h"

#include <string.h>
#include <time.h>

#include "esp_log.h"

static const char *TAG = "FANET_DECODER";

/* external stores (from your types module) */
extern weatherData weatherStore[MAX_DEVICES];
extern trackingData trackingStore[MAX_DEVICES];

/* external unpack functions (from your cpp ported logic) */
bool unpack_weatherdata(uint8_t *buffer, weatherData *wData, float snr, float rssi);
bool unpack_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr);
bool unpack_ground_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr);

/* optional store functions */
int storeWeatherData(const weatherData *newData);
int storeTrackingData(const trackingData *newData);

/* FANET packet types */
#define FANET_PCK_TYPE_TRACKING        0x01
#define FANET_PCK_TYPE_NAME            0x02
#define FANET_PCK_TYPE_WEATHER         0x04
#define FANET_PCK_TYPE_GROUND_TRACKING 0x07


void fanet_decoder_init(void)
{
    ESP_LOGI(TAG, "Decoder init");
}


/**
 * helper: safe header parse
 */
static inline const fanet_header *get_header(const uint8_t *data)
{
    return (const fanet_header *)data;
}


void fanet_decoder_decode(const uint8_t *data, int len, int rssi, int snr)
{
    if (!data || len < sizeof(fanet_header)) {
        ESP_LOGW(TAG, "Invalid packet (len=%d)", len);
        return;
    }

    const fanet_header *header = get_header(data);

    switch (header->type)
    {
        case FANET_PCK_TYPE_WEATHER:
        {
            weatherData wd;
            memset(&wd, 0, sizeof(wd));

            if (unpack_weatherdata((uint8_t *)data, &wd, snr, rssi)) {
                int idx = storeWeatherData(&wd);

                ESP_LOGI(TAG,
                         "WEATHER id=%04X rssi=%d snr=%d lat=%.5f lon=%.5f T=%.1f",
                         wd.fanet_id,
                         rssi,
                         snr,
                         wd.lat,
                         wd.lon,
                         wd.temp);

                ESP_LOGI(TAG, "stored index=%d", idx);
            }
            break;
        }

        case FANET_PCK_TYPE_TRACKING:
        {
            trackingData td;
            memset(&td, 0, sizeof(td));

            if (unpack_trackingdata((uint8_t *)data, &td, rssi, snr)) {
                int idx = storeTrackingData(&td);

                ESP_LOGI(TAG,
                         "TRACK id=%04X rssi=%d snr=%d lat=%.5f lon=%.5f alt=%.1f spd=%.1f",
                         td.fanet_id,
                         rssi,
                         snr,
                         td.lat,
                         td.lon,
                         td.alt,
                         td.speed);

                ESP_LOGI(TAG, "stored index=%d", idx);
            }
            break;
        }

        case FANET_PCK_TYPE_GROUND_TRACKING:
        {
            trackingData td;
            memset(&td, 0, sizeof(td));

            if (unpack_ground_trackingdata((uint8_t *)data, &td, rssi, snr)) {
                int idx = storeTrackingData(&td);

                ESP_LOGI(TAG,
                         "GROUND id=%04X rssi=%d snr=%d lat=%.5f lon=%.5f state=%d",
                         td.fanet_id,
                         rssi,
                         snr,
                         td.lat,
                         td.lon,
                         td.state);

                ESP_LOGI(TAG, "stored index=%d", idx);
            }
            break;
        }

        case FANET_PCK_TYPE_NAME:
        {
            char name[32] = {0};

            int name_len = len - 4;
            if (name_len > (int)sizeof(name) - 1)
                name_len = sizeof(name) - 1;

            memcpy(name, &data[4], name_len);

            ESP_LOGI(TAG,
                     "NAME vid=%02X id=%04X name=%s",
                     header->vendor,
                     header->address,
                     name);
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown FANET type: 0x%02X", header->type);
            break;
    }
}
