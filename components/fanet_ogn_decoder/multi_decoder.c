/*
 * decoder.c
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 *  Modified: dostosowany do nowego types.h/types.c
 */

#include "multi_decoder.h"
#include "ogn_types.h"
#include "fanet_types.h"

#include <string.h>
#include <stdint.h>
#include <time.h>

#include "esp_log.h"

static const char *TAG = "FANET_DECODER";

/* =========================
 * RAW HEADER PARSER
 * ========================= */
static inline void fanet_read_header(const uint8_t *data,
                                      uint8_t *type,
                                      uint8_t *vendor,
                                      uint16_t *addr){
    *type   = data[0] & 0x3F;
    *vendor = data[1];
    *addr   = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
}


/* =========================
 * MAIN DECODER
 * ========================= */
void fanet_decoder_decode(const uint8_t *data, int len, int rssi, int snr){
    if (!data || len < 4) {
        ESP_LOGW(TAG, "Packet too small (%d)", len);
        return;
    }
    uint8_t type;
    uint8_t vendor;
    uint16_t addr;

    fanet_read_header(data, &type, &vendor, &addr);
    (void)vendor;   // vendor already inside packet, unpack functions will read it

    switch (type){
    /* ================= TRACKING (type 1) ================= */
    case FANET_PCK_TYPE_TRACKING:{
        if (len < sizeof(fanet_packet_t1)) {
            ESP_LOGW(TAG, "TRACK packet too short (%d < %d)", len, (int)sizeof(fanet_packet_t1));
            return;
        }

        trackingData td;
        memset(&td, 0, sizeof(td));

        if (!unpack_trackingdata((uint8_t*)data, &td, rssi, snr)) {
            ESP_LOGW(TAG, "Failed to unpack tracking data (invalid speed)");
            return;
        }

        int idx = storeTrackingData(&td);

        ESP_LOGI(TAG,
                 "TRACK id=%04X rssi=%d snr=%.1f lat=%.5f lon=%.5f alt=%.1f spd=%.1f climb=%.1f heading=%.1f acft=%d",
                 td.common.fanet_id, td.common.rssi, td.common.snr,
                 td.common.lat, td.common.lon, td.alt, td.speed, td.climb, td.heading,
                 td.aircraftType);
        ESP_LOGI(TAG, "stored index=%d", idx);
        break;
    }

    /* ================= WEATHER (type 4) ================= */
    case FANET_PCK_TYPE_WEATHER:
    {
        if (len < sizeof(fanet_packet_t4)) {
            ESP_LOGW(TAG, "WEATHER packet too short (%d < %d)", len, (int)sizeof(fanet_packet_t4));
            return;
        }

        weatherData wd;
        memset(&wd, 0, sizeof(wd));

        if (!unpack_weatherdata((uint8_t*)data, &wd, (float)snr, (float)rssi)) {
            ESP_LOGW(TAG, "Failed to unpack weather data");
            return;
        }

        int idx = storeWeatherData(&wd);

        ESP_LOGI(TAG,
                 "WEATHER id=%04X rssi=%d snr=%.1f lat=%.5f lon=%.5f T=%.1f hum=%.1f%% press=%.1fhPa wind=%.1f/%.1f",
                 wd.common.fanet_id, wd.common.rssi, wd.common.snr,
                 wd.common.lat, wd.common.lon, wd.temp, wd.Humidity, wd.Baro,
                 wd.wSpeed, wd.wHeading);
        ESP_LOGI(TAG, "stored index=%d", idx);
        break;
    }

    /* ================= GROUND TRACKING (type 7) ================= */
    case FANET_PCK_TYPE_GROUND_TRACKING:{
        if (len < sizeof(fanet_packet_t7)) {
            ESP_LOGW(TAG, "GROUND packet too short (%d < %d)", len, (int)sizeof(fanet_packet_t7));
            return;
        }
        trackingData td;
        memset(&td, 0, sizeof(td));

        if (!unpack_ground_trackingdata((uint8_t*)data, &td, rssi, snr)) {
            ESP_LOGW(TAG, "Failed to unpack ground tracking data");
            return;
        }

        int idx = storeTrackingData(&td);

        ESP_LOGI(TAG,
                 "GROUND id=%04X rssi=%d snr=%.1f lat=%.5f lon=%.5f state=%d (%s)",
                 td.common.fanet_id, td.common.rssi, td.common.snr,
                 td.common.lat, td.common.lon, td.state,
                 (td.state < 16) ? trck_state_names[td.state] : "unknown");
        ESP_LOGI(TAG, "stored index=%d", idx);
        break;
    }

    /* ================= NAME (type 2) ================= */
    case FANET_PCK_TYPE_NAME:
    {
        if (len <= 4) {
            ESP_LOGW(TAG, "NAME packet too short");
            return;
        }

        char name[32] = {0};
        int name_len = len - 4;
        if (name_len > (int)sizeof(name) - 1)
            name_len = sizeof(name) - 1;
        memcpy(name, &data[4], name_len);

        ESP_LOGI(TAG, "NAME vid=%02X id=%04X name=%s", vendor, addr, name);
        break;
    }

    /* ================= UNKNOWN ================= */
    default:
        ESP_LOGW(TAG, "Unknown FANET type=0x%02X len=%d", type, len);
        break;
    }
}

void ogn_decoder_decode(const uint8_t *data, int len, int rssi, int snr) {
    ogn_tracking_data_t track;

    if (unpack_ogn_tracking(data, len, &track, rssi, snr)) {
        store_ogn_tracking_data(&track);
        print_ogn_tracking(&track);
    }
}