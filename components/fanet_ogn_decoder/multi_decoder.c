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
#include "ogn_conv.h"
#include <stdio.h>

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

/**
 * Decode raw OGN packet and process with additional conversions.
 * @param data  Raw packet bytes
 * @param len   Packet length
 * @param rssi  Received signal strength [dBm]
 * @param snr   Signal-to-noise ratio [dB]
 */
void ogn_decoder_decode(const uint8_t *data, int len, int rssi, int snr) {
    ogn_tracking_data_t track;

    if (unpack_ogn_tracking(data, len, &track, rssi, snr)) {
        // --- Additional processing using ogn_conv functions ---

        // 1. Convert OGN aircraft type to ADSB category
        uint8_t adsb_cat = AcftType_OGNtoADSB(track.acft_type);
        
        // 2. Get human-readable aircraft type name
        const char *acft_names[] = {
            "Glider", "Tow plane", "Helicopter", "Paraglider",
            "Hang glider", "Balloon", "UAV", "Other"
        };
        const char *acft_name = (track.acft_type <= OGN_ACFT_OTHER) 
                                ? acft_names[track.acft_type] 
                                : "Unknown";

        // 3. Convert speed from m/s to knots (1 m/s = 1.94384 kn)
        double speed_knots = track.speed * 1.94384;

        // 4. Convert climb rate from m/s to ft/min (1 m/s = 196.85 ft/min)
        double climb_fpm = track.climb * 196.85;

        // 5. Compute barometric temperature at GNSS altitude (example)
        float temp_kelvin = BaroTemp((float)track.alt_gnss);
        float temp_celsius = temp_kelvin - 273.15f;

        // 6. Decode Gray code (if any field was Gray-encoded, e.g., a hypothetical field)
        // uint8_t decoded = DecodeGray8(encoded_value);

        // Print enhanced tracking information
        printf("=== OGN Tracking ===\n");
        printf("Device: %s\n", track.common.devId);
        printf("Aircraft: %s (OGN type %d, ADSB cat %d)\n", acft_name, track.acft_type, adsb_cat);
        printf("Position: %.6f, %.6f\n", track.common.lat, track.common.lon);
        printf("Altitude GNSS: %.1f m\n", track.alt_gnss);
        printf("Speed: %.1f m/s (%.1f kn)\n", (float)track.speed, (float)speed_knots);
        printf("Climb: %.1f m/s (%.0f ft/min)\n", (float)track.climb, (float)climb_fpm);
        printf("Heading: %.1f deg, Turn: %d deg/s\n", (float)track.heading, track.turn_rate);
        printf("Temperature (ISA): %.1f °C\n", temp_celsius);
        printf("Fix: %s, DOP: %u\n", track.fix_mode ? "3D" : "2D", track.dop);
        printf("Emergency: %s\n", track.emergency ? "YES" : "NO");
        printf("===================\n");

        // Store in global array (original behavior)
        store_ogn_tracking_data(&track);
    }
}