/**
 * @file ogn_types.c
 * @brief OGN packet unpacking, storage and utilities - compatible with original C++
 */

 #include "ogn_types.h"
#include "ogn_conv.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static ogn_tracking_data_t ogn_tracking_store[MAX_OGN_DEVICES];

/*------------------------------------------------------------------------------
 * Helper: 24-bit sign extension (for latitude/longitude)
 *----------------------------------------------------------------------------*/
static inline int32_t sign_extend_24(uint32_t x) {
    if (x & 0x800000)
        x |= 0xFF000000;
    return (int32_t)x;
}

/*------------------------------------------------------------------------------
 * Helper: 9-bit sign extension (for raw pressure altitude)
 *----------------------------------------------------------------------------*/
static inline int16_t sign_extend_9(uint16_t x) {
    if (x & 0x100)
        x |= 0xFE00;
    return (int16_t)x;
}

/*------------------------------------------------------------------------------
 * Bit reader – extract up to 32 bits from a byte array (big-endian)
 *----------------------------------------------------------------------------*/
static uint32_t read_bits(const uint8_t *data, int *bitpos, int bits) {
    uint32_t value = 0;
    int byte = *bitpos / 8;
    int bit = *bitpos % 8;
    int remaining = bits;

    while (remaining > 0) {
        int bits_available = 8 - bit;
        int take = (remaining < bits_available) ? remaining : bits_available;
        uint32_t mask = (1 << take) - 1;
        value = (value << take) | ((data[byte] >> (8 - bit - take)) & mask);
        remaining -= take;
        bit += take;
        if (bit == 8) {
            bit = 0;
            byte++;
        }
    }
    *bitpos += bits;
    return value;
}

/*------------------------------------------------------------------------------
 * Convert address to device ID string (no VID, just 6-digit hex)
 *----------------------------------------------------------------------------*/
void ogn_addr_to_string(uint8_t vid, uint32_t addr, char *out) {
    (void)vid; // VID not used in OGN
    sprintf(out, "%06lX", (unsigned long)(addr & 0xFFFFFF));
}

/*------------------------------------------------------------------------------
 * Compare two common structures (ignore timestamp)
 *----------------------------------------------------------------------------*/
bool ogn_common_match(const ogn_common_t *a, const ogn_common_t *b) {
    return (a->address == b->address);
}

/*------------------------------------------------------------------------------
 * Copy common fields (timestamp is not overwritten)
 *----------------------------------------------------------------------------*/
void ogn_common_assign(ogn_common_t *dest, const ogn_common_t *src) {
    strcpy(dest->devId, src->devId);
    dest->vid = 0;
    dest->address = src->address;
    dest->rssi = src->rssi;
    dest->snr = src->snr;
    dest->lat = src->lat;
    dest->lon = src->lon;
}

/*------------------------------------------------------------------------------
 * Unpack OGN tracking (position) packet according to OGNTP specification
 *----------------------------------------------------------------------------*/
bool unpack_ogn_tracking(const uint8_t *buffer, int len, ogn_tracking_data_t *data,
                         int rssi, int snr) {
    if (!buffer || !data) return false;
    // OGN packet length: 20 bytes (no FEC) or 26 bytes (with FEC)
    if (len != 20 && len != 26) return false;

    memset(data, 0, sizeof(ogn_tracking_data_t));
    data->common.rssi = rssi;
    data->common.snr = (float)snr;
    data->common.timestamp = time(NULL);

    // -------------------------------
    // 1. Extract 24‑bit address and header flags
    // -------------------------------
    // According to OGN1_Packet, first 4 bytes contain:
    // - address (24 bits, big‑endian)
    // - addr_type (2 bits)
    // - addr_parity (1 bit)
    // - emergency (1 bit)
    // - relay_cnt (2 bits)
    // - other_data (1 bit)
    // - custom_enc (1 bit)
    uint32_t addr_raw = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    data->common.address = addr_raw;
    ogn_addr_to_string(0, addr_raw, data->common.devId);

    uint8_t header_byte = buffer[3];
    data->addr_type   = (ogn_addr_type_t)((header_byte >> 6) & 0x03);
    data->addr_parity = (header_byte >> 5) & 0x01;
    data->emergency   = (header_byte >> 4) & 0x01;
    data->relay_cnt   = (header_byte >> 2) & 0x03;
    data->other_data  = (header_byte >> 1) & 0x01;
    data->custom_enc  = header_byte & 0x01;

    // If 'other_data' flag is set, this is not a position packet
    if (data->other_data) return false;

    // -------------------------------
    // 2. Parse payload (16 bytes) using bit reader
    // -------------------------------
    int bitpos = 32; // 4 bytes already consumed

    // Aircraft type (4 bits)
    data->acft_type = (ogn_aircraft_type_t)read_bits(buffer, &bitpos, 4);
    // Stealth flag (1 bit)
    data->stealth = read_bits(buffer, &bitpos, 1) != 0;
    // UTC second (6 bits)
    data->time_sec = read_bits(buffer, &bitpos, 6);

    // Latitude (24 bits, signed, resolution 0.0008/60 deg)
    uint32_t lat_raw = read_bits(buffer, &bitpos, 24);
    data->common.lat = (double)sign_extend_24(lat_raw) * (0.0008 / 60.0);

    // Longitude (24 bits, signed, resolution 0.0016/60 deg)
    uint32_t lon_raw = read_bits(buffer, &bitpos, 24);
    data->common.lon = (double)sign_extend_24(lon_raw) * (0.0016 / 60.0);

    // GNSS altitude (14 bits, unsigned, 1 m resolution)
    uint32_t alt_gnss_raw = read_bits(buffer, &bitpos, 14);
    data->alt_gnss = (double)alt_gnss_raw;

    // Pressure altitude difference (9 bits, signed, 1 m resolution)
    uint32_t alt_pres_raw = read_bits(buffer, &bitpos, 9);
    data->alt_pressure = (double)sign_extend_9(alt_pres_raw);

    // Climb rate (9 bits, variable‑rate encoded)
    uint32_t climb_raw = read_bits(buffer, &bitpos, 9);
    // climb_raw is 9‑bit value that must be decoded using DecodeSR2V6
    data->climb = DecodeSR2V6((int16_t)climb_raw);   // returns 0.1 m/s

    // Speed (10 bits, variable‑rate encoded)
    uint32_t speed_raw = read_bits(buffer, &bitpos, 10);
    data->speed = DecodeUR2V8((uint16_t)speed_raw);  // returns 0.1 m/s

    // Heading (10 bits, variable‑rate encoded)
    uint32_t heading_raw = read_bits(buffer, &bitpos, 10);
    uint16_t heading_tenth = DecodeUR2V8((uint16_t)heading_raw);
    data->heading = heading_tenth;                   // stored as 0.1 deg

    // Turn rate (8 bits, variable‑rate encoded)
    uint32_t turn_raw = read_bits(buffer, &bitpos, 8);
    data->turn_rate = DecodeSR2V5((int8_t)turn_raw); // returns 0.1 deg/s

    // Fix mode (1 bit)
    data->fix_mode = read_bits(buffer, &bitpos, 1) != 0;
    // Fix quality (2 bits)
    data->fix_quality = (ogn_fix_quality_t)read_bits(buffer, &bitpos, 2);
    // DOP (6 bits)
    data->dop = read_bits(buffer, &bitpos, 6);
    // Reserved 4 bits (ignored)

    return true;
}

/*------------------------------------------------------------------------------
 * Store tracking data in global array
 *----------------------------------------------------------------------------*/
int store_ogn_tracking_data(const ogn_tracking_data_t *newData) {
    if (!newData) return -1;

    for (int i = 0; i < MAX_OGN_DEVICES; ++i) {
        if (ogn_tracking_store[i].common.timestamp != 0 &&
            ogn_common_match(&ogn_tracking_store[i].common, &newData->common)) {
            ogn_tracking_store[i] = *newData;
            return i;
        }
    }

    for (int i = 0; i < MAX_OGN_DEVICES; ++i) {
        if (ogn_tracking_store[i].common.timestamp == 0) {
            ogn_tracking_store[i] = *newData;
            return i;
        }
    }

    int oldest_idx = 0;
    time_t oldest_ts = ogn_tracking_store[0].common.timestamp;
    for (int i = 1; i < MAX_OGN_DEVICES; ++i) {
        if (ogn_tracking_store[i].common.timestamp < oldest_ts) {
            oldest_ts = ogn_tracking_store[i].common.timestamp;
            oldest_idx = i;
        }
    }
    ogn_tracking_store[oldest_idx] = *newData;
    return oldest_idx;
}

/*------------------------------------------------------------------------------
 * Debug print of tracking data
 *----------------------------------------------------------------------------*/
void print_ogn_tracking(const ogn_tracking_data_t *d) {
    printf("=== OGN Tracking ===\n");
    printf("Device: %s\n", d->common.devId);
    printf("Position: %.6f, %.6f\n", d->common.lat, d->common.lon);
    printf("Altitude GNSS: %.1f m\n", d->alt_gnss);
    printf("Speed: %.1f m/s (%.1f kn)\n", d->speed * 0.1f, d->speed * 0.194384f);
    printf("Climb: %.1f m/s (%.0f ft/min)\n", d->climb * 0.1f, d->climb * 19.685f);
    printf("Heading: %.1f deg, Turn: %.1f deg/s\n", d->heading * 0.1f, d->turn_rate * 0.1f);
    printf("Aircraft type: %d, Emergency: %s\n", d->acft_type, d->emergency ? "YES" : "NO");
    printf("Fix: %s, DOP: %u\n", d->fix_mode ? "3D" : "2D", d->dop);
    printf("===================\n");
}