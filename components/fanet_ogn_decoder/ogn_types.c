/**
 * @file ogn_types.c
 * @brief OGN packet unpacking, storage and utilities
 */

#include "ogn_types.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/*------------------------------------------------------------------------------
 * Global storage
 *----------------------------------------------------------------------------*/
ogn_tracking_data_t ogn_tracking_store[MAX_OGN_DEVICES];

/*------------------------------------------------------------------------------
 * Helper: 24-bit sign extension
 *----------------------------------------------------------------------------*/
static inline int32_t sign_extend_24(uint32_t x) {
    if (x & 0x800000)
        x |= 0xFF000000;
    return (int32_t)x;
}

/*------------------------------------------------------------------------------
 * Convert VID + address to string (e.g. "12ABCD")
 *----------------------------------------------------------------------------*/
void ogn_addr_to_string(uint8_t vid, uint32_t addr, char *out) {
    sprintf(out, "%02X%06lX", vid, (unsigned long)(addr & 0xFFFFFF));
}

/*------------------------------------------------------------------------------
 * Compare two common structures (ignore timestamp)
 *----------------------------------------------------------------------------*/
bool ogn_common_match(const ogn_common_t *a, const ogn_common_t *b) {
    return (a->vid == b->vid) && (a->address == b->address);
}

/*------------------------------------------------------------------------------
 * Copy common fields (timestamp is not overwritten)
 *----------------------------------------------------------------------------*/
void ogn_common_assign(ogn_common_t *dest, const ogn_common_t *src) {
    strcpy(dest->devId, src->devId);
    dest->vid = src->vid;
    dest->address = src->address;
    dest->rssi = src->rssi;
    dest->snr = src->snr;
    dest->lat = src->lat;
    dest->lon = src->lon;
    /* timestamp intentionally not copied */
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
 * Unpack OGN tracking (position) packet
 *----------------------------------------------------------------------------*/
bool unpack_ogn_tracking(const uint8_t *buffer, int len, ogn_tracking_data_t *data,
                         int rssi, int snr) {
    if (!buffer || len < 20 || !data) return false;

    // clear target
    memset(data, 0, sizeof(ogn_tracking_data_t));

    // common fields
    data->common.rssi = rssi;
    data->common.snr = (float)snr;
    data->common.timestamp = time(NULL);

    // --- header (bytes 0-3) ---
    data->common.vid = buffer[1];                         // vendor ID (byte1)
    uint32_t addr24 = ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 8) | buffer[4];
    data->common.address = addr24;
    ogn_addr_to_string(data->common.vid, addr24, data->common.devId);

    // bitstream parsing after header
    int bitpos = 32; // 4 bytes header consumed

    // address_type (2 bits)
    data->addr_type = (ogn_addr_type_t)read_bits(buffer, &bitpos, 2);
    // address_parity (1)
    data->addr_parity = read_bits(buffer, &bitpos, 1) != 0;
    // emergency (1)
    data->emergency = read_bits(buffer, &bitpos, 1) != 0;
    // relay_count (2)
    data->relay_cnt = read_bits(buffer, &bitpos, 2);
    // other_data (1)
    data->other_data = read_bits(buffer, &bitpos, 1) != 0;
    // custom_encrypt (1)
    data->custom_enc = read_bits(buffer, &bitpos, 1) != 0;

    // if other_data is set, this is not a position packet -> return false
    if (data->other_data) return false;

    // aircraft_type (4)
    data->acft_type = (ogn_aircraft_type_t)read_bits(buffer, &bitpos, 4);
    // stealth (1)
    data->stealth = read_bits(buffer, &bitpos, 1) != 0;
    // time_sec (6) – seconds of UTC minute
    data->time_sec = read_bits(buffer, &bitpos, 6);

    // latitude (24 bits, signed)
    uint32_t lat_raw = read_bits(buffer, &bitpos, 24);
    int32_t lat_signed = sign_extend_24(lat_raw);
    data->common.lat = (double)lat_signed * (0.0008 / 60.0); // resolution = 0.0008/60 deg

    // longitude (24 bits, signed)
    uint32_t lon_raw = read_bits(buffer, &bitpos, 24);
    int32_t lon_signed = sign_extend_24(lon_raw);
    data->common.lon = (double)lon_signed * (0.0016 / 60.0);

    // GNSS altitude (14 bits, unsigned) – resolution 1 m
    uint32_t alt_gnss_raw = read_bits(buffer, &bitpos, 14);
    data->alt_gnss = (double)alt_gnss_raw; // 0..61432 m

    // Pressure altitude (9 bits, signed, resolution 1 m) – difference from GNSS
    uint32_t alt_pres_raw = read_bits(buffer, &bitpos, 9);
    int16_t alt_pres_signed = (alt_pres_raw & 0x100) ? (int16_t)(alt_pres_raw | 0xFE00) : (int16_t)alt_pres_raw;
    data->alt_pressure = (double)alt_pres_signed; // +/-255 m

    // Climb rate (9 bits, signed, resolution 0.1 m/s)
    uint32_t climb_raw = read_bits(buffer, &bitpos, 9);
    int16_t climb_signed = (climb_raw & 0x100) ? (int16_t)(climb_raw | 0xFE00) : (int16_t)climb_raw;
    data->climb = (double)climb_signed * 0.1;

    // Speed (10 bits, unsigned, resolution 0.1 m/s)
    uint32_t speed_raw = read_bits(buffer, &bitpos, 10);
    data->speed = (double)speed_raw * 0.1; // max 383.2 m/s

    // Heading (10 bits, unsigned, resolution 0.1 deg)
    uint32_t heading_raw = read_bits(buffer, &bitpos, 10);
    data->heading = (double)heading_raw * 0.1;

    // Turn rate (8 bits, signed, resolution 0.1 deg/s? spec says 0.1 .. 0.8, using 0.1)
    uint32_t turn_raw = read_bits(buffer, &bitpos, 8);
    int8_t turn_signed = (turn_raw & 0x80) ? (int8_t)(turn_raw | 0xFFFFFF00) : (int8_t)turn_raw;
    data->turn_rate = turn_signed; // store as integer deg/s (scaled 1:1), actual resolution will be 0.1 if needed

    // GPS fix mode (1 bit)
    data->fix_mode = read_bits(buffer, &bitpos, 1) != 0;
    // GPS fix quality (2 bits)
    data->fix_quality = (ogn_fix_quality_t)read_bits(buffer, &bitpos, 2);
    // GPS DOP (6 bits)
    data->dop = read_bits(buffer, &bitpos, 6);
    // reserved 4 bits (ignore)
    // read_bits(buffer, &bitpos, 4); // optional

    return true;
}

/*------------------------------------------------------------------------------
 * Store tracking data in global array
 *----------------------------------------------------------------------------*/
int store_ogn_tracking_data(const ogn_tracking_data_t *newData) {
    if (!newData) return -1;

    // find existing device
    for (int i = 0; i < MAX_OGN_DEVICES; ++i) {
        if (ogn_tracking_store[i].common.timestamp != 0 &&
            ogn_common_match(&ogn_tracking_store[i].common, &newData->common)) {
            ogn_tracking_store[i] = *newData;
            return i;
        }
    }

    // find free slot
    for (int i = 0; i < MAX_OGN_DEVICES; ++i) {
        if (ogn_tracking_store[i].common.timestamp == 0) {
            ogn_tracking_store[i] = *newData;
            return i;
        }
    }

    // overwrite oldest
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
 * Debug print
 *----------------------------------------------------------------------------*/
void print_ogn_tracking(const ogn_tracking_data_t *d) {
    printf("=== OGN Tracking ===\n");
    printf("DevId: %s\n", d->common.devId);
    printf("Time: %ld\n", (long)d->common.timestamp);
    printf("Pos: %.6f, %.6f\n", d->common.lat, d->common.lon);
    printf("Alt GNSS: %.1f m, Press diff: %.1f m\n", d->alt_gnss, d->alt_pressure);
    printf("Speed: %.1f m/s, Heading: %.1f deg\n", d->speed, d->heading);
    printf("Climb: %.1f m/s, Turn: %d deg/s\n", d->climb, d->turn_rate);
    printf("Aircraft: %d, Emergency: %d\n", d->acft_type, d->emergency);
    printf("Fix: %s, DOP: %u\n", d->fix_mode ? "3D" : "2D", d->dop);
    printf("===================\n");
}

