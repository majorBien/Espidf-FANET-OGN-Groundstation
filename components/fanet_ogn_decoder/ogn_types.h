/**
 * @file ogn_types.h
 * @brief OGN protocol data types and structures (compatible with original C++ code)
 */

#ifndef OGN_TYPES_H
#define OGN_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Enumerations
 *============================================================================*/

/** Address type (bits 0-1 of header byte 4) */
typedef enum {
    OGN_ADDR_OTHER = 0,
    OGN_ADDR_ICAO  = 1,
    OGN_ADDR_RSV   = 2,
    OGN_ADDR_OGN   = 3
} ogn_addr_type_t;

/** Aircraft type (4 bits) - matches OGN1_Packet::Position.AcftType */
typedef enum {
    OGN_ACFT_GLIDER     = 0,
    OGN_ACFT_TOW        = 1,
    OGN_ACFT_HELI       = 2,
    OGN_ACFT_PARAGLIDER = 3,
    OGN_ACFT_HANGGLIDER = 4,
    OGN_ACFT_BALLOON    = 5,
    OGN_ACFT_UAV        = 6,
    OGN_ACFT_OTHER      = 7
} ogn_aircraft_type_t;

/** GPS fix quality (2 bits) */
typedef enum {
    OGN_FIX_NONE = 0,
    OGN_FIX_GPS  = 1,
    OGN_FIX_DGPS = 2,
    OGN_FIX_OTHER = 3
} ogn_fix_quality_t;

/*==============================================================================
 * Structures - using integer types with fixed scaling like original C++
 *============================================================================*/

/** Common data for every OGN device */
typedef struct {
    char devId[7];            /**< "VVVVVV" string (VID+address) */
    uint8_t vid;              /**< Vendor ID */
    uint32_t address;         /**< 24-bit device address */
    int rssi;                 /**< RSSI [dBm] */
    float snr;                /**< SNR [dB] */
    time_t timestamp;         /**< Reception timestamp */
    double lat;               /**< Latitude [deg] (decoded as double for convenience) */
    double lon;               /**< Longitude [deg] */
} ogn_common_t;

/** Tracking (position) packet data - scaled as in OGN1_Packet decode methods */
typedef struct {
    ogn_common_t common;               /**< Common fields */

    ogn_addr_type_t addr_type;         /**< Address type */
    bool addr_parity;                  /**< Address parity bit */
    bool emergency;                    /**< Emergency flag */
    uint8_t relay_cnt;                 /**< Relay count (0-3) */
    bool other_data;                   /**< Non-position packet */
    bool custom_enc;                   /**< Encrypted/private data */
    ogn_aircraft_type_t acft_type;     /**< Aircraft type */
    bool stealth;                      /**< Stealth mode */
    uint8_t time_sec;                  /**< UTC seconds (0-59, 0x3F = invalid) */

    /* These fields use the same scaling as original C++ decode functions:
     * - altitude: meters (0..61432) but original uses decimeters? Let's keep meters for simplicity.
     *   Actually original OGN1_Packet::DecodeAltitude() returns meters (as int).
     *   We'll keep double for now but can change to int if needed.
     * - alt_pressure: pressure altitude difference [m] (signed)
     * - climb: [0.1 m/s] -> stored as int16_t
     * - speed: [0.1 m/s] -> stored as uint16_t
     * - heading: [0.1 deg] -> stored as uint16_t
     * - turn_rate: [0.1 deg/s] -> stored as int16_t
     */
    double alt_gnss;                   /**< GNSS altitude [m] (above geoid) */
    double alt_pressure;               /**< Pressure altitude difference [m] */
    int16_t climb;                     /**< Climb rate [0.1 m/s] */
    uint16_t speed;                    /**< Ground speed [0.1 m/s] */
    uint16_t heading;                  /**< Track heading [0.1 deg] */
    int16_t turn_rate;                 /**< Turn rate [0.1 deg/s] */

    bool fix_mode;                     /**< 0=2D, 1=3D */
    ogn_fix_quality_t fix_quality;     /**< GPS fix quality */
    uint8_t dop;                       /**< Dilution of precision (0-63) */
} ogn_tracking_data_t;

/*==============================================================================
 * Global storage
 *============================================================================*/

#define MAX_OGN_DEVICES 50

extern ogn_tracking_data_t ogn_tracking_store[MAX_OGN_DEVICES];

/*==============================================================================
 * Function prototypes
 *============================================================================*/

/** Convert vendor ID + address to "VVVVVV" string */
void ogn_addr_to_string(uint8_t vid, uint32_t addr, char *out);

/** Compare two common structures (same device?) */
bool ogn_common_match(const ogn_common_t *a, const ogn_common_t *b);

/** Copy common fields (except timestamp) */
void ogn_common_assign(ogn_common_t *dest, const ogn_common_t *src);

/** Unpack raw OGN packet into tracking data (scaled as per original) */
bool unpack_ogn_tracking(const uint8_t *buffer, int len, ogn_tracking_data_t *data,
                         int rssi, int snr);

/** Store tracking data in global array, return index */
int store_ogn_tracking_data(const ogn_tracking_data_t *data);

/** Debug print of tracking data */
void print_ogn_tracking(const ogn_tracking_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* OGN_TYPES_H */