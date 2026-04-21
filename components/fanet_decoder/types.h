/*
 * types.h
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * CONFIG / CONSTANTS
 * ========================= */

#define MAX_DEVICES 20

/* FANET packet types */
#define FANET_PCK_TYPE_TRACKING        0x01
#define FANET_PCK_TYPE_NAME            0x02
#define FANET_PCK_TYPE_WEATHER         0x04
#define FANET_PCK_TYPE_GROUND_TRACKING 0x07

/* =========================
 * ENUMS
 * ========================= */

typedef enum {
    state_Other = 0,
    state_Walking = 1,
    state_Vehicle = 2,
    state_Bike = 3,
    state_Boot = 4,
    state_Need_ride = 8,
    state_Landed_well = 9,
    state_Need_technical_support = 12,
    state_Need_medical_help = 13,
    state_Distress_call = 14,
    state_Distress_call_automatically = 15,
    state_Flying = 16
} trck_state;

typedef enum {
    acft_Other = 0,
    acft_Paraglider = 1,
    acft_Hangglider = 2,
    acft_Balloon = 3,
    acft_Glider = 4,
    acft_Powered_Aircraft = 5,
    acft_Helicopter = 6,
    acft_UAV = 7
} trck_acft_type;

/* =========================
 * FANET HEADER (PHY LEVEL)
 * ========================= */

typedef struct __attribute__((packed)) {
    unsigned int type       : 6;
    unsigned int forward    : 1;
    unsigned int ext_header : 1;

    unsigned int vendor     : 8;
    unsigned int address    : 16;
} fanet_header;


typedef struct __attribute__((packed)) {
    fanet_header header;
    // Bytes 0–2 : Latitude (24-bit signed, little endian)
    unsigned int latitude_raw : 24;

    // Bytes 3–5 : Longitude (24-bit signed, little endian)
    unsigned int longitude_raw : 24;

    // Bytes 6–7 : Type + altitude field
    unsigned int altitude       : 11;  // bits 0–10 (meters)
    unsigned int altitude_scale : 1;   // bit 11 (0=×1, 1=×4)
    unsigned int aircraft_type  : 3;   // bits 12–14 (0..7)
    unsigned int track_online   : 1;   // bit 15

    // Byte 8 : Speed
    unsigned int speed_value : 7;  // bits 0–6 (×0.5 km/h)
    unsigned int speed_scale : 1;  // bit 7 (×5)
    
    // Byte 9 : Climb rate
    unsigned int climb_value : 7;  // bits 0–6 (×0.1 m/s, 2’s complement)
    unsigned int climb_scale : 1;  // bit 7 (×5)
    

    // Byte 10 : Heading (0–255 → 0–360°)
    unsigned int heading : 8;

    // Byte 11 : (optional) Turn rate
    unsigned int turn_value : 7;  // bits 0–6 (×0.25°/s, 2’s complement)
    unsigned int turn_scale : 1;  // bit 7 (×4)
    
    // Byte 12 : (optional) QNE offset
    unsigned int qne_value : 7;   // bits 0–6 (meters, 2’s complement)
    unsigned int qne_scale : 1;   // bit 7 (×4)
    
    
} fanet_packet_t1;



typedef struct __attribute__((packed)) {
    fanet_header header;
    // Bytes 0–2 : Latitude (24-bit signed, little endian)
    unsigned int latitude_raw : 24;

    // Bytes 3–5 : Longitude (24-bit signed, little endian)
    unsigned int longitude_raw : 24;

    // Bytes 6 : Type
    unsigned int track_online   : 1;   // bit 0
    unsigned int tbd  : 3;             // bits 3-1
    trck_state type : 4;             // bits 7-4

} fanet_packet_t7;


typedef struct {
    fanet_header header;
    unsigned int bExt_header2     :1;
    unsigned int bStateOfCharge   :1;
    unsigned int bRemoteConfig    :1;
    unsigned int bBaro            :1;
    unsigned int bHumidity        :1;
    unsigned int bWind            :1;
    unsigned int bTemp            :1;
    unsigned int bInternetGateway :1;
    unsigned int latitude       :24;
    unsigned int longitude      :24;
    int8_t temp                 :8;
    unsigned int heading        :8;
    unsigned int speed          :7;
    unsigned int speed_scale    :1;
    unsigned int gust          :7;
    unsigned int gust_scale    :1;
    unsigned int humidity      :8;
    int baro          :16;
    unsigned int charge        :8;
  } __attribute__((packed)) fanet_packet_t4;

/* =========================
 * WEATHER DATA
 * ========================= */

typedef struct {
    char devId[16];   // replaced String
    char name[32];

    uint8_t vid;
    uint16_t fanet_id;

    int rssi;
    float snr;

    time_t timestamp;
    uint32_t last_send;

    float lat;
    float lon;

    bool bTemp;
    float temp;

    float wHeading;
    bool bWind;
    float wSpeed;
    float wGust;

    bool bHumidity;
    float Humidity;

    bool bBaro;
    float Baro;

    bool bStateOfCharge;
    float Charge;

    bool bRain;
    float rain1h;
    float rain24h;

    bool bInternetGateway;

} weatherData;

/* =========================
 * TRACKING DATA
 * ========================= */

typedef struct {
    char devId[16];

    uint8_t vid;
    uint16_t fanet_id;

    int rssi;
    float snr;

    time_t timestamp;
    uint32_t last_send;

    float lat;
    float lon;

    float alt;
    uint16_t hdop;

    trck_acft_type aircraftType;

    float speed;
    float climb;
    float heading;

    bool onlineTracking;
    trck_state state;

    char adressType[8];

} trackingData;



/* =========================
 * FUNCTIONS
 * ========================= */

/* FANET utils */
const char *FANET2String(uint8_t vid, uint16_t fid);

/* PACK / UNPACK */
void pack_weatherdata(weatherData *wData, uint8_t *buffer);

bool unpack_weatherdata(uint8_t *buffer, weatherData *wData, float snr, float rssi);
bool unpack_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr);
bool unpack_ground_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr);

/* STORAGE */
int storeWeatherData(const weatherData *newData);
int storeTrackingData(const trackingData *newData);

/* DEBUG */
void print_weatherData(const weatherData *wData);

#ifdef __cplusplus
}
#endif