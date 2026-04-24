// types.h
#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#define MAX_DEVICES 20

// Enumerations
typedef enum trck_state_ {
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

extern const char *const trck_state_names[16];

typedef enum trck_acft_type_ {
    acft_Other = 0,
    acft_Paraglider = 1,
    acft_Hangglider = 2,
    acft_Balloon = 3,
    acft_Glider = 4,
    acft_Powered_Aircraft = 5,
    acft_Helicopter = 6,
    acft_UAV = 7
} trck_acft_type;

extern const char *const trck_acft_names[8];

// Packet type constants
enum {
    FANET_PCK_TYPE_TRACKING = 0x01,
    FANET_PCK_TYPE_NAME = 0x02,
    FANET_PCK_TYPE_WEATHER = 0x04,
    FANET_PCK_TYPE_GROUND_TRACKING = 0x07
};

// Common header for all FANET packets
typedef struct {
    unsigned int type : 6;
    unsigned int forward : 1;
    unsigned int ext_header : 1;
    unsigned int vendor : 8;
    unsigned int address : 16;
} __attribute__((packed)) fanet_header;

// Packet type 1 (tracking)
typedef struct __attribute__((packed)) {
    fanet_header header;
    unsigned int latitude_raw : 24;
    unsigned int longitude_raw : 24;
    unsigned int altitude : 11;
    unsigned int altitude_scale : 1;
    unsigned int aircraft_type : 3;
    unsigned int track_online : 1;
    unsigned int speed_value : 7;
    unsigned int speed_scale : 1;
    unsigned int climb_value : 7;
    unsigned int climb_scale : 1;
    unsigned int heading : 8;
    unsigned int turn_value : 7;
    unsigned int turn_scale : 1;
    unsigned int qne_value : 7;
    unsigned int qne_scale : 1;
} fanet_packet_t1;

// Packet type 7 (ground tracking)
typedef struct __attribute__((packed)) {
    fanet_header header;
    unsigned int latitude_raw : 24;
    unsigned int longitude_raw : 24;
    unsigned int track_online : 1;
    unsigned int tbd : 3;
    trck_state type : 4;
} fanet_packet_t7;

// Packet type 4 (weather)
typedef struct __attribute__((packed)) {
    fanet_header header;
    unsigned int bExt_header2 : 1;
    unsigned int bStateOfCharge : 1;
    unsigned int bRemoteConfig : 1;
    unsigned int bBaro : 1;
    unsigned int bHumidity : 1;
    unsigned int bWind : 1;
    unsigned int bTemp : 1;
    unsigned int bInternetGateway : 1;
    unsigned int latitude : 24;
    unsigned int longitude : 24;
    int8_t temp;
    unsigned int heading : 8;
    unsigned int speed : 7;
    unsigned int speed_scale : 1;
    unsigned int gust : 7;
    unsigned int gust_scale : 1;
    unsigned int humidity : 8;
    int16_t baro;
    unsigned int charge : 8;
} __attribute__((packed)) fanet_packet_t4;

// Common fields for all device data
struct FanetCommon {
    char devId[7];          // "VVVVVV" (vendor + id)
    uint8_t vid;
    uint16_t fanet_id;
    int rssi;
    float snr;
    time_t timestamp;
    uint32_t last_send;     // not used in decoder, kept for compatibility
    float lat;
    float lon;
};

// Weather data structure
typedef struct {
    struct FanetCommon common;
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
} weatherData;

// Tracking data structure
typedef struct {
    struct FanetCommon common;
    float alt;
    uint16_t hdop;
    trck_acft_type aircraftType;
    char *adressType;       // points to static string "FNT"
    float speed;
    float climb;
    float heading;
    bool onlineTracking;
    trck_state state;
} trackingData;

// Global storage arrays
extern weatherData weatherStore[MAX_DEVICES];
extern trackingData trackingStore[MAX_DEVICES];

// Function prototypes
void FANET2String(uint8_t vid, uint16_t fid, char *out);

bool unpack_weatherdata(uint8_t *buffer, weatherData *wData, float snr, float rssi);
bool unpack_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr);
bool unpack_ground_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr);

int storeWeatherData(const weatherData *newData);
int storeTrackingData(const trackingData *newData);

// Helper functions for common operations
bool fanet_common_match(const struct FanetCommon *a, const struct FanetCommon *b);
void fanet_common_assign(struct FanetCommon *dest, const struct FanetCommon *src);

// Optional debug print (uses printf)
void print_fanet_packet_t4(const fanet_packet_t4 *pkt);
void print_weatherData(const weatherData *wData);

#endif // TYPES_H