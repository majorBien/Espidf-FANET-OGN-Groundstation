// types.c
#include "types.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// Constant strings for enums
const char *const trck_state_names[16] = {
    "Other", "Walking", "Vehicle", "Bike", "Boot",
    "Need a ride", "Landed well", "Need technical support",
    "Need medical help", "Distress call", "Distress call automatically",
    "Flying"
};

const char *const trck_acft_names[8] = {
    "Other", "Paraglider", "Hangglider", "Balloon",
    "Glider", "Powered Aircraft", "Helicopter", "UAV"
};

// Global storage
weatherData weatherStore[MAX_DEVICES];
trackingData trackingStore[MAX_DEVICES];

// Convert vendor and ID to string "VVVVVV"
void FANET2String(uint8_t vid, uint16_t fid, char *out) {
    sprintf(out, "%02X%04X", vid, fid);
}

// Common match function
bool fanet_common_match(const struct FanetCommon *a, const struct FanetCommon *b) {
    return a->vid == b->vid && a->fanet_id == b->fanet_id;
}

// Common assign (copies all fields except last_send)
void fanet_common_assign(struct FanetCommon *dest, const struct FanetCommon *src) {
    strcpy(dest->devId, src->devId);
    dest->vid = src->vid;
    dest->fanet_id = src->fanet_id;
    dest->rssi = src->rssi;
    dest->snr = src->snr;
    dest->timestamp = src->timestamp;
    // dest->last_send unchanged (intentionally not copied)
    dest->lat = src->lat;
    dest->lon = src->lon;
}

// Weather data unpack
bool unpack_weatherdata(uint8_t *buffer, weatherData *wData, float snr, float rssi) {
    fanet_packet_t4 *pkt = (fanet_packet_t4 *)buffer;
    wData->common.snr = snr;
    wData->common.rssi = rssi;

    wData->common.vid = pkt->header.vendor;
    wData->common.fanet_id = pkt->header.address;
    FANET2String(wData->common.vid, wData->common.fanet_id, wData->common.devId);

    wData->bStateOfCharge = pkt->bStateOfCharge;
    wData->bBaro = pkt->bBaro;
    wData->bHumidity = pkt->bHumidity;
    wData->bWind = pkt->bWind;
    wData->bTemp = pkt->bTemp;

    wData->common.timestamp = time(NULL);

    // Latitude
    int32_t lat_raw = pkt->latitude & 0xFFFFFF;
    if (lat_raw & 0x800000)
        lat_raw |= 0xFF000000;
    wData->common.lat = (float)lat_raw / 93206.0f;

    // Longitude
    int32_t lon_raw = pkt->longitude & 0xFFFFFF;
    if (lon_raw & 0x800000)
        lon_raw |= 0xFF000000;
    wData->common.lon = (float)lon_raw / 46603.0f;

    // Temperature
    if (wData->bTemp) {
        int8_t iTemp = pkt->temp;
        wData->temp = iTemp / 2.0f;
    }

    // Wind
    if (wData->bWind) {
        wData->wHeading = (pkt->heading * 360.0f) / 256.0f;
        int speed;
        if (pkt->speed_scale)
            speed = pkt->speed * 5;
        else
            speed = pkt->speed;
        wData->wSpeed = speed / 5.0f;

        if (pkt->gust_scale)
            speed = pkt->gust * 5;
        else
            speed = pkt->gust;
        wData->wGust = speed / 5.0f;
    }

    // Humidity
    if (wData->bHumidity) {
        wData->Humidity = pkt->humidity * 4.0f / 10.0f;
    }

    // Barometric pressure
    if (wData->bBaro) {
        wData->Baro = (pkt->baro / 10.0f) + 430.0f;
    }

    // State of charge
    if (wData->bStateOfCharge) {
        wData->Charge = (pkt->charge & 0x0F) * 100.0f / 15.0f;
    }

    // Rain fields not present in packet type 4, set false
    wData->bRain = false;
    wData->rain1h = 0;
    wData->rain24h = 0;

    return true;
}

// Tracking data unpack (type 1)
bool unpack_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr) {
    fanet_packet_t1 *packet = (fanet_packet_t1 *)buffer;

    data->common.vid = packet->header.vendor;
    data->common.fanet_id = packet->header.address;
    FANET2String(data->common.vid, data->common.fanet_id, data->common.devId);
    data->common.rssi = rssi;
    data->common.snr = snr;
    data->adressType = "FNT";
    data->common.timestamp = time(NULL);

    // Latitude
    int32_t lat_raw = packet->latitude_raw & 0xFFFFFF;
    if (lat_raw & 0x800000)
        lat_raw |= 0xFF000000;
    data->common.lat = (float)lat_raw / 93206.0f;

    // Longitude
    int32_t lon_raw = packet->longitude_raw & 0xFFFFFF;
    if (lon_raw & 0x800000)
        lon_raw |= 0xFF000000;
    data->common.lon = (float)lon_raw / 46603.0f;

    // Altitude
    float altitude = (float)packet->altitude * (packet->altitude_scale ? 4.0f : 1.0f);
    data->alt = altitude;

    // HDOP not transmitted
    data->hdop = 0;

    // Aircraft type
    data->aircraftType = (trck_acft_type)packet->aircraft_type;

    // Speed
    float speed = (float)packet->speed_value * 0.5f * (packet->speed_scale ? 5.0f : 1.0f);
    data->speed = speed;
    if (data->speed > 315) return false;

    // Climb rate
    int8_t climb_raw = packet->climb_value;
    if (climb_raw & 0x40) climb_raw |= 0x80;  // sign extend 7-bit
    float climb = (float)climb_raw * 0.1f * (packet->climb_scale ? 5.0f : 1.0f);
    data->climb = climb;

    // Heading
    data->heading = (float)packet->heading * (360.0f / 256.0f);

    // Online tracking flag
    data->onlineTracking = packet->track_online;
    data->state = state_Flying;

    // Optional fields omitted (turn rate, QNE) as they are not always present

    return true;
}

// Ground tracking data unpack (type 7)
bool unpack_ground_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr) {
    fanet_packet_t7 *packet = (fanet_packet_t7 *)buffer;

    data->common.vid = packet->header.vendor;
    data->common.fanet_id = packet->header.address;
    FANET2String(data->common.vid, data->common.fanet_id, data->common.devId);
    data->common.rssi = rssi;
    data->common.snr = snr;
    data->adressType = "FNT";
    data->common.timestamp = time(NULL);

    // Latitude
    int32_t lat_raw = packet->latitude_raw & 0xFFFFFF;
    if (lat_raw & 0x800000)
        lat_raw |= 0xFF000000;
    data->common.lat = (float)lat_raw / 93206.0f;

    // Longitude
    int32_t lon_raw = packet->longitude_raw & 0xFFFFFF;
    if (lon_raw & 0x800000)
        lon_raw |= 0xFF000000;
    data->common.lon = (float)lon_raw / 46603.0f;

    data->state = packet->type;
    data->climb = 0;
    data->heading = 0;
    data->aircraftType = acft_Other;
    data->speed = 0;
    data->alt = 0;

    return true;
}

// Store weather data - returns index in store
int storeWeatherData(const weatherData *newData) {
    // Check for existing match
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (weatherStore[i].common.timestamp != 0 &&
            fanet_common_match(&weatherStore[i].common, &newData->common)) {
            // Preserve name? Not used in this decoder, just assign
            weatherStore[i] = *newData;
            return i;
        }
    }

    // Find free slot
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (weatherStore[i].common.timestamp == 0) {
            weatherStore[i] = *newData;
            return i;
        }
    }

    // Overwrite oldest
    int oldestIndex = 0;
    time_t oldest = weatherStore[0].common.timestamp;
    for (int i = 1; i < MAX_DEVICES; ++i) {
        if (weatherStore[i].common.timestamp < oldest) {
            oldest = weatherStore[i].common.timestamp;
            oldestIndex = i;
        }
    }
    weatherStore[oldestIndex] = *newData;
    return oldestIndex;
}

// Store tracking data - returns index in store
int storeTrackingData(const trackingData *newData) {
    // Check for existing match
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (trackingStore[i].common.timestamp != 0 &&
            fanet_common_match(&trackingStore[i].common, &newData->common)) {
            trackingStore[i] = *newData;
            return i;
        }
    }

    // Find free slot
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (trackingStore[i].common.timestamp == 0) {
            trackingStore[i] = *newData;
            return i;
        }
    }

    // Overwrite oldest
    int oldestIndex = 0;
    time_t oldest = trackingStore[0].common.timestamp;
    for (int i = 1; i < MAX_DEVICES; ++i) {
        if (trackingStore[i].common.timestamp < oldest) {
            oldest = trackingStore[i].common.timestamp;
            oldestIndex = i;
        }
    }
    trackingStore[oldestIndex] = *newData;
    return oldestIndex;
}

// Debug print functions (using printf)
void print_fanet_packet_t4(const fanet_packet_t4 *pkt) {
    printf("=== FANET Packet T4 ===\n");
    printf("Header:\n");
    printf("  Type: %u\n", pkt->header.type);
    printf("  Forward: %u\n", pkt->header.forward);
    printf("  Ext Header: %u\n", pkt->header.ext_header);
    printf("  Vendor: %u\n", pkt->header.vendor);
    printf("  Address: %u\n", pkt->header.address);
    printf("Flags:\n");
    printf("  bExt_header2: %u\n", pkt->bExt_header2);
    printf("  bStateOfCharge: %u\n", pkt->bStateOfCharge);
    printf("  bRemoteConfig: %u\n", pkt->bRemoteConfig);
    printf("  bBaro: %u\n", pkt->bBaro);
    printf("  bHumidity: %u\n", pkt->bHumidity);
    printf("  bWind: %u\n", pkt->bWind);
    printf("  bTemp: %u\n", pkt->bTemp);
    printf("  bInternetGateway: %u\n", pkt->bInternetGateway);
    printf("Data:\n");
    printf("  Latitude (raw): %ld\n", (long)pkt->latitude);
    printf("  Longitude (raw): %ld\n", (long)pkt->longitude);
    printf("  Temp: %d\n", pkt->temp);
    printf("  Heading: %u\n", pkt->heading);
    printf("  Speed: %u\n", pkt->speed);
    printf("  Speed Scale: %u\n", pkt->speed_scale);
    printf("  Gust: %u\n", pkt->gust);
    printf("  Gust Scale: %u\n", pkt->gust_scale);
    printf("  Humidity: %u\n", pkt->humidity);
    printf("  Baro: %d\n", pkt->baro);
    printf("  Charge: %u\n", pkt->charge);
    printf("=======================\n\n");
}

void print_weatherData(const weatherData *wData) {
    printf("=== WeatherData ===\n");
    printf("Timestamp: %ld\n", (long)wData->common.timestamp);
    printf("DevID: %s\n", wData->common.devId);
    printf("VID: %02X\n", wData->common.vid);
    printf("Fanet ID: %04X\n", wData->common.fanet_id);
    printf("RSSI: %d dBm\n", wData->common.rssi);
    printf("SNR: %.1f dB\n", wData->common.snr);
    printf("Latitude: %.6f\n", wData->common.lat);
    printf("Longitude: %.6f\n", wData->common.lon);
    printf("Measurements:\n");
    if (wData->bTemp)
        printf("  Temperature: %.1f C\n", wData->temp);
    else
        printf("  Temperature: N/A\n");
    if (wData->bWind) {
        printf("  Wind Heading: %.1f deg\n", wData->wHeading);
        printf("  Wind Speed: %.1f km/h\n", wData->wSpeed);
        printf("  Wind Gust: %.1f km/h\n", wData->wGust);
    } else {
        printf("  Wind: N/A\n");
    }
    if (wData->bHumidity)
        printf("  Humidity: %.1f %%RH\n", wData->Humidity);
    else
        printf("  Humidity: N/A\n");
    if (wData->bBaro)
        printf("  Barometric Pressure: %.1f hPa\n", wData->Baro);
    else
        printf("  Barometric Pressure: N/A\n");
    if (wData->bStateOfCharge)
        printf("  State of Charge: %.1f %%\n", wData->Charge);
    else
        printf("  State of Charge: N/A\n");
    printf("=====================\n\n");
}