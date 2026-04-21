/*
 * types.c
 *
 *  Created on: 21 kwi 2026
 *      Author: majorBien
 */
 
 #include "types.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/* =========================
 * STORAGE
 * ========================= */

weatherData weatherStore[MAX_DEVICES];
trackingData trackingStore[MAX_DEVICES];

/* =========================
 * INTERNAL HELPERS
 * ========================= */

static int storeFanetDataWeather(weatherData *store, const weatherData *newData){
    int oldestIndex = 0;
    time_t oldest = store[0].timestamp;

    /* 1. match */
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (store[i].timestamp != 0 &&
            store[i].vid == newData->vid &&
            store[i].fanet_id == newData->fanet_id)
        {
            /* preserve name */
            char tmpName[32];
            memcpy(tmpName, store[i].name, sizeof(tmpName));

            memcpy(&store[i], newData, sizeof(weatherData));

            memcpy(store[i].name, tmpName, sizeof(tmpName));
            return i;
        }
    }

    /* 2. free slot */
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (store[i].timestamp == 0) {
            memcpy(&store[i], newData, sizeof(weatherData));
            return i;
        }
    }

    /* 3. overwrite oldest */
    for (int i = 1; i < MAX_DEVICES; i++) {
        if (store[i].timestamp < oldest) {
            oldest = store[i].timestamp;
            oldestIndex = i;
        }
    }

    memcpy(&store[oldestIndex], newData, sizeof(weatherData));
    return oldestIndex;
}

static int storeFanetDataTracking(trackingData *store, const trackingData *newData){
    int oldestIndex = 0;
    time_t oldest = store[0].timestamp;

    for (int i = 0; i < MAX_DEVICES; i++) {
        if (store[i].timestamp != 0 &&
            store[i].vid == newData->vid &&
            store[i].fanet_id == newData->fanet_id)
        {
            char tmpName[16];
            memcpy(tmpName, store[i].devId, sizeof(tmpName));

            memcpy(&store[i], newData, sizeof(trackingData));

            memcpy(store[i].devId, tmpName, sizeof(tmpName));
            return i;
        }
    }

    for (int i = 0; i < MAX_DEVICES; i++) {
        if (store[i].timestamp == 0) {
            memcpy(&store[i], newData, sizeof(trackingData));
            return i;
        }
    }

    for (int i = 1; i < MAX_DEVICES; i++) {
        if (store[i].timestamp < oldest) {
            oldest = store[i].timestamp;
            oldestIndex = i;
        }
    }

    memcpy(&store[oldestIndex], newData, sizeof(trackingData));
    return oldestIndex;
}

/* =========================
 * API
 * ========================= */

int storeWeatherData(const weatherData *newData){
    return storeFanetDataWeather(weatherStore, newData);
}

int storeTrackingData(const trackingData *newData){
    return storeFanetDataTracking(trackingStore, newData);
}

/* =========================
 * FANET STRING
 * ========================= */

const char *FANET2String(uint8_t vid, uint16_t fid){
    static char buf[16];
    snprintf(buf, sizeof(buf), "%02X%04X", vid, fid);
    return buf;
}

/* =========================
 * PACK WEATHER
 * ========================= */

void pack_weatherdata(weatherData *wData, uint8_t *buffer){
    fanet_packet_t4 *pkt = (fanet_packet_t4 *)buffer;

    pkt->header.type = 4;
    pkt->header.vendor = wData->vid;
    pkt->header.forward = 0;
    pkt->header.ext_header = 0;
    pkt->header.address = wData->fanet_id;

    pkt->bTemp = wData->bTemp;
    pkt->bWind = wData->bWind;
    pkt->bHumidity = wData->bHumidity;
    pkt->bBaro = wData->bBaro;
    pkt->bStateOfCharge = wData->bStateOfCharge;
    pkt->bInternetGateway = 0;

    int32_t lat_i = (int32_t)roundf(wData->lat * 93206.0f);
    int32_t lon_i = (int32_t)roundf(wData->lon * 46603.0f);

    pkt->latitude = lat_i;
    pkt->longitude = lon_i;

    if (wData->bTemp) {
        int8_t t = (int8_t)roundf(wData->temp * 2.0f);
        pkt->temp = t;
    }

    if (wData->bWind) {
        pkt->heading = (uint8_t)(wData->wHeading * 256.0f / 360.0f);

        int speed = (int)roundf(wData->wSpeed * 5.0f);
        pkt->speed = (speed > 127) ? (speed / 5) : (speed & 0x7F);
        pkt->speed_scale = (speed > 127);

        int gust = (int)roundf(wData->wGust * 5.0f);
        pkt->gust = (gust > 127) ? (gust / 5) : (gust & 0x7F);
        pkt->gust_scale = (gust > 127);
    }

    if (wData->bHumidity) {
        pkt->humidity = (uint8_t)roundf(wData->Humidity * 10.0f / 4.0f);
    }

    if (wData->bBaro) {
        pkt->baro = (int16_t)roundf((wData->Baro - 430.0f) * 10.0f);
    }

    pkt->charge = (uint8_t)fminf(fmaxf(roundf(wData->Charge / 100.0f * 15.0f), 0), 15);
}

/* =========================
 * UNPACK WEATHER
 * ========================= */

bool unpack_weatherdata(uint8_t *buffer, weatherData *wData, float snr, float rssi){
    fanet_packet_t4 *pkt = (fanet_packet_t4 *)buffer;

    memset(wData, 0, sizeof(weatherData));

    wData->vid = pkt->header.vendor;
    wData->fanet_id = pkt->header.address;

    wData->snr = snr;
    wData->rssi = (int)rssi;

    wData->timestamp = time(NULL);

    int32_t lat = pkt->latitude;
    if (lat & 0x800000) lat |= 0xFF000000;
    wData->lat = (float)lat / 93206.0f;

    int32_t lon = pkt->longitude;
    if (lon & 0x800000) lon |= 0xFF000000;
    wData->lon = (float)lon / 46603.0f;

    if (pkt->bTemp)
        wData->temp = ((int8_t)pkt->temp) / 2.0f;

    if (pkt->bWind) {
        wData->wHeading = pkt->heading * 360.0f / 256.0f;

        int speed = pkt->speed_scale ? pkt->speed * 5 : pkt->speed;
        wData->wSpeed = speed / 5.0f;

        int gust = pkt->gust_scale ? pkt->gust * 5 : pkt->gust;
        wData->wGust = gust / 5.0f;
    }

    if (pkt->bHumidity)
        wData->Humidity = pkt->humidity * 4.0f / 10.0f;

    if (pkt->bBaro)
        wData->Baro = (pkt->baro / 10.0f) + 430.0f;

    if (pkt->bStateOfCharge)
        wData->Charge = pkt->charge * 100.0f / 15.0f;

    return true;
}

/* =========================
 * TRACKING (minimal stub)
 * ========================= */

bool unpack_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr){
    fanet_packet_t1 *pkt = (fanet_packet_t1 *)buffer;

    memset(data, 0, sizeof(trackingData));

    data->vid = pkt->header.vendor;
    data->fanet_id = pkt->header.address;
    data->rssi = rssi;
    data->snr = snr;
    data->timestamp = time(NULL);

    return true;
}

bool unpack_ground_trackingdata(uint8_t *buffer, trackingData *data, int rssi, int snr){
    fanet_packet_t7 *pkt = (fanet_packet_t7 *)buffer;

    memset(data, 0, sizeof(trackingData));

    data->vid = pkt->header.vendor;
    data->fanet_id = pkt->header.address;
    data->rssi = rssi;
    data->snr = snr;
    data->timestamp = time(NULL);

    return true;
}

/* =========================
 * DEBUG
 * ========================= */

void print_weatherData(const weatherData *w){
    printf("WEATHER: VID=%02X ID=%04X RSSI=%d SNR=%.1f LAT=%.5f LON=%.5f\n",
           w->vid, w->fanet_id, w->rssi, w->snr, w->lat, w->lon);
}




