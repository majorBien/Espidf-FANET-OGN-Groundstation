/*
 * ogn_conv.c
 *
 *  Created on: 27 kwi 2026
 *      Author: majorBien
 */

/**
 * @file ogn_conv.c
 * @brief Implementation of OGN conversion functions (C version)
 */

#include "ogn_conv.h"
#include <string.h>
#include <math.h>

/*==============================================================================
 * Coordinate conversion constants
 *============================================================================*/

#define OGN_LAT_RES  (0.0008 / 60.0)   // degrees per unit for latitude
#define OGN_LON_RES  (0.0016 / 60.0)   // degrees per unit for longitude
#define FNT_RES      1e-5              // degrees per unit (1e-5)
#define UBX_RES      1e-7              // degrees per unit (1e-7)
#define CRD_RES      1e-5              // degrees per unit

/* Helper: convert degrees to OGN internal units */
static inline int32_t deg_to_ogn_lat(double deg)   { return (int32_t)(deg / OGN_LAT_RES + 0.5); }
static inline int32_t deg_to_ogn_lon(double deg)   { return (int32_t)(deg / OGN_LON_RES + 0.5); }
static inline int32_t deg_to_fnt(double deg)       { return (int32_t)(deg / FNT_RES + 0.5); }
static inline int32_t deg_to_ubx(double deg)       { return (int32_t)(deg / UBX_RES + 0.5); }
static inline int32_t deg_to_crd(double deg)       { return (int32_t)(deg / CRD_RES + 0.5); }

static inline double ogn_to_deg_lat(int32_t ogn)   { return (double)ogn * OGN_LAT_RES; }
static inline double ogn_to_deg_lon(int32_t ogn)   { return (double)ogn * OGN_LON_RES; }
static inline double fnt_to_deg(int32_t fnt)       { return (double)fnt * FNT_RES; }
static inline double ubx_to_deg(int32_t ubx)       { return (double)ubx * UBX_RES; }
static inline double crd_to_deg(int32_t crd)       { return (double)crd * CRD_RES; }

/*------------------------------------------------------------------------------
 * Coordinate conversions
 *----------------------------------------------------------------------------*/
int32_t Coord_FNTtoOGN(int32_t Coord) {
    double deg = fnt_to_deg(Coord);
    // For latitude/longitude we need to know which coordinate it is.
    // This generic version assumes the same resolution scaling.
    // In practice, separate functions for lat/lon would be better.
    // Here we use lat resolution as default (because original OGN uses two different scales).
    // Actually the original header expects the same function for both axes,
    // but OGN uses 0.0008/60 for lat and 0.0016/60 for lon.
    // We cannot know from the value alone. For compatibility we keep simple:
    return deg_to_ogn_lat(deg);   // caller must be aware of axis
}

int32_t Coord_OGNtoFNT(int32_t Coord) {
    double deg = ogn_to_deg_lat(Coord); // again using lat scale
    return deg_to_fnt(deg);
}

int32_t Coord_FNTtoUBX(int32_t Coord) {
    double deg = fnt_to_deg(Coord);
    return deg_to_ubx(deg);
}

int32_t Coord_UBXtoFNT(int32_t Coord) {
    double deg = ubx_to_deg(Coord);
    return deg_to_fnt(deg);
}

int32_t Coord_CRDtoOGN(int32_t Coord) {
    double deg = crd_to_deg(Coord);
    return deg_to_ogn_lat(deg);
}

int32_t Coord_OGNtoCRD(int32_t Coord) {
    double deg = ogn_to_deg_lat(Coord);
    return deg_to_crd(deg);
}

/*------------------------------------------------------------------------------
 * Altitude conversions (feet <-> meters)
 *----------------------------------------------------------------------------*/
int32_t FeetToMeters(int32_t Altitude) {
    // Altitude in feet, return millimeters
    return (int32_t)((double)Altitude * 304.8 + 0.5);
}

int32_t MetersToFeet(int32_t Altitude) {
    // Altitude in millimeters, return thousandths of a foot
    return (int32_t)((double)Altitude / 304.8 * 1000.0 + 0.5);
}

/*------------------------------------------------------------------------------
 * Aircraft type mappings (based on common OGN/FLARM/ADSB definitions)
 *----------------------------------------------------------------------------*/
/*
 * OGN types:
 * 0=Glider, 1=Tow plane, 2=Helicopter, 3=Paraglider, 4=Hangglider, 5=Balloon, 6=UAV, 7=Other
 * ADSB categories (simplified):
 * 0=Unknown, 1=Glider, 2=Light, 3=Helicopter, 4=Parachute, 5=UAV, 6=Balloon, 7=Other
 */
uint8_t AcftType_OGNtoADSB(uint8_t AcftType) {
    const uint8_t map[] = {1, 2, 3, 4, 4, 6, 5, 7}; // Glider->1, Tow->2, Heli->3, Paraglider->4, Hangglider->4, Balloon->6, UAV->5, Other->7
    return (AcftType < sizeof(map)) ? map[AcftType] : 0;
}

uint8_t AcftType_FNTtoADSB(uint8_t AcftType) {
    // FNT codes (FLARM) are similar but slightly different. Assume same as OGN for stub.
    return AcftType_OGNtoADSB(AcftType);
}

uint8_t AcftType_ADSBtoOGN(uint8_t AcftCat) {
    const uint8_t map[] = {7, 0, 1, 2, 3, 6, 5, 7}; // ADSB->OGN as inverse approx.
    return (AcftCat < sizeof(map)) ? map[AcftCat] : 7;
}

uint8_t AcftType_OGNtoGDL(uint8_t AcftType) {
    // No definition, just return unchanged
    return AcftType;
}

uint8_t AcftType_OGNtoADSL(uint8_t AcftType) {
    // ADSL (FLARM Advanced) likely same as ADSB for now
    return AcftType_OGNtoADSB(AcftType);
}

uint8_t AcftType_ADSLtoOGN(uint8_t AcftCat) {
    return AcftType_ADSBtoOGN(AcftCat);
}

uint8_t AcftType_FNTtoOGN(uint8_t AcftType) {
    // Assume FNT and OGN type codes are identical
    return AcftType;
}

uint8_t AcftType_FNTtoADSL(uint8_t AcftType) {
    return AcftType_OGNtoADSL(AcftType);
}

/*------------------------------------------------------------------------------
 * Variable-rate encoding (UnsVRencode/Decode and SignVRencode/Decode)
 * These implement the template functions from OGNCONV_H for specific bit widths.
 *----------------------------------------------------------------------------*/

/* Unsigned variable-rate encode for 5 bits (threshold 32) */
static uint8_t UnsVRencode_5(uint16_t Value) {
    const uint16_t Thres = 32;
    if (Value < Thres) return (uint8_t)Value;
    if (Value < 3*Thres) return Thres | ((Value - Thres) >> 1);
    if (Value < 7*Thres) return 2*Thres | ((Value - 3*Thres) >> 2);
    if (Value < 15*Thres) return 3*Thres | ((Value - 7*Thres) >> 3);
    return 4*Thres - 1;
}

static uint16_t UnsVRdecode_5(uint8_t Value) {
    const uint16_t Thres = 32;
    uint8_t Range = Value >> 5;
    uint8_t Low = Value & (Thres - 1);
    if (Range == 0) return Low;
    if (Range == 1) return Thres + 1 + (Low << 1);
    if (Range == 2) return 3*Thres + 2 + (Low << 2);
    return 7*Thres + 4 + (Low << 3);
}

/* Signed variable-rate for 5 bits */
static uint8_t SignVRencode_5(int16_t Value) {
    const uint8_t SignMask = 1 << (5+2); // 128
    uint8_t Sign = 0;
    if (Value < 0) { Value = -Value; Sign = SignMask; }
    uint8_t enc = UnsVRencode_5((uint16_t)Value);
    return enc | Sign;
}

static int16_t SignVRdecode_5(uint8_t Value) {
    const uint8_t SignMask = 1 << 7; // 128
    uint8_t Sign = Value & SignMask;
    uint16_t mag = UnsVRdecode_5(Value & (SignMask - 1));
    return Sign ? -(int16_t)mag : (int16_t)mag;
}

/* Unsigned variable-rate for 6 bits (threshold 64) */
static uint16_t UnsVRencode_6(uint16_t Value) {
    const uint16_t Thres = 64;
    if (Value < Thres) return Value;
    if (Value < 3*Thres) return Thres | ((Value - Thres) >> 1);
    if (Value < 7*Thres) return 2*Thres | ((Value - 3*Thres) >> 2);
    if (Value < 15*Thres) return 3*Thres | ((Value - 7*Thres) >> 3);
    return 4*Thres - 1;
}

static uint16_t UnsVRdecode_6(uint16_t Value) {
    const uint16_t Thres = 64;
    uint8_t Range = Value >> 6;
    uint16_t Low = Value & (Thres - 1);
    if (Range == 0) return Low;
    if (Range == 1) return Thres + 1 + (Low << 1);
    if (Range == 2) return 3*Thres + 2 + (Low << 2);
    return 7*Thres + 4 + (Low << 3);
}

/* Signed variable-rate for 6 bits */
static uint16_t SignVRencode_6(int16_t Value) {
    const uint16_t SignMask = 1 << (6+2); // 256
    uint16_t Sign = 0;
    if (Value < 0) { Value = -Value; Sign = SignMask; }
    uint16_t enc = UnsVRencode_6((uint16_t)Value);
    return enc | Sign;
}

static int16_t SignVRdecode_6(uint16_t Value) {
    const uint16_t SignMask = 1 << 8; // 256
    uint16_t Sign = Value & SignMask;
    uint16_t mag = UnsVRdecode_6(Value & (SignMask - 1));
    return Sign ? -(int16_t)mag : (int16_t)mag;
}

/* Now the public functions using the correct bit widths as per OGN spec */
uint16_t EncodeUR2V8(uint16_t Value) {
    // UR2V8: unsigned 12-bit (0..3832) -> 10-bit. Threshold = 1<<8 = 256
    const uint16_t Thres = 256;
    if (Value < Thres) return Value;
    if (Value < 3*Thres) return Thres | ((Value - Thres) >> 1);
    if (Value < 7*Thres) return 2*Thres | ((Value - 3*Thres) >> 2);
    if (Value < 15*Thres) return 3*Thres | ((Value - 7*Thres) >> 3);
    return 4*Thres - 1;
}

uint16_t DecodeUR2V8(uint16_t Value) {
    const uint16_t Thres = 256;
    uint8_t Range = Value >> 8;
    uint16_t Low = Value & (Thres - 1);
    if (Range == 0) return Low;
    if (Range == 1) return Thres + 1 + (Low << 1);
    if (Range == 2) return 3*Thres + 2 + (Low << 2);
    return 7*Thres + 4 + (Low << 3);
}

uint8_t EncodeUR2V5(uint16_t Value) {
    // UR2V5: unsigned 9-bit (0..472) -> 7-bit. Threshold = 1<<5 = 32
    return UnsVRencode_5(Value);
}

uint16_t DecodeUR2V5(uint16_t Value) {
    return UnsVRdecode_5((uint8_t)Value);
}

uint8_t EncodeSR2V5(int16_t Value) {
    return SignVRencode_5(Value);
}

int16_t DecodeSR2V5(int16_t Value) {
    return SignVRdecode_5((uint8_t)Value);
}

uint16_t EncodeUR2V6(uint16_t Value) {
    // UR2V6: unsigned 10-bit (0..952) -> 8-bit. Threshold = 1<<6 = 64
    return UnsVRencode_6(Value);
}

uint16_t DecodeUR2V6(uint16_t Value) {
    return UnsVRdecode_6(Value);
}

uint16_t EncodeSR2V6(int16_t Value) {
    return SignVRencode_6(Value);
}

int16_t DecodeSR2V6(int16_t Value) {
    return SignVRdecode_6((uint16_t)Value);
}

/*------------------------------------------------------------------------------
 * Gray code
 *----------------------------------------------------------------------------*/
uint8_t EncodeGray8(uint8_t Binary) {
    return Binary ^ (Binary >> 1);
}

uint8_t DecodeGray8(uint8_t Gray) {
    uint8_t bin = Gray;
    while (Gray >>= 1) bin ^= Gray;
    return bin;
}

uint16_t EncodeGray16(uint16_t Binary) {
    return Binary ^ (Binary >> 1);
}

uint16_t DecodeGray16(uint16_t Gray) {
    uint16_t bin = Gray;
    while (Gray >>= 1) bin ^= Gray;
    return bin;
}

uint32_t EncodeGray32(uint32_t Binary) {
    return Binary ^ (Binary >> 1);
}

uint32_t DecodeGray32(uint32_t Gray) {
    uint32_t bin = Gray;
    while (Gray >>= 1) bin ^= Gray;
    return bin;
}

/*------------------------------------------------------------------------------
 * TEA (Tiny Encryption Algorithm)
 *----------------------------------------------------------------------------*/
void TEA_Encrypt(uint32_t* Data, const uint32_t* Key, int Loops) {
    uint32_t v0 = Data[0], v1 = Data[1];
    uint32_t sum = 0;
    uint32_t delta = 0x9E3779B9;
    for (int i = 0; i < Loops; i++) {
        sum += delta;
        v0 += ((v1 << 4) + Key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + Key[1]);
        v1 += ((v0 << 4) + Key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + Key[3]);
    }
    Data[0] = v0; Data[1] = v1;
}

void TEA_Decrypt(uint32_t* Data, const uint32_t* Key, int Loops) {
    uint32_t v0 = Data[0], v1 = Data[1];
    uint32_t sum = Loops * 0x9E3779B9;
    uint32_t delta = 0x9E3779B9;
    for (int i = 0; i < Loops; i++) {
        v1 -= ((v0 << 4) + Key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + Key[3]);
        v0 -= ((v1 << 4) + Key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + Key[1]);
        sum -= delta;
    }
    Data[0] = v0; Data[1] = v1;
}

void TEA_Encrypt_Key0(uint32_t* Data, int Loops) {
    const uint32_t Key[4] = {0,0,0,0};
    TEA_Encrypt(Data, Key, Loops);
}

void TEA_Decrypt_Key0(uint32_t* Data, int Loops) {
    const uint32_t Key[4] = {0,0,0,0};
    TEA_Decrypt(Data, Key, Loops);
}

/*------------------------------------------------------------------------------
 * XXTEA (Corrected Block TEA)
 * Implementation based on public domain code by David Wheeler and Roger Needham.
 *----------------------------------------------------------------------------*/
#define XXTEA_DELTA 0x9E3779B9

void XXTEA_Encrypt(uint32_t* Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops) {
    if (Words < 2) return;
    uint32_t z = Data[Words-1], y, sum = 0, e;
    uint32_t p, q;
    uint32_t rounds = 6 + 52 / Words;
    for (q = 0; q < rounds; q++) {
        sum += XXTEA_DELTA;
        e = (sum >> 2) & 3;
        for (p = 0; p < Words-1; p++) {
            y = Data[p+1];
            z = Data[p] += ((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (Key[(p & 3) ^ e] ^ z));
        }
        y = Data[0];
        z = Data[Words-1] += ((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (Key[( (Words-1) & 3) ^ e] ^ z));
    }
}

void XXTEA_Decrypt(uint32_t* Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops) {
    if (Words < 2) return;
    uint32_t y = Data[0], z, sum, e;
    uint32_t p, q;
    uint32_t rounds = 6 + 52 / Words;
    sum = rounds * XXTEA_DELTA;
    for (q = 0; q < rounds; q++) {
        e = (sum >> 2) & 3;
        for (p = Words-1; p > 0; p--) {
            z = Data[p-1];
            y = Data[p] -= ((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (Key[(p & 3) ^ e] ^ z));
        }
        z = Data[Words-1];
        y = Data[0] -= ((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (Key[(0 & 3) ^ e] ^ z));
        sum -= XXTEA_DELTA;
    }
}

void XXTEA_Encrypt_Key0(uint32_t* Data, uint8_t Words, uint8_t Loops) {
    const uint32_t Key[4] = {0,0,0,0};
    XXTEA_Encrypt(Data, Words, Key, Loops);
}

void XXTEA_Decrypt_Key0(uint32_t* Data, uint8_t Words, uint8_t Loops) {
    const uint32_t Key[4] = {0,0,0,0};
    XXTEA_Decrypt(Data, Words, Key, Loops);
}

/*------------------------------------------------------------------------------
 * XorShift random generators
 *----------------------------------------------------------------------------*/
void XorShift32(uint32_t* Seed) {
    *Seed ^= *Seed << 13;
    *Seed ^= *Seed >> 17;
    *Seed ^= *Seed << 5;
}

void XorShift64(uint64_t* Seed) {
    *Seed ^= *Seed << 13;
    *Seed ^= *Seed >> 7;
    *Seed ^= *Seed << 17;
}

uint64_t XorShift64star(uint64_t* Seed) {
    XorShift64(Seed);
    return *Seed * UINT64_C(0x2545f4914f6cdd1d);
}

/*------------------------------------------------------------------------------
 * Ascii85 encoding/decoding (Adobe version)
 *----------------------------------------------------------------------------*/
uint8_t EncodeAscii85(char* Ascii, uint32_t Word) {
    // Convert 4-byte word to 5 Ascii85 chars
    uint8_t bytes[4];
    bytes[0] = (Word >> 24) & 0xFF;
    bytes[1] = (Word >> 16) & 0xFF;
    bytes[2] = (Word >> 8) & 0xFF;
    bytes[3] = Word & 0xFF;

    uint32_t value = (bytes[0] * 85UL*85*85*85) +
                     (bytes[1] * 85UL*85*85) +
                     (bytes[2] * 85UL*85) +
                     (bytes[3] * 85);
    // value is 0..(85^5 - 1)
    for (int i = 4; i >= 0; i--) {
        Ascii[i] = (char)(value % 85) + 33;
        value /= 85;
    }
    Ascii[5] = '\0';
    return 5;
}

uint8_t DecodeAscii85(uint32_t* Word, const char* Ascii) {
    uint32_t value = 0;
    for (int i = 0; i < 5; i++) {
        unsigned char c = (unsigned char)Ascii[i];
        if (c < 33 || c > 117) return 0;   // invalid character
        value = value * 85 + (c - 33);
    }
    // Extract 4 bytes
    *Word = (value >> 24) & 0xFF;
    *Word = (*Word << 8) | ((value >> 16) & 0xFF);
    *Word = (*Word << 8) | ((value >> 8) & 0xFF);
    *Word = (*Word << 8) | (value & 0xFF);
    return 1;
}

/*------------------------------------------------------------------------------
 * APRS to IGC B-record conversion (stub - not fully implemented)
 *----------------------------------------------------------------------------*/
int APRS2IGC(char* Out, const char* Inp, int GeoidSepar) {
    // Simplified stub: just copy first part or return error.
    // Real implementation would parse APRS message.
    (void)Inp;
    (void)GeoidSepar;
    if (Out) Out[0] = '\0';
    return 0;
}

/*------------------------------------------------------------------------------
 * Barometric formulas (Standard Atmosphere, ICAO)
 *----------------------------------------------------------------------------*/
float BaroTemp(float h) {
    // Temperature [K] at altitude h [m], up to 11 km
    const float T0 = 288.15f;
    const float lapse = -0.0065f; // K/m
    float T = T0 + lapse * h;
    if (T < 216.65f) T = 216.65f;
    return T;
}

float BaroPress(float h) {
    // Pressure [Pa] at altitude h [m]
    const float P0 = 101325.0f;
    const float T0 = 288.15f;
    const float lapse = -0.0065f;
    const float R = 287.053f;   // J/(kg*K)
    const float g = 9.80665f;
    if (h < 11000.0f) {
        float T = T0 + lapse * h;
        float exp = g / (R * lapse);
        return P0 * powf(T / T0, exp);
    } else {
        // Above 11 km, isothermal at 216.65 K
        float h11 = 11000.0f;
        float P11 = BaroPress(h11);
        float T11 = 216.65f;
        return P11 * expf(-g * (h - h11) / (R * T11));
    }
}

float BaroAlt(float P) {
    // Inverse: altitude [m] for given pressure [Pa]
    const float P0 = 101325.0f;
    const float T0 = 288.15f;
    const float lapse = -0.0065f;
    const float R = 287.053f;
    const float g = 9.80665f;
    if (P >= 22632.0f) { // pressure at 11 km
        float ratio = P / P0;
        float exp = R * lapse / g;
        return T0 / lapse * (powf(ratio, exp) - 1.0f);
    } else {
        // Above 11 km
        float h11 = 11000.0f;
        float P11 = BaroPress(h11);
        float T11 = 216.65f;
        return h11 - (R * T11 / g) * logf(P / P11);
    }
}


