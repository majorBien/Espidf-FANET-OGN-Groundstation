/*
 * ogn_conv.h
 *
 *  Created on: 27 kwi 2026
 *      Author: majorBien
 */

/**
 * @file ogn_conv.h
 * @brief OGN conversion and helper functions (C version of OGNCONV_H)
 */

#ifndef OGN_CONV_H
#define OGN_CONV_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Coordinate conversions between different formats
 *============================================================================*/

/**
 * Convert coordinate from FNT (FLARM) format to OGN format.
 * @param Coord Coordinate in FNT format (1e-5 deg, int32_t)
 * @return Coordinate in OGN format (0.0008/60° for lat, 0.0016/60° for lon)
 */
int32_t Coord_FNTtoOGN(int32_t Coord);

/**
 * Convert coordinate from OGN format to FNT format.
 */
int32_t Coord_OGNtoFNT(int32_t Coord);

/**
 * Convert coordinate from FNT format to UBX (u‑blox) format (1e-7 deg).
 */
int32_t Coord_FNTtoUBX(int32_t Coord);

/**
 * Convert coordinate from UBX format to FNT format.
 */
int32_t Coord_UBXtoFNT(int32_t Coord);

/**
 * Convert coordinate from CRD (internal OGN, 1e-5 deg) to OGN format.
 */
int32_t Coord_CRDtoOGN(int32_t Coord);

/**
 * Convert coordinate from OGN format to CRD format.
 */
int32_t Coord_OGNtoCRD(int32_t Coord);

/*==============================================================================
 * Altitude conversions
 *============================================================================*/

/**
 * Convert feet to meters (1 ft = 0.3048 m).
 * @param Altitude Altitude in feet (int32_t)
 * @return Altitude in millimeters (int32_t)
 */
int32_t FeetToMeters(int32_t Altitude);

/**
 * Convert meters to feet (1 m = 3.280839895 ft).
 * @param Altitude Altitude in millimeters (int32_t)
 * @return Altitude in thousandths of a foot (int32_t)
 */
int32_t MetersToFeet(int32_t Altitude);

/*==============================================================================
 * Aircraft type mappings
 *============================================================================*/

/**
 * Convert OGN aircraft type (0-7) to ADSB category.
 */
uint8_t AcftType_OGNtoADSB(uint8_t AcftType);

/**
 * Convert FNT (FLARM) aircraft type to ADSB category.
 */
uint8_t AcftType_FNTtoADSB(uint8_t AcftType);

/**
 * Convert ADSB category to OGN aircraft type.
 */
uint8_t AcftType_ADSBtoOGN(uint8_t AcftCat);

/**
 * Convert OGN type to GDL format (placeholder, returns same value).
 */
uint8_t AcftType_OGNtoGDL(uint8_t AcftType);

/**
 * Convert OGN type to ADSL (FLARM Advanced) format.
 */
uint8_t AcftType_OGNtoADSL(uint8_t AcftType);

/**
 * Convert ADSL format to OGN type.
 */
uint8_t AcftType_ADSLtoOGN(uint8_t AcftCat);

/**
 * Convert FNT type to OGN type.
 */
uint8_t AcftType_FNTtoOGN(uint8_t AcftType);

/**
 * Convert FNT type to ADSL format.
 */
uint8_t AcftType_FNTtoADSL(uint8_t AcftType);

/*==============================================================================
 * Variable-rate encoding/decoding (floating point compression)
 *============================================================================*/

/**
 * Encode unsigned 12-bit value (0..3832) into 10 bits.
 */
uint16_t EncodeUR2V8(uint16_t Value);

/**
 * Decode 10-bit value to unsigned 12-bit (0..3832).
 */
uint16_t DecodeUR2V8(uint16_t Value);

/**
 * Encode unsigned 9-bit value (0..472) into 7 bits.
 */
uint8_t EncodeUR2V5(uint16_t Value);

/**
 * Decode 7-bit value to unsigned 9-bit (0..472).
 */
uint16_t DecodeUR2V5(uint16_t Value);

/**
 * Encode signed 10-bit value (-472..472) into 8 bits.
 */
uint8_t EncodeSR2V5(int16_t Value);

/**
 * Decode 8-bit value to signed 10-bit (-472..472).
 */
int16_t DecodeSR2V5(int16_t Value);

/**
 * Encode unsigned 10-bit value (0..952) into 8 bits.
 */
uint16_t EncodeUR2V6(uint16_t Value);

/**
 * Decode 8-bit value to unsigned 10-bit (0..952).
 */
uint16_t DecodeUR2V6(uint16_t Value);

/**
 * Encode signed 11-bit value (-952..952) into 9 bits.
 */
uint16_t EncodeSR2V6(int16_t Value);

/**
 * Decode 9-bit value to signed 11-bit (-952..952).
 */
int16_t DecodeSR2V6(int16_t Value);

/*==============================================================================
 * Gray code
 *============================================================================*/

uint8_t  EncodeGray8 (uint8_t  Binary);
uint8_t  DecodeGray8 (uint8_t  Gray);
uint16_t EncodeGray16(uint16_t Binary);
uint16_t DecodeGray16(uint16_t Gray);
uint32_t EncodeGray32(uint32_t Binary);
uint32_t DecodeGray32(uint32_t Gray);

/*==============================================================================
 * TEA / XXTEA block cipher
 *============================================================================*/

void TEA_Encrypt (uint32_t* Data, const uint32_t* Key, int Loops);
void TEA_Decrypt (uint32_t* Data, const uint32_t* Key, int Loops);

void TEA_Encrypt_Key0 (uint32_t* Data, int Loops);
void TEA_Decrypt_Key0 (uint32_t* Data, int Loops);

void XXTEA_Encrypt(uint32_t* Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops);
void XXTEA_Decrypt(uint32_t* Data, uint8_t Words, const uint32_t Key[4], uint8_t Loops);

void XXTEA_Encrypt_Key0(uint32_t* Data, uint8_t Words, uint8_t Loops);
void XXTEA_Decrypt_Key0(uint32_t* Data, uint8_t Words, uint8_t Loops);

/*==============================================================================
 * Pseudo-random generators
 *============================================================================*/

void XorShift32(uint32_t* Seed);
void XorShift64(uint64_t* Seed);
uint64_t XorShift64star(uint64_t* Seed);   // returns 64-bit random value

/*==============================================================================
 * Ascii85 encoding (Adobe)
 *============================================================================*/

/**
 * Encode a 32-bit word into a 5-character Ascii85 string.
 * @param Ascii Output buffer (at least 6 bytes, will be NULL-terminated).
 * @param Word  32-bit value to encode.
 * @return Length of encoded string (always 5).
 */
uint8_t EncodeAscii85(char* Ascii, uint32_t Word);

/**
 * Decode a 5-character Ascii85 string to a 32-bit word.
 * @param Word  Pointer to variable where result will be stored.
 * @param Ascii 5-character string (no NULL terminator needed).
 * @return 1 on success, 0 on error (invalid character).
 */
uint8_t DecodeAscii85(uint32_t* Word, const char* Ascii);

/*==============================================================================
 * APRS to IGC B-record conversion (simplified)
 *============================================================================*/

/**
 * Convert an APRS message (e.g. from FLARM) to an IGC B-record.
 * @param Out   Output buffer for IGC B-record (should be at least 15 bytes).
 * @param Inp   Input APRS string.
 * @param GeoidSepar Geoid separation (meters) – unused in this stub.
 * @return 1 on success, 0 on failure.
 */
int APRS2IGC(char* Out, const char* Inp, int GeoidSepar);

/*==============================================================================
 * Barometric formulas
 *============================================================================*/

/**
 * Standard atmosphere temperature [K] at given altitude [m].
 */
float BaroTemp(float h);

/**
 * Standard atmosphere pressure [Pa] at given altitude [m].
 */
float BaroPress(float h);

/**
 * Standard atmosphere altitude [m] for given pressure [Pa].
 */
float BaroAlt(float P);

#ifdef __cplusplus
}
#endif

#endif /* OGN_CONV_H */