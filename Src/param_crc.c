#define PARAM_CRC_C
// Utility class for CRC calculation in parametric model without tables. 
// Here some commonly used crc algorithms: 
// (check is CRC value computed on ASCII string "123456789")

#include <stdint.h>
#include <stdbool.h>
#include "param_crc.h"


uint8_t reverse8( uint8_t value )
// Reverses bit order in the byte. More more information 
// see Henry S.Warren "Hacker's Delight"
// param: value byte to be reversed
// return: Reversed value of input byte. 
// E.g. if input was 0xAC (10101100b) then output will be 0x35 (00110101b)
{
    value = (uint8_t) ((value & 0x55) << 1 | (value & 0xAA) >> 1);
    value = (uint8_t) ((value & 0x33) << 2 | (value & 0xCC) >> 2);
    value = (uint8_t) ((value & 0x0F) << 4 | (value & 0xF0) >> 4);

    return value;
}


uint16_t reverse16( uint16_t value )
// Reverses bit order in the short. More more information 
// see Henry S.Warren "Hacker's Delight"
// param: value short to be reversed
// return: Reversed value of input short. 
//   E.g. if input was 0x62AE (0110001010101110b) 
// then output will be 0x7546 (0111010101000110b)
{
    value = (uint16_t) ((value & 0x5555) << 1 | (value & 0xAAAA) >> 1);
    value = (uint16_t) ((value & 0x3333) << 2 | (value & 0xCCCC) >> 2);
    value = (uint16_t) ((value & 0x0F0F) << 4 | (value & 0xF0F0) >> 4);
    value = (uint16_t) ((value & 0x00FF) << 8 | (value & 0xFF00) >> 8);

    return value;
}


uint32_t reverse32( uint32_t value)
// Reverses bit order in the long int. More more information 
// see Henry S.Warren "Hacker's Delight"
// param: value long int to be reversed
// return: Reversed value of input long int. 
//   E.g. if input was 0x000062AE (00000000000000000110001010101110b) 
// then output will be 0x75460000 (01110101010001100000000000000000b)
{
    value = (uint32_t) ((value & 0x55555555UL) << 1
            | (value & 0xAAAAAAAAUL) >> 1);
    value = (uint32_t) ((value & 0x33333333UL) << 2
            | (value & 0xCCCCCCCCUL) >> 2);
    value = (uint32_t) ((value & 0x0F0F0F0FUL) << 4
            | (value & 0xF0F0F0F0UL) >> 4);
    value = (uint32_t) ((value & 0x00FF00FFUL) << 8
            | (value & 0xFF00FF00UL) >> 8);
    value = (uint32_t) ((value & 0x0000FFFFUL) << 16
            | (value & 0xFFFF0000UL) >> 16);

    return value;
}


uint32_t param_crc32( uint8_t * data, uint32_t start, uint32_t length )
// Calculates most standard CRC-32 with polynom 0x4C11DB7, 
// start and xor values of -1 and double bit reversing.
// param: data byte array to be processed
// param: start start position from wich calculation begins
// param: length amount of bytes to be processed
// return: calculated crc32
{
    return param_crc_calculate_crc32( data, start, length, \
        0x4C11DB7UL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, true, true );
}


uint32_t param_crc_calculate_crc32( uint8_t * data,
                                     uint32_t start,
                                     uint32_t length,
                                     uint32_t poly,
                                     uint32_t init,
                                     uint32_t xorOut,
                                     bool reverseIn,
                                     bool reverseOut )
// The most general calculation of CRC-32, 
// that takes into account a lot of parameters.
// param: data Byte array to be processed
// param: start start position from wich calculation begins
// param: length amount of bytes to be processed
// param: poly Crc polynom to use
// param: init Start value of CRC register
// param: xorOut Value with wich final CRC will be xor-ed (0 means no xor)
// param: reverseIn Should we reverse input bytes before processing?
// param: reverseOut Should we reverse calculated CRC before final xor?
// return: calculated CRC
{
    uint32_t i;
    uint32_t crc = init;

    for (i = start; i < start + length; i++) {
        crc = param_crc_update_crc32( crc, data[ i ], poly, reverseIn );
    }

    if (reverseOut) {
        crc = reverse32( crc );
    }

    crc ^= xorOut;

    return crc;
}


uint32_t param_crc_update_crc32( uint32_t crc,
                                  uint8_t b,
                                  uint32_t poly,
                                  bool reverseIn )
// Updates already calculated crc with the next byte.
// param: crc Input crc
// param: b ongoing byte
// param: poly polynom to use
// param: reverseIn reverse byte before processing?
// return: updated crc
{
    uint32_t j;
    
    if (reverseIn) {
        b = reverse32( b );
    }

    for (j = 0x80; j != 0; j >>= 1)
    {
        uint32_t bit = crc & 0x80000000UL;
        crc <<= 1;
        if ((b & j) != 0) {
            bit ^= 0x80000000UL;
        }
        if (bit != 0) {
            crc ^= poly;
        }
    }

    return crc;
}


uint16_t param_crc16( uint8_t * data, uint16_t start, uint16_t length )
// Calculates CRC-16 for RTU mode MODBUS protocol: with polynom 0x8005,
// start value of -1, xor value of 0 and double bit reversing.
// param: data byte array to be processed
// param: start start position from wich calculation begins
// param: length amount of bytes to be processed
// return: calculated crc16
{
    return param_crc_calculate_crc16( data, start, length, \
        0x8005, 0xFFFF, 0x0000, true, true );
}


uint16_t param_crc16_init( void )
// Initiates CRC-16 for RTU mode MODBUS protocol: with polynom 0x8005,
// start value of -1, xor value of 0 and double bit reversing.
// return: initiated crc16
{
    return 0xFFFF;
}


uint16_t param_crc16_update( uint16_t crc, uint8_t b )
// Updates already calculated crc with the next byte for RTU mode
// MODBUS protocol: with polynom 0x8005, xor value of 0 and input reversing.
// param: crc Input crc
// param: b ongoing byte
// return: updated crc16
{
    return param_crc_update_crc16( crc, b, 0x8005, true );
}


uint16_t param_crc16_finit( uint16_t crc )
// Finalizes CRC-16 for RTU mode MODBUS protocol: with polynom 0x8005,
// start value of -1, xor value of 0 and double bit reversing.
// param: crc Input crc
// return: finalized crc16
{
    crc = reverse16( crc );
    return crc ^= 0x0000;
}


uint16_t param_crc_calculate_crc16( uint8_t * data,
                                     uint16_t start,
                                     uint16_t length,
                                     uint16_t poly,
                                     uint16_t init,
                                     uint16_t xorOut,
                                     bool reverseIn,
                                     bool reverseOut )
// The most general calculation of CRC-16,
// that takes into account a lot of parameters.
// param: data Byte array to be processed
// param: start start position from wich calculation begins
// param: length amount of bytes to be processed
// param: poly Crc polynom to use
// param: init Start value of CRC register
// param: xorOut Value with wich final CRC will be xor-ed (0 means no xor)
// param: reverseIn Should we reverse input bytes before processing?
// param: reverseOut Should we reverse calculated CRC before final xor?
// return: calculated CRC
{
    uint16_t i;
    uint16_t crc = init;

    for (i = start; i < start + length; i++) {
        crc = param_crc_update_crc16( crc, data[ i ], poly, reverseIn );
    }

    if (reverseOut) {
        crc = reverse16( crc );
    }

    crc ^= xorOut;

    return crc;
}


uint16_t param_crc_update_crc16( uint16_t crc,
                                  uint8_t b,
                                  uint16_t poly,
                                  bool reflectIn )
// Updates already calculated crc with the next byte.
// param: crc Input crc
// param: b ongoing byte
// param: poly polynom to use
// param: reverseIn reverse byte before processing?
// return: updated crc
{
    uint16_t j;
    
    if (reflectIn) {
        b = reverse8( b );
    }

    for (j = 0x80; j != 0; j >>= 1)
    {
        uint16_t bit = crc & 0x8000;
        crc <<= 1;
        if ((b & j) != 0) {
            bit ^= 0x8000;
        }
        if (bit != 0) {
            crc ^= poly;
        }
    }

    return crc;
}
