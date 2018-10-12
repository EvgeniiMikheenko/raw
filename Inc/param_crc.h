#ifndef PARAM_CRC_H
#  define PARAM_CRC_H
#endif

#ifdef PARAM_CRC_H

#ifdef __MCCXAP2B__
#  include "app/energomera/common/net/common.h"
typedef boolean bool;
typedef int8u   uint8_t;
typedef int16u  uint16_t;
typedef int32u  uint32_t;
#  define true  1
#  define false 0
#else
#  include <stdint.h>
#  include <stdbool.h>
#endif


// Calculates most standard CRC-32 with polynom 0x4C11DB7,
// start and xor values of -1 and double bit reversing.
// param: data byte array to be processed
// param: start start position from wich calculation begins
// param: length amount of bytes to be processed
// return: calculated crc32
uint32_t param_crc32( uint8_t * data, uint32_t start, uint32_t length );

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
uint32_t param_crc_calculate_crc32( uint8_t * data,
                                     uint32_t start,
                                     uint32_t length,
                                     uint32_t poly,
                                     uint32_t init,
                                     uint32_t xorOut,
                                     bool reverseIn,
                                     bool reverseOut );

// Updates already calculated crc with the next byte.
// param: crc Input crc
// param: b ongoing byte
// param: poly polynom to use
// param: reverseIn reverse byte before processing?
// return: updated crc
uint32_t param_crc_update_crc32( uint32_t crc,
                                  uint8_t b,
                                  uint32_t poly,
                                  bool reverseIn );

// Calculates CRC-16 for RTU mode MODBUS protocol: with polynom 0x8005,
// start value of -1, xor value of 0 and double bit reversing.
// param: data byte array to be processed
// param: start start position from wich calculation begins
// param: length amount of bytes to be processed
// return: calculated crc16
uint16_t param_crc16( uint8_t * data, uint16_t start, uint16_t length );

// Initiates CRC-16 for RTU mode MODBUS protocol: with polynom 0x8005,
// start value of -1, xor value of 0 and double bit reversing.
// return: initiated crc16
uint16_t param_crc16_init( void );

// Updates already calculated crc with the next byte for RTU mode
// MODBUS protocol: with polynom 0x8005, xor value of 0 and input reversing.
// param: crc Input crc
// param: b ongoing byte
// return: updated crc16
uint16_t param_crc16_update( uint16_t crc, uint8_t b );

// Finalizes CRC-16 for RTU mode MODBUS protocol: with polynom 0x8005,
// start value of -1, xor value of 0 and double bit reversing.
// param: crc Input crc
// return: finalized crc16
uint16_t param_crc16_finit( uint16_t crc );


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
uint16_t param_crc_calculate_crc16( uint8_t * data,
                                     uint16_t start,
                                     uint16_t length,
                                     uint16_t poly,
                                     uint16_t init,
                                     uint16_t xorOut,
                                     bool reverseIn,
                                     bool reverseOut );

// Updates already calculated crc with the next byte.
// param: crc Input crc
// param: b ongoing byte
// param: poly polynom to use
// param: reverseIn reverse byte before processing?
// return: updated crc
uint16_t param_crc_update_crc16( uint16_t crc,
                                  uint8_t b,
                                  uint16_t poly,
                                  bool reflectIn );
#endif //ifdef PARAM_CRC_H
