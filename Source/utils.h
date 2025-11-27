#ifndef _UTILS_H
#define _UTILS_H

#include <stdint.h>

//uint8_t ReadCalibrationByte(uint8_t location);	// Read out calibration byte
void CCPWrite(volatile uint8_t * address, uint8_t value);
uint8_t half(uint8_t number);       // Converts an uint to int, then returns the half
uint8_t twice(uint8_t number);      // Converts an uint to int, then return the double, or saturates
char NibbleToChar(uint8_t nibble);  // Converts a nibble to the corresponding ASCII representing the HEX value

#endif