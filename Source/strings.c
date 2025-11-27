#include <avr/pgmspace.h>
#include "main.h"

//#define SPANISH

const char VERSION[]    PROGMEM = "FW 2.71";

// Strings with special characters:
// 0x1F = delta
const char STR_1_over_delta_T[] PROGMEM = { ' ', '1', 0x16, 0x1F, 'T', '=', 0 };  // 1/delta T
const char STR_delta_V[]        PROGMEM = { 0x1F, 'V', '=', 0 };        // delta V
const char STR_KHZ[]            PROGMEM = "KHZ";                        // KHz
const char STR_KCNT[]           PROGMEM = "KCNT";                       // C
const char STR_STOP[]           PROGMEM = "STOP";                       // STOP
const char STR_mV[]             PROGMEM = { ' ', 0x1A, 0x1B, 'V', 0 };  // mV
const char STR_V[]              PROGMEM = " V";                         // V
const char STR_Vdiv[]		    PROGMEM = { 'V', 0x1C, 0x1D, 0x1E, 0 }; // V/div
const char STR_Sdiv[]		    PROGMEM = { 'S', 0x1C, 0x1D, 0x1E, 0 }; // S/div
const char STR_F1[]             PROGMEM = "F1: ";                       // 1:
const char STR_F2[]             PROGMEM = "F2: ";                       // 2:
