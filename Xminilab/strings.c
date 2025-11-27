#include <avr/pgmspace.h>
#include "main.h"

//#define SPANISH

const char VERSION[]    PROGMEM = "FW 2.70";

// Strings with special characters:
// 0x1F = delta
const char one_over_delta_T[] PROGMEM = { ' ', '1', 0x16, 0x1F, 'T', '=', 0 };  // 1/delta T
const char delta_V[]    PROGMEM = { 0x1F, 'V', '=', 0 };        // delta V
const char unitkHz[]    PROGMEM = "KHZ";                        // KHz
const char Kcount[]     PROGMEM = "KCNT";                       // C
const char Stop[]       PROGMEM = "STOP";                       // STOP
const char unitmV[]     PROGMEM = { ' ', 0x1A, 0x1B, 'V', 0 };  // mV
const char unitV[]      PROGMEM = " V";                         // V
const char Vdiv[]		PROGMEM = { 'V', 0x1C, 0x1D, 0x1E, 0 }; // V/div
const char Sdiv[]		PROGMEM = { 'S', 0x1C, 0x1D, 0x1E, 0 }; // S/div
const char Fone[]       PROGMEM = "F1: ";                       // 1:
const char Ftwo[]       PROGMEM = "F2: ";                       // 2:
