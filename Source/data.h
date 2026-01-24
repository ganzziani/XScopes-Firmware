#ifndef _DATA_H
#define _DATA_H

#include <stdint.h>
#include "awg.h"
#include "main.h"

extern const uint32_t Powersof10[];
extern const char Minustext[];
extern const char multinv[];
extern int16_t EEMEM offset16CH1;
extern int16_t EEMEM offset16CH2;
extern int8_t EEMEM gain8CH1;
extern int8_t EEMEM gain8CH2;
extern int8_t  EEMEM offset8CH1[8][7];
extern int8_t  EEMEM offset8CH2[8][7];
extern const uint8_t FLGPIO[12];
extern uint8_t EEMEM EEGPIO[12];
extern const int8_t Hamming[128];
extern const int8_t Hann[128];
extern const int8_t Blackman[128];
extern const int8_t Exp[128];
extern int8_t EEMEM EEwave[256];

#endif
