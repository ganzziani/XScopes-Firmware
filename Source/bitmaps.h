#ifndef _BITMAPS_H
#define _BITMAPS_H

#include <stdint.h>

#define FONT3x6_d1      0x30    // Long d character part 1
#define FONT3x6_d2      0x38    // Long d character part 2
#define FONT3x6_m       0x08    // End of long m character

extern const uint8_t    Logo[];
extern const uint8_t    Font3x6[];
extern const uint8_t    Font10x15[];
extern const int8_t     TrigDown[];
extern const int8_t     TrigUp[];
extern const int8_t     TrigDual[];
extern const uint8_t    PulsePosIcon[];
extern const uint8_t    PulseNegIcon[];
extern const uint8_t    BattIcon[];

#endif