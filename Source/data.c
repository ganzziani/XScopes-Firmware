#include <avr/pgmspace.h>
#include "main.h"
#include "mso.h"

const uint32_t Powersof10[] PROGMEM = { 1,10,100,1000,10000,100000,1000000, 10000000 };
// index                                0 1  2   3    4     5      6        7

// Offset calibration table (compensates OpAmp's offset voltage)
// One row for each srate
int16_t EEMEM offset16CH1    = 0;
int16_t EEMEM offset16CH2    = 0;
int8_t  EEMEM gain8CH1       = 0;     // 1
int8_t  EEMEM gain8CH2       = 0;     // 1
int8_t  EEMEM offset8CH1[8][7] = { { 0 } };
int8_t  EEMEM offset8CH2[7][7] = { { 0 } };

// Default settings stored in flash
const uint8_t FLGPIO[12] PROGMEM = {
    6,                                      // GPIO0 Srate:     500us/div
    _BV(chon),                              // GPIO1 CH1ctrl:   Channel on
    _BV(chon),                              // GPIO2 CH2ctrl:   Channel on
    _BV(low),                               // GPIO3 CHDctrl:   Channel off, thick low on
    0xFF,                                   // GPIO4 CHDmask:   All bits selected
    _BV(edge),                              // GPIO5 Trigger:   Edge trigger
    0x00,                                   // GPIO6 Mcursors:  Cursors off
    _BV(line)|_BV(showset)|_BV(grid0),      // GPIO7 Display:   Show settings, line, Normal grid
    _BV(hamming)|_BV(uselog)|_BV(scopemode),// GPIO8 MFFT:      Hamming window, no log, scope mode
    0x00,                                   // GPIO9 Sweep:     No Sweep
    0x1C,                                   // GPIOA Sniffer:   Off, UART: 8 Data bits, 19200 bps
	_BV(vdc),                               // GPIOB MStatus
};

// Saved settings stored in EEProm
uint8_t EEMEM EEGPIO[12] = {
    6,                                      // GPIO0 Srate:     500us/div
    _BV(chon),                              // GPIO1 CH1ctrl:   Channel on
    _BV(chon),                              // GPIO2 CH2ctrl:   Channel on
    _BV(low),                               // GPIO3 CHDctrl:   Channel off, thick low on
    0xFF,                                   // GPIO4 CHDmask:   All bits selected
    _BV(edge),                              // GPIO5 Trigger:   Edge trigger
    0x00,                                   // GPIO6 Mcursors:  Cursors off
    _BV(line)|_BV(showset)|_BV(grid0),      // GPIO7 Display:   Show settings, line, Normal grid
    _BV(hamming)|_BV(uselog)|_BV(scopemode),// GPIO8 MFFT:      Hamming window, no log, scope mode
    0x00,                                   // GPIO9 Sweep:     No Sweep
    0x1C,                                   // GPIOA Sniffer:   Off, UART: 8 Data bits, 19200 bps
    _BV(vdc),                               // GPIOB MStatus
};

// Default settings stored in flash
const NVMVAR FLM PROGMEM = {            // Flash Copy
    1,      //  CH1gain;        // Channel 1 gain
    1,      //  CH2gain;        // Channel 2 gain
    64,     //  HPos:           // Center of the buffer
    32,     //  VcursorA;       // Vertical cursor A
    96,     //  VcursorB;       // Vertical cursor B
    66,     //  Hcursor1A;      // CH1 Horizontal cursor A
    62,     //  Hcursor1B;      // CH1 Horizontal cursor B
    66,     //  Hcursor2A;      // CH2 Horizontal cursor A
    62,     //  Hcursor2B;      // CH2 Horizontal cursor B
    0,      //  thold;          // Trigger Hold
    128,    //  Tpost;          // Post Trigger
    0,      //  Tsource;        // Trigger source
    128,    //  Tlevel;         // Trigger Level
    160,    //  Window1;        // Window Trigger low level 1
    96,     //  Window2;        // Window Trigger high level 2
    24,     //  Ttimeout        // 1.024 second timeout
    -96,    //  CH1pos;         // Channel 1 position
    -32,    //  CH2pos;         // Channel 2 position
    0,      //  CHDpos;         // Channel position
    0,      //  CHDdecode;      // Decode Protocol
    1,      //  Sweep1;         // Sweep Start Frequency
    255,    //  Sweep2;         // Sweep End Frequency
    16,     //  SWSpeed;        // Sweep Speed
    -128,   //  AWGamp;         // Amplitude  4V
    1,      //  AWGtype;        // Waveform type Sinewave
    128,    //  AWGduty;        // Duty cycle range: 50%
    0,      //  AWGoffset;      // Offset 0V
    44000,  //  AWGdesiredF;    // Desired frequency 440Hz
}; 

// Saved settings stored in EEProm
NVMVAR EEMEM EEM = {                    // EEMEM Copy
    1,      //  CH1gain;        // Channel 1 gain
    1,      //  CH2gain;        // Channel 2 gain
    64,     //  HPos:           // Center of the buffer
    32,     //  VcursorA;       // Vertical cursor A
    96,     //  VcursorB;       // Vertical cursor B
    66,     //  Hcursor1A;      // CH1 Horizontal cursor A
    62,     //  Hcursor1B;      // CH1 Horizontal cursor B
    66,     //  Hcursor2A;      // CH2 Horizontal cursor A
    62,     //  Hcursor2B;      // CH2 Horizontal cursor B
    0,      //  thold;          // Trigger Hold
    128,    //  Tpost;          // Post Trigger
    0,      //  Tsource;        // Trigger source
    128,    //  Tlevel;         // Trigger Level
    160,    //  Window1;        // Window Trigger low level 1
    96,     //  Window2;        // Window Trigger high level 2
    24,     //  Ttimeout        // 1.024 second timeout
    -96,    //  CH1pos;         // Channel 1 position
    -32,    //  CH2pos;         // Channel 2 position
    0,      //  CHDpos;         // Channel position
    0,      //  CHDdecode;      // Decode Protocol
    1,      //  Sweep1;         // Sweep Start Frequency
    255,    //  Sweep2;         // Sweep End Frequency
    16,     //  SWSpeed;        // Sweep Speed
    -128,   //  AWGamp;         // Amplitude  4V
    1,      //  AWGtype;        // Waveform type Sinewave
    128,    //  AWGduty;        // Duty cycle range: 50%
    0,      //  AWGoffset;      // Offset 0V
    44000,  //  AWGdesiredF;    // Desired frequency 440Hz
};

// Maximum values
const NVMVAR MAXM PROGMEM = {            // Flash Copy
    6,      //  CH1gain;        // Max Gain is 6
    6,      //  CH2gain;        // Max Gain is 6
    127,    //  HPos            // Max Horizontal Position
    127,    //  VcursorA;       // Max cursor position
    127,    //  VcursorB;       // Max cursor position
    127,    //  Hcursor1A;      // Max cursor position
    127,    //  Hcursor1B;      // Max cursor position
    127,    //  Hcursor2A;      // Max cursor position
    127,    //  Hcursor2B;      // Max cursor position
    255,    //  thold;          //
    0x7FFF, //  Tpost;          // Max Post Trigger
    10,     //  Tsource;        // 10 Trigger sources
    252,    //  Tlevel;         // Max Trigger Level
    255,    //  Window1;        //
    255,    //  Window2;        //
    255,    //  Ttimeout        // Trigger Timeout
    255,    //  CH1pos;         //
    255,    //  CH2pos;         //
    64,     //  CHDpos;         // Max CHD position
    2,      //  CHDdecode;      // 3 protocols
    255,    //  Sweep1;         //
    255,    //  Sweep2;         //
    127,    //  SWSpeed;        // Sweep Speed Max
    255,    //  AWGamp;         //
    5,      //  AWGtype;        // 5 waveform type
    255,    //  AWGduty;        //
    255,    //  AWGoffset;      //
    0x00BEFFFF,  //  AWGdesiredF;    // 12517375
};

// Hamming window = 0.53836-0.46164*COS(2*PI*n/(FFT_N-1))
const int8_t Hamming[128] PROGMEM = {
      9,  9,  9,  9, 10, 10, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13,
     14, 14, 15, 16, 16, 17, 18, 19, 19, 20, 21, 22, 23, 24, 25, 26,
     27, 28, 29, 30, 31, 32, 33, 35, 36, 37, 38, 39, 41, 42, 43, 45,
     46, 47, 49, 50, 52, 53, 54, 56, 57, 59, 60, 62, 63, 64, 66, 67,
     69, 70, 72, 73, 75, 76, 77, 79, 80, 82, 83, 85, 86, 87, 89, 90,
     91, 93, 94, 95, 97, 98, 99,100,102,103,104,105,106,107,108,110,
    111,112,113,114,114,115,116,117,118,119,119,120,121,121,122,123,
    123,124,124,125,125,126,126,126,126,127,127,127,127,127,127,127,
};

// Hann window = 0.5*(1-COS(2*PI*n/(FFT_N-1)))
const int8_t Hann[128] PROGMEM = {
      0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  2,  2,  3,  3,  4,
      4,  5,  6,  6,  7,  8,  9, 10, 10, 11, 12, 13, 14, 15, 16, 17,
     18, 20, 21, 22, 23, 24, 26, 27, 28, 29, 31, 32, 34, 35, 36, 38,
     39, 41, 42, 44, 45, 47, 48, 50, 51, 53, 54, 56, 58, 59, 61, 62,
     64, 65, 67, 69, 70, 72, 73, 75, 76, 78, 79, 81, 83, 84, 86, 87,
     88, 90, 91, 93, 94, 95, 97, 98,100,101,102,103,105,106,107,108,
    109,110,111,112,113,114,115,116,117,118,119,120,120,121,122,122,
    123,123,124,124,125,125,126,126,126,127,127,127,127,127,127,127,
};

// Blackman window = 0.42 -0.5*cos(2*PI*n/(FFT_N-1)) +0.08*cos(4*PI*n/(FFT_N-1))
const int8_t Blackman[128] PROGMEM = {
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
      1,  2,  2,  2,  3,  3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,
      8,  9,  9, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
     22, 23, 24, 25, 26, 28, 29, 30, 32, 33, 34, 36, 37, 39, 40, 42,
     43, 45, 47, 48, 50, 52, 53, 55, 57, 59, 60, 62, 64, 66, 67, 69,
     71, 73, 75, 77, 78, 80, 82, 84, 86, 87, 89, 91, 92, 94, 96, 97,
     99,101,102,104,105,107,108,110,111,112,113,115,116,117,118,119,
    120,121,122,123,123,124,125,125,126,126,127,127,127,127,127,127,
};

const int8_t Exp[128] PROGMEM = {      // AWG Exponential
    -117,-107, -97, -87, -78, -69, -61, -53, -45, -38, -31, -24, -18, -12,  -6,   0,
       5,  11,  16,  20,  25,  29,  33,  37,  41,  45,  49,  52,  55,  58,  61,  64,
      67,  69,  72,  74,  76,  79,  81,  83,  85,  87,  88,  90,  92,  93,  95,  96,
      97,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 111, 112,
     113, 113, 114, 115, 115, 116, 116, 117, 117, 118, 118, 118, 119, 119, 120, 120,
     120, 121, 121, 121, 122, 122, 122, 122, 123, 123, 123, 123, 123, 124, 124, 124,
     124, 124, 124, 125, 125, 125, 125, 125, 125, 125, 126, 126, 126, 126, 126, 126,
     126, 126, 126, 126, 126, 126, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
};

int8_t EEMEM EEwave[256] = {      // Custom AWG, preset ECG
       0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
       0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
       0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   6,   7,   7,   8,   8,
       8,   8,   7,   7,   6,   5,   5,   4,   3,   2,   1,   1,   0,   0,   0,  -1,
      -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -2,  -3,  -6,  -9, -12,  -1,  15,
      34,  50,  72,  93, 109, 125, 106,  82,  59,  35,   7, -19, -36, -30, -22, -14,
      -6,  -1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,
       1,   1,   1,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,
       5,   5,   5,   6,   6,   7,   7,   8,   9,   9,  10,  11,  12,  12,  13,  14,
      15,  15,  16,  16,  17,  17,  18,  18,  18,  19,  19,  19,  19,  19,  19,  19,
      18,  18,  17,  17,  16,  15,  14,  12,  11,   9,   8,   7,   5,   4,   3,   3,
       2,   1,   1,   0,   0,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,   0,   0,   0,
       1,   1,   2,   2,   3,   3,   3,   3,   2,   2,   1,   1,   1,   0,   0,   0,
       0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
       0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
       0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};
