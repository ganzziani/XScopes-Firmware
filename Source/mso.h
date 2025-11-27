#ifndef _MSO_H
#define _MSO_H

#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "ffft.h"

enum {  // Menu reference:
    Mdefault,   // Default Menu
    MCH1,       // " CH  ON  \0      GAIN-    \0   GAIN+", // Channel 1
    MCH2,       // " CH  ON  \0      GAIN-    \0   GAIN+", // Channel 2
    MCHD,       // "LOGIC ON \0     SELECT   \0 PROTOCOL", // Logic
    MSNIFFER,   // " NORMAL   \0    SINGLE   \0 CIRCULAR", // Sniffer mode
    MTRIGTYPE,  // " NORMAL   \0    SINGLE   \0    AUTO ", // Trigger Type
    MCURSOR1,   // " VCURSOR  \0   HCUR CH1 \0  HCUR CH2", // Cursor
    MWINDOW,    // " HAMMING   \0    HANN   \0  BLACKMAN", // Spectrum Analyzer Window
    MSOURCE,    // "  CH1    \0       CH2     \0  LOGIC ", // Trigger Source
    MDISPLAY1,  // " GRID   \0  FLIP DISPLAY \0  INVERT ", // Display
    MMETER,     // "  VDC      \0   V P-P  \0 FREQUENCY ", // Meter mode
    MAWG,       // "  SINE    \0    SQUARE  \0  TRIANGLE", // AWG
    MCH1OPT,    // " POSITION  \0   INVERT   \0    MATH ", // Channel 1 options
    MCH2OPT,    // " POSITION  \0   INVERT   \0    MATH ", // Channel 2 options
    MCHDOPT1,   // " POSITION  \0   INVERT  \0   THICK0 ", // Logic Options 1
    MMAIN1,     // "  CH1   \0       CH2  \0       LOGIC", // Menu Select 1 - Channel
    MMAIN2,     // "TRIGTYPE  \0    TRIGSRC  \0 MORETRIG", // Menu Select 2 - Trigger
    MMAIN3,     // " SCOPE    \0    METER   \0      FFT ", // Menu Select 3 - Mode
    MMAIN4,     // " IQ FFT   \0  FFT WINDOW  \0    LOG ", // Menu Select 4 - FFT
    MMAIN5,     // "CURSORS \0      DISPLAY \0       AWG", // Menu Select 5 - Misc
    MAWG2,      // "WAVE TYPE  \0   SWEEP   \0 FREQUENCY", // AWG Menu 2
    MAWG4,      // " EXP      \0    NOISE   \0   CUSTOM ", // AWG Menu 4
    MAWG5,      // " SPEED    \0    MODE    \0    RANGE ", // AWG Menu 5
    MAWG6,      // "SW FREQ    \0   SW AMP  \0  SW DUTY ", // AWG Menu 6
//    MAWG7,      // " SWEEP    \0  CV/GATE  \0 POS. RANGE", // AWG Menu 7
//    MCVG,       // "CV/GATE   \0  CONTINUOUS \0   C1=1V ", // AWG CV/Gate
    MSCOPEOPT,  // " ROLL     \0   ELASTIC  \0  XY MODE ", // Scope options
    MTRIG2,     // "TRIGMODE  \0  POSTTRIG \0   TRIGHOLD", // Menu Trigger 2
    MTRIGMODE,  // " WINDOW   \0    EDGE    \0    SLOPE ", // Trigger edge and mode
    MCURSOR2,   // "  AUTO     \0   TRACK   \0 REFERENCE", // More Cursor Options
    MCHDSEL1,   // "  ALL     \0    BIT_0    \0    BIT_1", // Logic Channel Select
    MCHDSEL2,   // " BIT_2    \0    BIT_3    \0    BIT_4", // Logic Channel Select
    MCHDSEL3,   // " BIT_5    \0    BIT_6    \0    BIT_7", // Logic Channel Select
    MTSEL1,     // " EXT TRIG \0    BIT_0    \0    BIT_1", // Logic Trigger Select
    MTSEL2,     // " BIT_2    \0    BIT_3    \0    BIT_4", // Logic Trigger Select
    MTSEL3,     // " BIT_5    \0    BIT_6    \0    BIT_7", // Logic Trigger Select
    MCHDOPT2,   // " PARALLEL  \0   SERIAL  \0      PULL", // Logic Options 2
    MPROTOCOL,  // " I2C      \0     UART    \0     SPI ", // Protocol
    MCHDPULL,   // " NO PULL  \0   PULL UP  \0 PULL DOWN", // Logic Inputs Pull
    MDISPLAY2,  // " PERSISTENT  \0   LINE   \0    SHOW ", // Display
    MSPI,       // " CPOL     \0     CPHA    \0      GO ", // SPI Clock polarity and phase
    MCH1MATH,   // " SUBTRACT \0   MULTIPLY \0  AVERAGE ", // Channel 1 math
    MCH2MATH,   // " SUBTRACT \0   MULTIPLY \0  AVERAGE ", // Channel 2 math
    MAWG3,      // "AMPLITUDE \0  DUTY CYCLE \0   OFFSET", // AWG Menu 3
    MSWMODE,    // "  DOWN    \0  PINGPONG   \0   ACCEL ", // Sweep Mode Menu
    MUART,                                                 // UART Settings
    MPOSTT,     // "          \0     MOVE-   \0    MOVE+", // Post Trigger  16 bit
    MAWGFREQ,   // "          \0     MOVE-   \0    MOVE+", // Frequency     32 bit
    MTLEVEL,    // "          \0     MOVE-   \0    MOVE+", // Trigger Level
    MTW1,       // "          \0     MOVE-   \0    MOVE+", // Window Trigger 1
    MTW2,       // "          \0     MOVE-   \0    MOVE+", // Window Trigger 2
    MSW1,       // "          \0     MOVE-   \0    MOVE+", // Sweep Start
    MSW2,       // "          \0     MOVE-   \0    MOVE+", // Sweep End
    MHPOS,      // "STOP      \0     MOVE-   \0    MOVE+", // Run/Stop - Horizontal Scroll
    // shortcuts below
    MSWSPEED,   // "          \0     MOVE-   \0    MOVE+", // Sweep Speed
    MAWGAMP,    // "          \0     MOVE-   \0    MOVE+", // Amplitude
    MAWGOFF,    // "          \0     MOVE-   \0    MOVE+", // Offset
    MAWGDUTY,   // "          \0     MOVE-   \0    MOVE+", // Duty Cycle
    MTHOLD,     // "          \0     MOVE-   \0    MOVE+", // Trigger Hold
    MCH1POS,    // "POSITION  \0     MOVE-   \0    MOVE+", // CH1 Channel position
    MCH2POS,    // "POSITION  \0     MOVE-   \0    MOVE+", // CH2 Channel position
    // shortcuts above
    MVC1,       // "VCURSOR1  \0     MOVE-   \0    MOVE+", // V Cursor 1
    MVC2,       // "VCURSOR2  \0     MOVE-   \0    MOVE+", // V Cursor 2
    MCH1HC1,    // "HCURSOR1  \0     MOVE-   \0    MOVE+", // H Cursor 1 CH1
    MCH1HC2,    // "HCURSOR2  \0     MOVE-   \0    MOVE+", // H Cursor 2 CH1
    MCH2HC1,    // "HCURSOR1  \0     MOVE-   \0    MOVE+", // H Cursor 1 CH2
    MCH2HC2,    // "HCURSOR2  \0     MOVE-   \0    MOVE+", // H Cursor 2 CH2
    MOUTSIDE    // Equal or above this -> out of range
};

typedef struct {
    int8_t  offset;             // Channel offset
    uint8_t max;                // Maximum value
    uint8_t min;                // Minimum value
    uint8_t vpp;                // Peak to peak
    uint8_t f;                  // Maximum frequency
} ACHANNEL;

typedef struct {
    uint8_t CH1data[256];       // CH1 Samples
    uint8_t CH2data[256];       // CH2 Samples
    uint8_t CHDdata[256];       // CHD Samples
    uint8_t frame;              // Frame number
    uint8_t index;              // Index number
} DATA;

extern ACHANNEL CH1, CH2;       // Analog Channel 1, Channel 2
extern DATA DC;

// Function prototypes
void AutoSet(void);
void MSO(void);
void Apply(void);                   // Apply oscilloscope settings
void StartDMAs(void);
void CheckPost(void);               // Check Post Trigger
void SaveEE(void);                  // Save settings to EEPROM

extern uint8_t old_s;
extern uint8_t old_g1;
extern uint8_t old_g2;

#endif
