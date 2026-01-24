#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "hardware.h"
#include "ffft.h"
#include "mygccdef.h"
#include "asmutil.h"
#include "data.h"
#include "strings.h"
#include "display.h"
#include "bitmaps.h"

// Oscilloscope Global variables, using GPIO for optimized access
#define Srate       GPIO0   // Sampling rate
#define CH1ctrl     GPIO1   // CH1 controls
#define CH2ctrl     GPIO2   // CH2 controls
#define CHDctrl     GPIO3   // CHD controls 1
#define CHDmask     GPIO4   // Mask inputs
#define Trigger     GPIO5   // MSO Trigger
#define Mcursors    GPIO6   // MSO Cursors
#define Display     GPIO7   // Display options
#define MFFT        GPIO8   // MSO FFT Options and mode settings
#define Sweep       GPIO9   // Sweep Options
#define Sniffer     GPIOA   // Sniffer controls
#define MStatus     GPIOB   // MSO Trigger mode and cursors
#define Misc        GPIOC   // Miscellaneous bits
#define Index       GPIOD   // sample index
#define Menu        GPIOE   // Menu
#define Buttons     GPIOF   // Current buttons pressed

// CH1ctrl bits    (GPIO1)
// CH2ctrl bits    (GPIO2)
#define chon        0       // Channel on
#define chx10       1       // x10 probe
#define bwlimit     2       // Bandwidth Limit
#define acdc        3       // AC/DC Select
#define chinvert    4       // Invert channel
#define chaverage   5       // Average samples
#define chmath      6       // Math (Subtract or Multiply active)
#define submult     7       // Subtract or Multiply

// CHDctrl bits    (GPIO3)
#define chon        0       // Channel on
#define pull        1       // Pull input
#define pullup      2       // Pull up / down
#define low         3       // Thick line when logic '0'
#define chinvert    4       // Invert channel
#define hexs        5       // Serial Hex Display
#define hexp        6       // Parallel Hex Display
#define ascii       7       // Use ASCII with UART and SPI Sniffer

// Trigger bits    (GPIO5)
#define normal      0       // Normal trigger
#define single      1       // Single trigger
#define autotrg     2       // Auto trigger
#define trigdir     3       // trigger falling or rising
#define round       4       // Sniffer circular buffer
#define slope       5       // Trigger mode edge or slope
#define window      6       // Window trigger
#define edge        7       // Edge trigger
                            // Dual Edge trigger if others not set

// Mcursors bits    (GPIO6)
#define roll        0       // Roll scope on slow sampling rates
#define autocur     1       // Auto cursors
#define track       2       // Track vertical with horizontal
#define cursorh1    3       // CH1 Horizontal Cursor on
#define cursorh2    4       // CH2 Horizontal Cursor on
#define cursorv     5       // Vertical Cursor on
#define reference   6       // Reference waveforms on
#define singlesniff 7       // Stop when Sniffer buffer is full

// Display bits     (GPIO7) // Display options
#define grid0       0       // Grid settings (2 bits)
#define grid1       1
#define elastic     2       // Average on successive traces
#define disp_inv    3       // Invert display
#define flip        4       // Flip display
#define persistent  5       // Persistent Display
#define line        6       // Continuous Drawing
#define showset     7       // Show scope settings (time/div volts/div)

// MFFT bits        (GPIO8)
#define hamming     0       // FFT Hamming window
#define hann        1       // FFT Hann window
#define blackman    2       // FFT Blackman window
#define uselog      3       // Apply logarithm to FFT
#define iqfft       4       // Perform IQFFT
#define scopemode   5       // MSO mode
#define xymode      6       // XY mode
#define fftmode     7       // FFT mode
                            // Meter mode enabled when other modes are 0

// Sweep bits       (GPIO9)
#define SWAcceldir  0       // Acceleration direction
#define SWAccel     1       // Accelerate
#define swdown      2       // Sweep direction
#define pingpong    3       // Ping Pong Sweep
#define SweepF      4       // Sweep Frequency
#define SweepA      5       // Sweep Amplitude
#define SweepO      6       // Sweep Offset
#define SweepD      7       // Sweep Duty Cycle

// Sniffer bits    (GPIOA)
#define baud0       0       // UART Baud rate
#define baud1       1       // UART Baud rate
#define baud2       2       // UART Baud rate
#define uart0       3       // UART Data bits
#define uart1       4       // UART Data bits
#define parmode     5       // Use UART parity
#define parity      6       // UART parity odd or even
#define stopbit     7       // UART 1 or 2 stop bits
#define SSINV       5       // SPI Invert Select
#define CPOL        6       // SPI Clock Polarity
#define CPHA        7       // SPI Clock Phase

// MStatus bits     (GPIOB)
#define update      0       // Update
#define updateawg   1       // Update AWG
#define updatemso   2       // Settings changed, need to call Apply() function
#define gosniffer   3       // Enter protocol sniffer
#define stop        4       // Scope stopped
#define triggered   5       // Scope triggered
#define vdc         6       // Calculate VDC
#define vp_p        7       // Calculate VPP
                            // Calculate frequency if other bits are 0
                            // Counter if both bits are 1

// Misc             (GPIOC) // Miscellaneous bits
#define keyrep      0       // Automatic key repeat
#define negative    1       // Print Negative font
#define bigfont     2       // Small or large font, this bit is normally 0
#define redraw      3       // Redraw screen
#define sacquired   4       // Data has been acquired (for slow sampling)
#define slowacq     5       // Acquired one set of samples a slow sampling rates
#define lowbatt     6       // VCC is below 3.15V
#define autosend    7       // Continuously send data to UART

// Key              (GPIOF) // Key input
#define KM          0       // KM pressed
#define K3          1       // K3 pressed
#define K2          2       // K2 pressed
#define K1          3       // K1 pressed
#define userinput   4       // User input received
//#define           5       // normally 0
//#define           6       // normally 0
//#define           7       // normally 0

void WaitDisplay(void);
void PowerDown(void);
void MSO(void);
void Restore(void);
void SaveEE(void);          // Save settings to EEPROM
void CalibrateOffset(void);
uint8_t ReadCalibrationByte(uint8_t location);	// Read out calibration byte
void delay_ms(uint8_t n);

extern uint8_t EEMEM EESleepTime;     // Sleep timeout in minutes

// Big buffer to store large but temporary data
typedef union {
    struct {
        complex_t   bfly[FFT_N];	// FFT buffer: (re16,im16)*256 = 1024 bytes
        uint8_t     magn[FFT_N];	// Magnitude output: 128 bytes, IQ: 256 bytes
    } FFT;
    struct {
        int8_t      CH1[512];		// CH1 Temp data
        int8_t      CH2[512];		// CH2 Temp data
        uint8_t     CHD[512];       // CHD Temp data
        union {
            int16_t Vdc[2];         // Channel 1 and Channel 2 DC
            uint32_t Freq;
        } METER;
    } IN;
    struct {
        int8_t AWGTemp1[BUFFER_AWG];
        int8_t AWGTemp2[BUFFER_AWG];
    } DATA;
    struct {
        union {
            struct {
                uint8_t RX[BUFFER_SERIAL];
                uint8_t TX[BUFFER_SERIAL];
            } Serial;
            struct {
                uint8_t decoded[BUFFER_I2C];
                uint8_t addr_ack[BUFFER_I2C/4+1];
            } I2C;
            struct {
                uint8_t decoded[BUFFER_SERIAL*2];
            } All;
        } data;
        volatile uint16_t indrx;    // RX index
        volatile uint16_t indtx;    // TX index
        uint8_t databits;           // UART Data bits
        uint8_t *addr_ack_ptr;      // pointer to address or data (packed bits)
        uint8_t *data_ptr;
        uint8_t addr_ack_pos;       // counter for keeping track of all bits / location for DMA
        uint16_t baud;              // Baud rate
    } LOGIC;
} TempData;

// Variables that need to be stored in NVM

enum protocols { spi, i2c, rs232, irda, onewire, midi };

typedef struct {
//  Type        Name            Index Description
    uint8_t     CH1gain;        // 12 Channel 1 gain
    uint8_t     CH2gain;        // 13 Channel 2 gain
    uint8_t     HPos;           // 14 Horizontal Position
    uint8_t     VcursorA;       // 15 Vertical cursor A
    uint8_t     VcursorB;       // 16 Vertical cursor B
    uint8_t     Hcursor1A;      // 17 CH1 Horizontal cursor A
    uint8_t     Hcursor1B;      // 18 CH1 Horizontal cursor B
    uint8_t     Hcursor2A;      // 19 CH2 Horizontal cursor A
    uint8_t     Hcursor2B;      // 20 CH2 Horizontal cursor B
    uint8_t     Thold;          // 21 Trigger Hold
    uint16_t    Tpost;          // 22 23 Post Trigger
    uint8_t     Tsource;        // 24 Trigger source
    uint8_t     Tlevel;         // 25 Trigger Level
    uint8_t     Window1;        // 26 Window Trigger level 1
    uint8_t     Window2;        // 27 Window Trigger level 2
    uint8_t     Ttimeout;       // 28 Trigger Timeout
    int8_t      CH1pos;         // 29 Channel 1 position
    int8_t      CH2pos;         // 30 Channel 2 position
    uint8_t     CHDpos;         // 31 Channel position
    uint8_t     CHDdecode;      // 32 Decode Protocol
    uint8_t     Sweep1;         // 33 Sweep Start Frequency
    uint8_t     Sweep2;         // 34 Sweep End Frequency
    uint8_t     SWSpeed;        // 35 Sweep Speed
    int8_t      AWGamp;         // 36 Amplitude range: [-128,0]
    uint8_t     AWGtype;        // 37 Waveform type
    uint8_t     AWGduty;        // 38 Duty cycle range: [1,255]
    int8_t      AWGoffset;      // 39 Offset
    uint32_t    AWGdesiredF;    // 40 41 42 43 Desired frequency multiplied by 100
} NVMVAR;

extern TempData T;
extern NVMVAR M;

#endif
