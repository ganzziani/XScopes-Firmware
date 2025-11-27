// Xminilab Portable, Hardware 1.11
// Hardware specific definitions

#ifndef HARDWARE_H
#define HARDWARE_H

#include "SSD1309.h"

#define MENUPULL    0x50    			// Menu button Pull down, invert pin
#define XMINIPORTABLE
#define BUFFER_AWG          256         // Buffer size for the AWG output
#define BUFFER_SERIAL       640         // Buffer size for SPI or UART Sniffer
#define BUFFER_I2C          1024        // Serial Sniffer I2C size
#define DATA_IN_PAGE_I2C    64          // I2C data in a page
#define DATA_IN_PAGE_SERIAL 40          // Data that fits on a page in the sniffer

#define LCD_RESET           1           // RESET
#define LCD_RS              3           // Data / Instruction
#define LCD_CS              0           // Chip Select
#define	LCD_CTRL            VPORT0.OUT  // PORTB

// PORT DEFINITIONS
#define LEDGRN              4           // PORTD
#define LEDRED              0           // PORTD

#define EXT_TRIGGER         0x6D        // Event CH2 = PORTD Pin 5

#define ONGRN()             setbit(VPORT3.OUT, LEDGRN)
#define OFFGRN()            clrbit(VPORT3.OUT, LEDGRN)
#define ONRED()             setbit(VPORT3.OUT, LEDRED)
#define OFFRED()            clrbit(VPORT3.OUT, LEDRED)

#define LCD_POWER           2            // PORTD
#define LCDVOLTON() clrbit(VPORT3.OUT, LCD_POWER)
#define LCDVOLTOFF() setbit(VPORT3.OUT, LCD_POWER)

// Port definitions for Assembly code

#define EXTPIN              0x001e,5 // External trigger pin is VPORT3.5
#define CH1ADC              0x0224   // ADCA CH0.RESL
#define CH2ADC              0x022C   // ADCA CH1.RESL

#define AWG_SCALE 16    // AWG scale. Xminilab Portable has 4V output on the AWG

#endif
