/* ----------------------------------------------------------------------------
	128x64 Graphic LCD management for SSD1306 driver

	FILE NAME 	: SSD1306.c
	LAYER		: Application

	DESCRIPTION	: The purpose of this function is to manage a graphic LCD
			  by providing function for control and display text and
			  graphic

	AUTHOR		: Gabriel Anzziani
    www.gabotronics.com

 */

/******************************************************************************
/                       DECLARATIONS / DEFINITIONS                            /
/ ****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "display.h"
#include "mygccdef.h"

/* Local functions prototypes */
void LcdDataWrite (unsigned char);
void LcdWaitBusy (void);

/******************************************************************************
/                               PROGRAM CODE                                  /
/ ****************************************************************************/

/*-----------------------------------------------------------------------------
LCD Initialization
	GLCD_LcdINIT()
-----------------------------------------------------------------------------*/
void GLCD_LcdInit(void)	{
    // Recommended power up sequence
    clrbit(LCD_CTRL, LCD_RESET);         // Reset Low for at least 3 uS
  	_delay_us(4);
    setbit(LCD_CTRL, LCD_RESET);         // Reset Low for at least 3 uS
    _delay_us(4);
    cli();
    // Recommended initialization sequence
    LcdInstructionWrite(LCD_DISP_OFF);
    LcdInstructionWrite(LCD_SET_RATIO_OSC);
    LcdInstructionWrite(0x80);
    LcdInstructionWrite(LCD_MULTIPLEX);
    LcdInstructionWrite(0x3F);
    LcdInstructionWrite(LCD_SET_OFFSET);
    LcdInstructionWrite(0x00);
    LcdInstructionWrite(LCD_SET_LINE);
    LcdInstructionWrite(LCD_CHARGE_PUMP);
    LcdInstructionWrite(LCD_PUMP_ON);
//set at main LcdInstructionWrite(LCD_SET_SEG_REMAP1);
//set at main LcdInstructionWrite(LCD_SET_SCAN_NOR);
    LcdInstructionWrite(LCD_SET_PADS);
    LcdInstructionWrite(0x12);
    LcdInstructionWrite(LCD_SET_CONTRAST);
    LcdInstructionWrite(0xFF);
    LcdInstructionWrite(LCD_SET_CHARGE);
    LcdInstructionWrite(0xF1);
    LcdInstructionWrite(LCD_SET_VCOM);
    LcdInstructionWrite(0x40);
    LcdInstructionWrite(LCD_EON_OFF);
    LcdInstructionWrite(LCD_DISP_NOR);
    LcdInstructionWrite(LCD_MEM_ADDRESSING);
    LcdInstructionWrite(0x00);          // Horizontal Addressing mode
    LcdInstructionWrite(LCD_DISP_ON);
    sei();
    Disp_send.display_setup[0]=LCD_SET_PAGE;
    Disp_send.display_setup[1]=LCD_SET_COL_HI;    // Set column at 0
    Disp_send.display_setup[2]=LCD_SET_COL_LO;    
}

void GLCD_LcdOff(void)	{
    cli();
    LcdInstructionWrite(LCD_DISP_OFF);
    LcdInstructionWrite(LCD_EON_OFF);
    LcdInstructionWrite(LCD_PUMP_OFF);
    sei();
    clrbit(VPORT3.OUT, 2);   // Power down board
}

/*-------------------------------------------------------------------------------
Send instruction to the LCD
	LcdInstructionWrite (uint8_t u8Instruction)
		u8Instruction = Instruction to send to the LCDCHSIZE 2 2469
-------------------------------------------------------------------------------*/
void LcdInstructionWrite (uint8_t u8Instruction) {
    clrbit(LCD_CTRL, LCD_CS);			// Select
    clrbit(LCD_CTRL,LCD_RS);            // Instruction mode
    USARTD0.DATA= u8Instruction;
    while(!testbit(USARTD0.STATUS,6));  // Wait until transmit done
    setbit(USARTD0.STATUS,6);
}

// Transfer buffer to display
void dma_display(void) {
    clrbit(LCD_CTRL, LCD_CS);			// Select
    clrbit(LCD_CTRL,LCD_RS);            // Instruction mode
    setbit(DMA.CH2.CTRLA,6);            // reset DMA CH0
    DMA.CH2.ADDRCTRL  = 0b00010000;     // Increment source, Destination fixed
    DMA.CH2.TRFCNT    = 3;              // buffer size
    DMA.CH2.DESTADDR0 = (((uint16_t) &USARTD0.DATA)>>0*8) & 0xFF;
    DMA.CH2.DESTADDR1 = (((uint16_t) &USARTD0.DATA)>>1*8) & 0xFF;
    DMA.CH2.TRIGSRC   = DMA_CH_TRIGSRC_USARTD0_DRE_gc;
    DMA.CH2.SRCADDR0  = (((uint16_t)(Disp_send.display_setup))>>0*8) & 0xFF;
    DMA.CH2.SRCADDR1  = (((uint16_t)(Disp_send.display_setup))>>1*8) & 0xFF;
    DMA.CH2.CTRLA     = 0b10000100;     // no repeat, 1 byte burst
    _delay_us(4);       // Wait for 3 bytes to be sent
    setbit(LCD_CTRL, LCD_RS);           // Data mode
    DMA.CH2.TRFCNT    = DISPLAY_DATA_SIZE;       // buffer size
    DMA.CH2.CTRLB     = 0b00010001;     // Low priority interrupt on complete
    DMA.CH2.CTRLA     = 0b10000100;     // no repeat, 1 byte burst
}

// DMA done, now at most 2 bytes are left to be sent
ISR(DMA_CH2_vect) {
    _delay_us(3);							// Wait for last byte to transfer
    setbit(LCD_CTRL, LCD_CS);               // No Select
    setbit(DMA.INTFLAGS, 0);
}
