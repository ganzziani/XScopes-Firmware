/* ----------------------------------------------------------------------------
	128x64 Graphic LCD management for SSD1305 driver

	FILE NAME 	: SSD1305.c
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

/*-----------------------------------------------------------------------------
LCD Initialization
	GLCD_LcdINIT()
-----------------------------------------------------------------------------*/
void GLCD_LcdInit(void)	{
    // Recommended power up sequence
    clrbit(LCD_CTRL, LCD_RESET);         // Reset Low for 30 uS
  	delay_ms(100);
    setbit(LCD_CTRL, LCD_RESET);         // Reset Low for 30 uS
    delay_ms(4);
    cli();
    // Recommended initialization sequence
    LcdInstructionWrite(LCD_DISP_OFF);
    LcdInstructionWrite(LCD_SET_RATIO_OSC);
    LcdInstructionWrite(0xA0);
//    LcdInstructionWrite(LCD_MULTIPLEX);
//    LcdInstructionWrite(0x3F);
//    LcdInstructionWrite(LCD_SET_OFFSET);
//    LcdInstructionWrite(0x00);
//    LcdInstructionWrite(LCD_SET_LINE);

//set at main LcdInstructionWrite(LCD_SET_SEG_REMAP1);
//set at main LcdInstructionWrite(LCD_SET_SCAN_NOR);
    LcdInstructionWrite(LCD_SET_COL_ADDR);
    LcdInstructionWrite(2);
    LcdInstructionWrite(129);
	LcdInstructionWrite(LCD_SET_AREA_COLOR);
    LcdInstructionWrite(0x05);			    // Set Monochrome & Low Power Save Mode
//reset value    LcdInstructionWrite(LCD_SET_LOOKUP);
//reset value    LcdInstructionWrite(0x3F);
//reset value    LcdInstructionWrite(0x3F);
//reset value    LcdInstructionWrite(0x3F);
//reset value    LcdInstructionWrite(0x3F);              // Define All Banks Pulse Width as 64 Clocks*/

//reset value    LcdInstructionWrite(LCD_SET_PADS);
//reset value    LcdInstructionWrite(0x12);
    LcdInstructionWrite(LCD_SET_CONTRAST);
    LcdInstructionWrite(0xFF);
    LcdInstructionWrite(LCD_SET_CHARGE);
    LcdInstructionWrite(0x82);
    LcdInstructionWrite(LCD_SET_VCOM);
    LcdInstructionWrite(0x3C);
    LcdInstructionWrite(LCD_EON_OFF);
    LcdInstructionWrite(LCD_DISP_NOR);
    LcdInstructionWrite(LCD_MEM_ADDRESSING);
    LcdInstructionWrite(0x00);          // Horizontal Addressing mode

    LCDVOLTON();
  	delay_ms(250);
    LcdInstructionWrite(LCD_DISP_ON);
    sei();
    Disp_send.display_setup[0]=LCD_SET_PAGE;
    Disp_send.display_setup[1]=LCD_SET_COL_HI;    // Set column at 0
    Disp_send.display_setup[2]=LCD_SET_COL_LO+2;    
}

void GLCD_LcdOff(void)	{
    cli();                              // Disable interrupts
    LcdInstructionWrite(LCD_DISP_OFF);
	LCDVOLTOFF();
  	delay_ms(100);
    sei();
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

// Transfer display buffer to LCD
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
    _delay_us(3);
/*    uint8_t i=0;
    while(!testbit(USARTD0.STATUS,6)) {     // Wait until transmit done
        _delay_us(1);
        if(--i) break;
    }
    setbit(USARTD0.STATUS,6);               // Clear flag*/
    setbit(LCD_CTRL, LCD_CS);               // No Select
    setbit(DMA.INTFLAGS, 0);
}

