#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "display.h"
#include "mygccdef.h"
#include "main.h"
#include "mso.h"
#include "utils.h"

Disp_data Disp_send;
uint8_t   u8CursorX, u8CursorY;


// Clear display buffer
void clr_display(void) {
    uint8_t *p = Disp_send.display_data;   // Locate pointer at start of data buffer;
    uint8_t i=0;
    do {    // Erase all 1024 bytes in the buffer
        // Unroll inner loop for speed - Clear 4 lines
        *p++=0; *p++=0; *p++=0; *p++=0;
    } while(--i);
    lcd_goto(0,0);
}

void GLCD_setting(void) {
    cli();
    if(testbit(Display, flip)) {
        LcdInstructionWrite(LCD_SET_SCAN_NOR);   // direction
        LcdInstructionWrite(LCD_SET_SEG_REMAP1);
    }
    else {
        LcdInstructionWrite(LCD_SET_SCAN_FLIP);   // direction
        LcdInstructionWrite(LCD_SET_SEG_REMAP0);
    }
    if(testbit(Display, disp_inv)) LcdInstructionWrite(LCD_DISP_REV);   // invert
    else LcdInstructionWrite(LCD_DISP_NOR);   // no invert
    sei();
}

// Sprites, each byte pair represents next pixel relative position
void sprite(uint8_t x, uint8_t y, const int8_t *ptr) {
    do {
        int8_t a=pgm_read_byte(ptr++);  // Get next x
        int8_t b=pgm_read_byte(ptr++);  // Get next y
        if((uint8_t)a==255) return;     // 255 marks the end of the sprite
        set_pixel(x+a,y+b);
    } while(1);
}

// Print a char on the display using the 3x6 font
void putchar3x6(char u8Char) {
    uint16_t pointer;
	uint8_t data;
	pointer = (unsigned int)(Font3x6)+(u8Char-20)*(3);
    if(u8Char!='\n') {
        // Draw a char: 3 bytes
        for(uint8_t i=3; i; i--) {
            data = pgm_read_byte_near(pointer++);
		    if(testbit(Misc,negative)) data = ~(data|128);
		    display_or(data);
	    }
    }
    // Special characters
    if(u8Char==0x1C) {       // Begin long 'd' character
        display_or(FONT3x6_d1);
    }
    else if(u8Char==0x1D) {  // Complete long 'd' character
        display_or(FONT3x6_d2);
        u8CursorX++;
    }
    else if(u8Char==0x1A) {  // Complete long 'm' character
        display_or(FONT3x6_m);
    }
    else if(u8CursorX < 128) {  // if not then insert a space before next letter
		data = 0;
		if(testbit(Misc,negative)) data = 127;
		display_or(data);
	}
    if(u8CursorX>=128 || u8Char=='\n') {    // Next line
        u8CursorX = 0; u8CursorY++;
    }
}

// Print a string in program memory to the display
void print3x6(const char *ptr) {
    char c;
    while ((c=pgm_read_byte(ptr++)) != 0x00) {
        putchar3x6(c);
    }
}

// Print Number using  the 3x6 font
void printN3x6(uint8_t Data) {
    uint8_t d=0x30;
	while (Data>=100)	{ d++; Data-=100; }
    if(d>0x30) putchar3x6(d);
	d=0x30;
	while (Data>=10)	{ d++; Data-=10; }
    putchar3x6(d);
    putchar3x6(Data+0x30);
}

// Prints a HEX number
void printhex3x6(uint8_t n) {
    uint8_t temp;
    temp = n>>4;   putchar3x6(NibbleToChar(temp));
    temp = n&0x0F; putchar3x6(NibbleToChar(temp));
}

extern const uint16_t milivolts[];

// Print Voltage, multiply by 10 if using the x10 probe
void printV(int16_t Data, uint8_t gain, uint8_t CHCtrl) {
    int32_t Data32 = (int32_t)Data*milivolts[gain];
    if(testbit(CHCtrl,chx10)) Data32*=10;
    printF(u8CursorX,u8CursorY,Data32/8);
}    

// Print Fixed point Number with 5 digits
// or Print Long integer with 7 digits
void printF(uint8_t x, uint8_t y, int32_t Data) {
	uint8_t D[8]={0,0,0,0,0,0,0,0},point=0;
    uint8_t *DisplayPointer = &Disp_send.display_data[((uint16_t)(y<<7)) + (x)];
    lcd_goto(x,y);
    if(Data<0) {    // Negative number: Print minus
        Data=-Data;
        if(testbit(Misc,bigfont)) { // putchar10x15('-');
            for(uint8_t i=4; i; i--) {
                *DisplayPointer++ = 0x03;
            }
            DisplayPointer +=2; 
        }            
        else putchar3x6('-');
    }
    else {          // Positive number: Print space
        if(testbit(Misc,bigfont)) { // putchar10x15(' ');
            DisplayPointer+=6;
        }            
        else putchar3x6(' ');
    }
    if(testbit(Misc,negative)) {   // 7 digit display
        point=3;
    }
    else {  // 4 digit display
	    if(Data>=999900000L) Data = 9999;
	    else if(Data>=100000000L)  Data = (Data/100000);
	    else if(Data>=10000000L) {
    	    Data = (Data/10000);
    	    point = 1;
	    }
	    else if(Data>=1000000L) {
    	    Data = (Data/1000);
    	    point = 2;
	    }
	    else {
    	    Data = (Data/100);
    	    point = 3;
	    }
    }
    
    uint8_t i=7;
    do {    // Decompose number
        uint32_t power;
        power=pgm_read_dword_near(Powersof10+i);
        while(Data>=power) { D[i]++; Data-=power; }
    } while(i--);

    if(testbit(Misc, negative)) i=7;    // To print all digits
    else i=3;
	for(; i!=255; i--) {
		if(testbit(Misc,bigfont)) {
            // putchar10x15();
            uint8_t *TempPointer = DisplayPointer;
            uint16_t FontPointer = (unsigned int)(Font10x15)+(D[i])*20;
            // Upper side
            DisplayPointer-=128;
            for(uint8_t i=10; i; i--) {
                *DisplayPointer++ = pgm_read_byte_near(FontPointer++);
            }
            // Lower Side
            DisplayPointer = TempPointer;
            for(uint8_t i=10; i; i--) {
                *DisplayPointer++ = pgm_read_byte_near(FontPointer++);
            }
            DisplayPointer+=2;
			if(point==i) { // putchar10x15('.'); Small point to Save space
                *DisplayPointer++ = 0x60;
                *DisplayPointer++ = 0x60;
                DisplayPointer+=2;
            }                
		}
		else {
			putchar3x6(0x30+D[i]);
			if(point==i) putchar3x6('.');
		}
	}
}

// Print small font text at x,y from program memory
void tiny_printp(uint8_t x, uint8_t y, const char *ptr) {
    lcd_goto(x,y);
    print3x6(ptr);
}
