#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include "hardware.h"

typedef struct {
    uint8_t     display_setup[3];
    uint8_t     display_data[DISPLAY_DATA_SIZE];
} Disp_data;

extern Disp_data Disp_send;
extern uint8_t u8CursorX, u8CursorY;

#define lcd_goto(x,y) do { u8CursorX=(x); u8CursorY=(y); } while(0)

/* EXTERN Function Prototype(s) */
void print3x6(const char *);
void putchar3x6 (char);
void printN3x6(uint8_t Data);
void printhex3x6(uint8_t n);           // Prints a HEX number
void clr_display(void);
void sprite(uint8_t x, uint8_t y, const int8_t *ptr);
void printV(int16_t Data, uint8_t gain, uint8_t CHCtrl);
void printF(uint8_t x, uint8_t y, int32_t Data);
void tiny_printp(uint8_t x, uint8_t y, const char *ptr);
void LcdInstructionWrite (unsigned char);

void GLCD_LcdInit(void);
void GLCD_setting(void);
void GLCD_LcdOff(void);
void dma_display(void);

#endif
