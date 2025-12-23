#ifndef _ASMUTIL_H
#define _ASMUTIL_H

void    set_pixel(uint8_t x, uint8_t y);
void	display_or(uint8_t data);           // OR data with display buffer
void    lcd_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
uint8_t addwsat(uint8_t a, int8_t b);
uint8_t saddwsat(int8_t a, int8_t b);
void    windowCH1(uint8_t w1, uint8_t w2);
void    windowCH2(uint8_t w1, uint8_t w2);
void    slopedownCH1(unsigned char);
void    slopeupCH1(unsigned char);
void    slopedownCH2(unsigned char);
void    slopeupCH2(unsigned char);
void    trigdownCH1(unsigned char);
void    trigupCH1(unsigned char);
void    trigdownCH2(unsigned char);
void    trigupCH2(unsigned char);
void    trigdownCHD(unsigned char);
void    trigupCHD(unsigned char);

#endif
