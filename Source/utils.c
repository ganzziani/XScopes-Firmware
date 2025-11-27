#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "utils.h"
#include "mygccdef.h"

// From Application Note AVR1003
void CCPWrite( volatile uint8_t * address, uint8_t value ) {
    uint8_t volatile saved_sreg = SREG;
    cli();
    #ifdef __ICCAVR__
    asm("movw r30, r16");
    #ifdef RAMPZ
    RAMPZ = 0;
    #endif
    asm("ldi  r16,  0xD8 \n"
    "out  0x34, r16  \n"
    #if (__MEMORY_MODEL__ == 1)
    "st     Z,  r17  \n");
    #elif (__MEMORY_MODEL__ == 2)
    "st     Z,  r18  \n");
    #else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
    "st     Z,  r19  \n");
    #endif /* __MEMORY_MODEL__ */

    #elif defined __GNUC__
    volatile uint8_t * tmpAddr = address;
    #ifdef RAMPZ
    RAMPZ = 0;
    #endif
    asm volatile(
    "movw r30,  %0"	      "\n\t"
    "ldi  r16,  %2"	      "\n\t"
    "out   %3, r16"	      "\n\t"
    "st     Z,  %1"       "\n\t"
    :
    : "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
    : "r16", "r30", "r31"
    );
    #endif
    SREG = saved_sreg;
}

// Converts an uint to int, then returns the half
uint8_t half(uint8_t number) {
    int8_t temp;
    temp=(int8_t)(number-128);
    temp=temp/2;
    return (uint8_t)(temp+128);
}

// Converts an uint to int, then return the double, or saturates
uint8_t twice(uint8_t number) {
    int8_t temp;
    if(number<=64) return 0;
    if(number>=192) return 255;
    temp = (int8_t)(number-128);
    temp = temp*2;
    return (uint8_t)(temp+128);
}

// Receives a nibble, returns the corresponding ascii that represents the HEX value
char NibbleToChar(uint8_t nibble) {
    nibble=nibble+'0';          // '0' thru '9'
    if(nibble>'9') nibble+=7;   // 'A' thru 'F'
    return nibble;
}
