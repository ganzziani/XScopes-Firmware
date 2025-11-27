#ifndef _MYGCCDEF_H
#define _MYGCCDEF_H

#include <stdint.h>

#define setbit(port, bit) ((port) |= (uint8_t)_BV(bit))
#define setbits(port, mask) ((port) |= (uint8_t)(mask))
#define clrbit(port, bit) ((port) &= (uint8_t)~_BV(bit))
#define clrbits(port, mask) ((port) &= (uint8_t)~(mask))
#define testbit(port, bit) (uint8_t)(((uint8_t)port & (uint8_t)_BV(bit)))
#define togglebit(port, bit) (port ^= (uint8_t)_BV(bit))
#define	hibyte(x) (uint8_t)(x>>8)
#define	lobyte(x) (uint8_t)(x&0x00FF)

#define REVERSE(a) do                     \
{                                         \
  a=((a>>1)&0x55)|((a<<1)&0xaa);          \
  a=((a>>2)&0x33)|((a<<2)&0xcc);          \
  asm volatile("swap %0":"=r"(a):"0"(a)); \
} while(0) 

// Multiply two 8bit signed numbers, return 8bit
#define FMULS8(_a,_b)                     \
    ({                                    \
        int8_t _c;                        \
        asm (                             \
        "fmuls %[a], %[b]"    "\n\t"      \
        "mov   %[c], R1"      "\n\t"      \
        "clr    __zero_reg__"             \
        : [c] "=r" (_c)                   \
        : [a] "a" ((uint8_t)(_a)),  [b] "a" ((uint8_t)(_b))); \
        _c;}) 

// Multiply two 8bit signed numbers, use rounding, return 8bit
#define FMULS8R(_a,_b)                    \
    ({                                    \
        int8_t _c;                        \
        asm (                             \
        "fmuls %[a], %[b]"    "\n\t"      \
        "sbrc  r0, 7"         "\n\t"      \
        "inc   r1"            "\n\t"      \
        "mov   %[c], R1"      "\n\t"      \
        "clr    __zero_reg__"             \
        : [c] "=r" (_c)                   \
        : [a] "a" ((uint8_t)(_a)),  [b] "a" ((uint8_t)(_b))); \
        _c;})

// Multiply two 8bit signed numbers, return 16bit
#define FMULS(_a,_b)                    \
    ({                                  \
        int16_t _c;                     \
        asm (                           \
        "fmuls %[a], %[b]"  "\n\t"      \
        "movw  %[c], R0"    "\n\t"      \
        "clr    __zero_reg__"           \
        : [c] "=r" (_c)                 \
        : [a] "a" ((uint8_t)(_a)),  [b] "a" ((uint8_t)(_b))); \
        _c;}) 

/*
// Converts a decimal number to ASCII HEX representation ('0' thru 'F')
#define NibbleToChar(_a) asm volatile ("rol %0" : "=r" (reg) : "0" (reg)) 
    ({                                  \
        asm (                           \
        "subi %[a], 0xD0"   "\n\t"      \
        "cpi  %[a], 0x3A"   "\n\t"      \
        "brlt 0f"           "\n\t"      \
        "subi %[a], 0xF9"   "\n\t"      \
        "0:"                "\n\t"      \
        : [a] "=r"                      \
        : [a] "d"                       \
        }) */

static __inline__ void NOP (void) { __asm__ volatile ( "nop    " "\n\t" ); }
static __inline__ void WDR (void) { __asm__ volatile ( "wdr    " "\n\t" ); }
static __inline__ void CLT (void) { __asm__ volatile ( "clt    " "\n\t" ); }
static __inline__ void SET (void) { __asm__ volatile ( "set    " "\n\t" ); }
static __inline__ void SLP (void) { __asm__ volatile ( "sleep  " "\n\t" ); }

// ### ROL ### Logical Rotate Left Of Register Through Carry 
#define ASM_ROL(reg) asm volatile ("rol %0" : "=r" (reg) : "0" (reg)) 

// ### ROR ### Logical Rotate Right Of Register Through Carry 
#define ASM_ROR(reg) asm volatile ("ror %0" : "=r" (reg) : "0" (reg)) 

// Swap two bytes
#define SWAP(x,y) do { (x)=(x)^(y); (y)=(x)^(y); (x)=(x)^(y); } while(0)

// 8:8 Fixed Point Math
typedef int16_t fixed;
#define int2fix(a)   (((int16_t)(a))<<8)            // Convert char to fix
#define fix2int(a)   ((int8_t)((a)>>8))             // Convert fix to char
#define float2fix(a) ((int16_t)((a)*256.0))         // Convert float to fix
#define fix2float(a) ((float)(a)/256.0)             // Convert fix to float
#define multfix(a,b) ((int16_t)((((int32_t)(a))*((int32_t)(b)))>>8)) // Multiply two fixed
#define divfix(a,b)  ((int16_t)((((int32_t)(a))<<8)/((int32_t)(b)))) // Divide two fixed

/*// 16:16 Fixed Point Math
//Type conversions
typedef int32_t fixed;
#define int2fix(a)   (((int32_t)(a))<<16)            //Convert char to fix. a is a char
#define fix2int(a)   ((int16_t)((a)>>16))    //Convert fix to char. a is an int(in fixed notation)
#define float2fix(a) ((int32_t)((a)*65536.0))         //Convert float to fix. a is a float
#define fix2float(a) ((float)(a)/65536.0)         //Convert fix to float. a is an int(in fixed notation) 
#define multfix(a,b) ((int32_t)((((int64_t)(a)) * ((int64_t)(b)) )>>16)) //multiply two fixed #
#define divfix(a,b)  ((int32_t)((((int64_t)(a))<<16)/((int64_t)(b)))) //divide two fixed #
*/

#endif
