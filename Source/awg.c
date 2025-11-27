#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "awg.h"

// Global AWG variables
uint8_t AWGBuffer[BUFFER_AWG]; // AWG Output Buffer
uint8_t cycles;         // Cycles in AWG buffer

void moveF(void) {
    uint8_t i=6;        // Start checking at 10KHz
	do {                // Find frequency range
        if(M.AWGdesiredF>=pgm_read_dword_near(Powersof10+i)) break;
    } while(i--);
    uint16_t add;
    if(i>=2) add=pgm_read_dword_near(Powersof10+i-2);
    else add=1;
	if(testbit(Misc,negative)) M.AWGdesiredF-=add;
	else if(testbit(Misc, bigfont)) M.AWGdesiredF+=add;
	else {  // Shortcuts (lower to next power of 10)
		if(i==0) i=7;   // Jump back to 10kHz
        M.AWGdesiredF=pgm_read_dword_near(Powersof10+i-1);
    }
}

// AWG Frequency = cycles * (CPU freq / Buffer size) / (Timer Period * prescale)
// AWG Frequency = cycles * 125000 / (Timer Period * prescale)
// Minimum AWG frequency with cycle=32: 61Hz
// Maximum AWG frequency with cycle=1:  3.906kHz
// Tradeoff: Frequency resolution  <-> Amplitude resolution
// Low periods have poor Freq resolution
// High cycles have poor Amp resolution
void BuildWave(void) {
    if(M.AWGamp>0)      M.AWGamp=0;         // AWGAmp must be negative
    if(M.AWGduty==0)    M.AWGduty=1;        // Zero is invalid
    uint8_t *pf = &M.AWGdesiredF;           // Get address of M.AWGdesiredF
    if(M.AWGdesiredF==0)  *pf = 1;          // Minimum Freq= 0.01Hz (only need to access lower byte of M.AWGdesiredF)
    // Maximum Frequency check done at CheckMax function
    
    // Determine number of cycles
    //  F > 15728Hz -> cycles = 32, prescale = 1   (Period range:   254 - 31) -> Max 125,173.75 (max set in MAXM struct)
    //  F >  7864Hz -> cycles = 16, prescale = 1   (Period range:   254 - 127)
    //  F >  3932Hz -> cycles =  8, prescale = 1   (Period range:   254 - 127)
    //  F >  1966Hz -> cycles =  4, prescale = 1   (Period range:   254 - 127)
    //  F >   983Hz -> cycles =  2, prescale = 1   (Period range:   254 - 127)
    //  F >  2.56Hz -> cycles =  1, prescale = 1   (Period range: 48828 - 127)
    uint8_t Fcomp8 = M.AWGdesiredF >> 16;   // Use only 8bits for comparison
    uint8_t Flevel= 1572864>>16;           // Choose 15728Hz as transition frequency (Flevel = 18h)
    for (cycles = 32; cycles > 1 && Fcomp8 < Flevel; cycles >>= 1) {
        Flevel >>= 1;
    }
    // Construct 256 bytes waveform
    
    int8_t *p=(int8_t *)T.DATA.AWGTemp1;
    uint16_t Seed;
    uint8_t i=0;
    switch(M.AWGtype) {
        case 0: // Random
            Seed = TCD1.CNT;
            do {
                Seed = 25173 * Seed + 1;
                *p++ = hibyte(Seed);
            } while(++i);
        break;
        case 1: // Sine
            do { *p++ = Sin(i+64); } while(++i);
        break;
        case 2: // Square
            setbit(Misc,bigfont);    // No interpolation
            do { *p++ = (i<128)?-127:127; } while(++i);
        break;
        case 3: // Triangle
            for(;i<128;i++) *p++ = 127-((i)<<1);
            do { *p++ = -127+(i<<1); } while(++i);
        break;
        case 4: // Exponential
            for(;i<128;i++) *p++ = -pgm_read_byte_near(Exp+i);
            do { *p++ = pgm_read_byte_near(Exp-128+i); } while(++i);
        break;
        case 5: // Custom wave from EEPROM
            eeprom_read_block(T.DATA.AWGTemp1, EEwave, BUFFER_AWG);
        break;
    }
    // Prepare output buffer:
    // ******** Duty cycle ********
    uint16_t step=0;
	uint16_t inc;
    i=0; inc=(256-M.AWGduty)<<1;
    p=(int8_t *)T.DATA.AWGTemp1;
    do {
        uint8_t j=hibyte(step);
        T.DATA.AWGTemp2[j] = *p;
        int8_t awgpoint;
        if(!testbit(Misc,bigfont)) {  // Interpolation
            int8_t k=*p++;
            awgpoint = (k+(*p))/2;
        } else awgpoint = *p++;         // No Interpolation
        if(j<255) T.DATA.AWGTemp2[j+1] = awgpoint;
        step+=inc;
        if(i==127) inc=M.AWGduty<<1;
    } while(++i);
    // 
    uint32_t Numerator, Denominator;
    Numerator = 12500000 * cycles;
    uint16_t Fcomp16 = M.AWGdesiredF >> 8;   // Use only 16bits for comparison
    PMIC.CTRL = 0x06;   // Disable low level interrupts
    if(Fcomp16<(256>>8)) {          // F < 2.56Hz
        TCD1.CTRLA = 0x06;                  // Prescaler: 256
        Denominator = M.AWGdesiredF * 256;
    }
    else {
        TCD1.CTRLA = 0x01;                  // Prescaler: 1
        Denominator = M.AWGdesiredF;
    }        
    // Avoid discontinuity when changing frequency by writing to PERBUF
    TCD1.PERBUF = (Numerator / Denominator) -1;  // Set Period
    i=0;
    do {
    // ******** Multiply by Gain ********
        uint8_t j=FMULS8(M.AWGamp,T.DATA.AWGTemp2[(uint8_t)(i*cycles)]); // Keep index < 256
    // ******** Add Offset ********
        AWGBuffer[i]=saddwsat(j,M.AWGoffset);
    } while(++i);
    clrbit(MStatus, updateawg);
    clrbit(Misc,bigfont);    // default value
    PMIC.CTRL = 0x07; // Enable all interrupts
}
