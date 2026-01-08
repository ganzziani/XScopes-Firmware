// TODO & Optimizations
/*      Custom bootloader:
            - Save constants tables in bootloader
            - Calibration in User Signature Row
        Detect low voltage with comparator
        Force trigger, Trigger timeout in menu
        USE NVM functions from BOOT */

// TODO Expansion boards: SD, Keyboard, Display, RS232, MIDI, Video, RAM

/* Hardware resources:
    Timers:
        RTC   Clock for menu and sleep timeout
        TCC0  Frequency counter time keeper
            Also used as split timer, source is Event CH7 (40.96mS)
            TCC0L Controls the auto trigger
            TCC0H Auto key repeat
        TCC1  Counts post trigger samples
              UART sniffer time base
              Frequency counter low 16bits
        TCD0  Split timer, source is Event CH6 (1.024ms)
            TCD0L 40.96mS period - 24.4140625 Hz - Source for Event CH7
	        TCD0H Controls LCD refresh rate
	    TCD1  Overflow used for AWG
        TCE0  Controls Interrupt ADC (srate >= 11), srate: 6, 7, 8, 9, 10
              Fixed value for slow sampling
              Frequency counter high 16bits
    Events:
	    CH0 TCE0 overflow used for ADC
	    CH1 ADCA CH0 conversion complete
        CH2 EXT Trigger or logic pin for freq. measuring
        CH3 TCD1 overflow used for DAC
        CH4 TCC0 overflow used for freq. measuring
        CH5 TCC1 overflow used for freq. measuring
        CH6 CLKPER / 32768 -> every 1.024ms
        CH7 TCD0L underflow: 40.96mS period - 24.4140625 Hz
	DMAs:
	    CH0 ADC CH0  / SPI Sniffer MOSI / UART Sniffer
	    CH1 ADC CH1  / SPI Sniffer MISO
	    CH2 Port CHD / Display
	    CH3 AWG DAC
    USART:
	    USARTD0 for Display
	    USARTC0 for Sniffer
	    USARTE0 for External Interface Port
	RAM:
	    1024:   Display buffer
         128:   Endpoint0 out + in
         128:   Endpoint1 out + in
         768:   CH1+CH2+CHD Data
         256:   AWG Buffer
          31:   M
		1536:   Temp (FFT, logic sniffer)
        -----
        3871    Total + plus some global variables
    Interrupt Levels:
        PORTC_INT1      High        SPI sniffer
        PORTC INT0      High        I2C sniffer
        TCC1            High        UART sniffer
        USARTE0 UDRE    High        Serial port ready to send
        TCE0:           Medium      Slow Sampling
        USARTE0 RXC     Medium      Serial port RX
        USB BUSEVENT    Medium      USB Bus Event
        USB_TRNCOMPL    Medium      USB Transaction Complete
        PORTA INT0:     Medium      keys
        TCC0L:          Low         auto trigger
        TCC0H:          Low         auto keys
        RTC:            Low         sleep timeout, menu timeout
        TCD0H:          Low         Wake from sleep (LCD refresh rate)
*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/fuse.h>
#include "main.h"
#include "mso.h"
#include "logic.h"
#include "awg.h"
#include "interface.h"
#include "USB\usb_xmega.h"
#include "USB\Descriptors.h"
#include "utils.h"

FUSES = {
    .FUSEBYTE1 = 0x00,  /* Watchdog Configuration */
    .FUSEBYTE2 = 0xBD,  /* Reset Configuration */
    .FUSEBYTE4 = 0xF7,  /* Start-up Configuration */
    .FUSEBYTE5 = 0xE9,  /* EESAVE and BOD Level */
};
/*  Fuses:
    WDWP = 8CLK
    WDP = 8CLK
    BOOTRST = BOOTLDR
    TOSCSEL = XTAL
    BODPD = SAMPLED
    RSTDISBL = [ ]
    SUT = 4MS
    WDLOCK = [ ]
    BODACT = CONTINUOUS
    EESAVE = [ ]
    BODLVL = 2V8    */

// Big buffer to store large but temporary data
TempData T;

// Variables that need to be saved in NVM
NVMVAR M;

// EEProm variables
uint8_t EEMEM EESleepTime = 32;     // Sleep timeout in minutes
uint8_t EEMEM EEDACgain   = 0;      // DAC gain calibration
uint8_t EEMEM EEDACoffset = 0;      // DAC offset calibration
uint8_t EEMEM EECalibrated = 0;     // Offset calibration done

//static void CalibrateDAC(void);
void SimpleADC(void);
static inline void LoadEE(void);                  // Load settings from EEPROM
void CalibrateOffset(void);
void CalibrateGain(void);

int main(void) {
    // Clock Settings
	// USB Clock
	OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc; // Configure DFLL for 48MHz, calibrated by USB SOF
	DFLLRC32M.CALB = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));
	DFLLRC32M.COMP1 = 0x1B; // Xmega AU manual, p41
	DFLLRC32M.COMP2 = 0xB7;
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;

    OSC.XOSCCTRL = 0xCB;    // Crystal type 0.4-16 MHz XTAL - 16K CLK Start Up time
    OSC.PLLCTRL = 0xC2;     // XOSC is PLL Source - 2x Factor (32MHz)
    OSC.CTRL = OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_XOSCEN_bm;
    delay_ms(2);
    // Switch to internal 2MHz if crystal fails
    if(!testbit(OSC.STATUS,OSC_XOSCRDY_bp)) {   // XT ready flag not set?
        OSC.XOSCCTRL = 0x00;    // Disable external oscillators
        OSC.PLLCTRL = 0x10;     // 2MHz is PLL Source - 16x Factor (32MHz)
    }
    OSC.CTRL = OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_PLLEN_bm | OSC_XOSCEN_bm;

    // Watchdog timer on
    CCPWrite(&WDT.CTRL, WDT_PER_8KCLK_gc | WDT_ENABLE_bm | WDT_CEN_bm);
    
    delay_ms(2);
    CCPWrite(&CLK.CTRL, CLK_SCLKSEL_PLL_gc);    // Switch to PLL clock
    // Clock OK!
    OSC.CTRL = OSC_RC32MEN_bm | OSC_PLLEN_bm | OSC_XOSCEN_bm;    // Disable internal 2MHz

    // PORTS CONFIGURATION
    PORTCFG.VPCTRLA = 0x41; // VP1 Map to PORTE, VP0 Map to PORTB
    PORTCFG.VPCTRLB = 0x32; // VP3 Map to PORTD, VP2 Map to PORTC
    // Initial value PORTA.DIR       = 0x00; // CH2, CH1, 1V, K1, K2, K3, K4, REF
    VPORT0.DIR		= 0x0B; // PORTB.DIR RES, AWG, D/C, R/W
    // Initial Value PORTB.OUT       = 0x00; //
    // Initial Value PORTC.DIR       = 0x00; // LOGIC
    VPORT3.DIR		= 0x1F; // PORTD.DIR USB, EXT, GREEN, DAT, TP, CLK, RED
    VPORT3.OUT		= 0x04; // Green LED on, PORT.OUT LCD voltage off
    VPORT1.DIR		= 0x09;	// PORTE.DIR TX, RX, RTS (input), CTS (power external board)
    VPORT1.OUT		= 0x01; // PORTE.OUT Power to external board
    PORTA.PIN1CTRL  = MENUPULL; // Pull up or pull down on pin PA1
    PORTA.PIN2CTRL  = 0x18; // Pull up on pin PA2
    PORTA.PIN3CTRL  = 0x18; // Pull up on pin PA3
    PORTA.PIN4CTRL  = 0x18; // Pull up on pin PA4
    PORTA.PIN5CTRL  = 0x07; // Digital Input Disable on pin PA5
    PORTA.PIN6CTRL  = 0x07; // Digital Input Disable on pin PA6
    PORTA.PIN7CTRL  = 0x07; // Digital Input Disable on pin PA7
    PORTA.INTCTRL   = 0x02; // PORTA will generate medium level interrupts
    PORTA.INT0MASK  = 0x1E; // PA4, PA3, PA2, PA1 will be the interrupt 0 sources
    //PORTB.PIN2CTRL	= 0x07; // Input Disable on pin PB2
    PORTC.INT0MASK  = 0x01; // PC0 (SDA) will be the interrupt 0 source
    PORTC.INT1MASK  = 0x80; // PC7 (SCK) will be the interrupt 1 source
    PORTD.PIN5CTRL  = 0x01; // Sense rising edge (Freq. counter)
    PORTE.PIN1CTRL  = 0x18; // Pull up on pin PE1 (RTS)

    // Initialize USARTD0 for OLED
    USARTD0.BAUDCTRLA = FBAUD;	// SPI clock rate for display
    USARTD0.CTRLC     = 0xC0;   // Master SPI mode,
    USARTD0.CTRLB     = 0x08;   // Enable TX
    
    // Initialize USARTE0 for External Interface Port
    USARTE0.BAUDCTRLA = 0x17;   // BSCALE = -6, BSEL = 1047
    USARTE0.BAUDCTRLB = 0xA4;   // ==> 115211 bps (~115.2kbps)
    USARTE0.CTRLC     = 0x03;   // Async, No Parity, 1 stop bit, 8 data bits
    USARTE0.CTRLB     = 0x18;   // Enable RX and TX
	USARTE0.CTRLA     = 0x20;   // Enable RX interrupt

    // Event System
    EVSYS.CH0MUX    = 0xE0;     // Event CH0 = TCE0 overflow used for ADC
    EVSYS.CH1MUX    = 0x20;     // Event CH1 = ADCA CH0 conversion complete
    //EVSYS.CH2MUX              // Event CH2 = Frequency counter source
    EVSYS.CH3MUX    = 0xD8;     // Event CH3 = TCD1 overflow used for DAC
    EVSYS.CH4MUX    = 0xC0;     // Event CH4 = TCC0 overflow used for freq. measuring
    EVSYS.CH5MUX    = 0xC8;     // Event CH5 = TCC1 overflow used for freq. measuring
    EVSYS.CH6MUX    = 0x8F;     // Event CH6 = CLKPER / 32768
    EVSYS.CH7MUX    = 0xD0;     // Event CH7 = TCD0 underflow

    // DAC
    DACB.CTRLB        = 0x01;   // CH0 auto triggered by an event
    DACB.CTRLC        = 0x11;   // Use AREFA (2.0V), data is left adjusted
    DACB.EVCTRL       = 0x03;   // Event CH3 triggers the DAC Conversion
    //DACB.CH0GAINCAL = eeprom_read_byte(&EEDACgain);      // Load DAC gain calibration
    //DACB.CH0OFFSETCAL = eeprom_read_byte(&EEDACoffset);  // Load DAC offset calibration
    DACB.CTRLA = 0x05;          // Enable DACB and CH0

    // DMA for DAC
    DMA.CH3.ADDRCTRL  = 0xD0;   // Reload after transaction, Increment source
    DMA.CH3.TRIGSRC   = 0x25;   // Trigger source is DACB CH0
    DMA.CH3.TRFCNT    = 256;    // AWG Buffer is 256 bytes
	DMA.CH3.SRCADDR0  = (((uint16_t) AWGBuffer)>>0*8) & 0xFF;
	DMA.CH3.SRCADDR1  = (((uint16_t) AWGBuffer)>>1*8) & 0xFF;
//	DMA.CH3.SRCADDR2  = 0;
	DMA.CH3.DESTADDR0 = (((uint16_t)(&DACB.CH0DATAH))>>0*8) & 0xFF;
	DMA.CH3.DESTADDR1 = (((uint16_t)(&DACB.CH0DATAH))>>1*8) & 0xFF;
//	DMA.CH3.DESTADDR2 = 0;
    DMA.CH3.CTRLA     = 0b10100100;     // Enable CH3, repeat mode, 1 byte burst, single
    DMA.CTRL          = 0x80;           // Enable DMA, single buffer, round robin

    LoadEE();        // Load Oscilloscope EE settings

    // Interrupt Configuration
    PMIC.CTRL = 0x07;       // Enable High, Medium and Low level interrupts
    sei();                  // Enable global interrupts

	USB_ResetInterface();

    // Initialize LCD
    GLCD_LcdInit();
    GLCD_setting();

    // RTC Clock Settings
    /* Set 1.024kHz from internal 32.768kHz RC oscillator */
    uint8_t i;
    CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;
    i=eeprom_read_byte(&EECalibrated);  // Check if the device has been calibrated
    #ifndef NODISPLAY
    if(i==0) setbit(Misc,bigfont);      // Use bigfont bit to enter settings
    #else
    if(i==0) CalibrateOffset();
    #endif
    i=eeprom_read_byte(&EESleepTime);
    // Check if K2 pressed
    if(!testbit(PORTA.IN,3)) setbit(Misc,bigfont);   // Use bigfont bit to enter settings
    Buttons=0;  // Without this, you need to press KD before power up
    while(testbit(Misc, bigfont)) {
        uint8_t Volt;
        clr_display();
        ADCA.CTRLA        = 0x01;   // Enable ADC
        ADCA.CH3.MUXCTRL  = 0x28;   // PA5 (Connected to 2.048V / 2)
        ADCA.PRESCALER    = 0x07;   // Prescaler 512 (62.5kHZ ADC clock)
        ADCA.CTRLB        = 0x14;   // signed mode, no free run, 8 bit
        // 2.048V measure
        ADCA.REFCTRL      = 0x10;   // REF= VCC/1.6 (~2.0625V)
        ADCA.CH3.CTRL     = 0x81;   // Start conversion, single ended input
        delay_ms(1);
        Volt = ADCA.CH3.RESL;
        printF(64,4,(uint32_t)Volt*3248);     // Convert to volts (V * 2.0625*2/127)
//        send(VPORT2.IN);            // Send logic port data to serial port
        memcpy_P(Disp_send.display_data+30,  &Logo, 69);   // Print Gabotronics, top center
        tiny_printp(50,1,VERSION);
        if(CLK.CTRL & CLK_SCLKSEL_RC32M_gc) {   // Clock error?
            tiny_printp(0,7,PSTR("NO XT"));
        }
        tiny_printp(0,6,PSTR("OFFSET       SLEEP:      RESTORE"));
        u8CursorX=58;   // Already have a new line from previous print
        if(i) printN3x6(i);    // Sleep timeout
        else putchar3x6('-');
        if(testbit(Buttons, userinput)) {
            clrbit(Buttons, userinput);
            if(testbit(Buttons,K1)) CalibrateOffset();
            if(testbit(Buttons,K2)) i+=16;
            if(testbit(Buttons,K3)) Restore();
            if(testbit(Buttons,KM)) {
                eeprom_write_byte(&EESleepTime, i);
                clrbit(Misc,bigfont);   // set bit to normal value
            }
        }
        dma_display(); WaitDisplay();
    }
    Buttons=0;
    while(RTC.STATUS & RTC_SYNCBUSY_bm);  // Wait for RTC / Main clock sync
    RTC.PER = (uint16_t)(i)*60;
    RTC.COMP = 60;              // 1 Minute for Menu timeout
    if(i) {
        RTC.INTCTRL = 0x05;     // Generate low level interrupts (Compare and Overflow)
        RTC.CTRL = 0x07;        // Divisor 1024 (1 second per count)
    }
    for(;;) {
        MSO();              // go to MSO
    }        
    return 0;
}

// Waits for the DMA to complete (the DMA's ISR will SET LCD_CS)
void WaitDisplay(void) {
    uint8_t n=0;
    WDR();
    while(!testbit(LCD_CTRL,LCD_CS)) {   // Wait for transfer complete
        _delay_us(8);
        n++;
        if(n==0) break;     // timeout ~ 2.048mS
    }
    OFFGRN();
    OFFRED();    
}

// Tactile Switches - This is configured as medium level interrupt
ISR(PORTA_INT0_vect) {
    uint8_t in,j=0;
    // Debounce: need to read 10 consecutive equal numbers
    OFFGRN();   // Avoid having the LED on during this interrupt
    OFFRED();
    for(uint8_t i=25; i>0; i--) {
        delay_ms(1);
		in = PORTA.IN & 0x1E;       // Read port
		if(j!=in) { j=in; i=10; }   // Value changed
	}
    Buttons=0;
    if(!testbit(in,1)) setbit(Buttons, KM); // Menu key
    #ifdef PORTABLE
        if(testbit(Display, flip)) {
            if(!testbit(in,4)) setbit(Buttons,K1);
            if(!testbit(in,2)) setbit(Buttons,K3);
        }
        else {
            if(!testbit(in,4)) setbit(Buttons,K3);
            if(!testbit(in,2)) setbit(Buttons,K1);
        }
    #else
        if(testbit(Display, flip)) {
            if(!testbit(in,4)) setbit(Buttons,K3);
            if(!testbit(in,2)) setbit(Buttons,K1);
        }
        else {
            if(!testbit(in,4)) setbit(Buttons,K1);
            if(!testbit(in,2)) setbit(Buttons,K3);
        }
    #endif
    if(!testbit(in,3)) setbit(Buttons,K2);
    if(Buttons) {
        setbit(MStatus, update);         // Valid key
        setbit(Buttons, userinput);
        // TCC0H used for auto repeat key
        if(TCC0.CTRLE!=0) {             // Not doing a frequency count
            TCC0.CNTH = 20;                             // Restart timer - 819.2mS timeout
            setbit(TCC0.INTFLAGS, TC2_HUNFIF_bp);       // Clear trigger timeout interrupt
            TCC0.INTCTRLA |= TC2_HUNFINTLVL_LO_gc;      // Enable Auto Key interrupt
        }
    } else {    // No keys pressed -> disable auto repeat key interrupt
        TCC0.INTCTRLA &= ~TC2_HUNFINTLVL_LO_gc;         // Disable Auto Key interrupt
        clrbit(Misc,keyrep);
    }
    RTC.CNT=0;  // Clear screen saver timer
}

// Set auto repeat key flag
ISR(TCC2_HUNF_vect) {
    TCC0.INTCTRLA &= ~TC2_HUNFINTLVL_LO_gc;         // Disable Auto Key interrupt
    if(Buttons) setbit(Misc,keyrep);
}

// Read out calibration byte.
uint8_t ReadCalibrationByte(uint8_t location) {
    uint8_t result;
    // Load the NVM Command register to read the calibration row.
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(location);
    // Clean up NVM Command register.
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    return result;
}

// Calibrate offset, inputs must be connected to ground
void CalibrateOffset(void) {
    uint8_t i;
    #ifndef NODISPLAY
        Buttons=0;
        clr_display();
        print3x6(PSTR("DISCONNECT CH1,CH2"));
        tiny_printp(116,7,PSTR("GO"));
        dma_display();
        while(!Buttons);
    #else
        setbit(Buttons,K3);
    #endif
    if(testbit(Buttons,K3)) {
        uint8_t s=0;
        int16_t avrg1, avrg2;
        clr_display();
	    for(Srate=0; Srate<8; Srate++) {	// Cycle thru first 8 srates
            i=6; do {                       // Cycle thru all the gains
                int8_t  *q1, *q2;  // temp pointers to signed 8 bits
                s++;
                M.CH1gain=i;
                M.CH2gain=i;
                SimpleADC();
                q1=T.IN.CH1;
                q2=T.IN.CH2;
                // Calculate offset for CH1
                avrg1=0;
                avrg2=0;
                uint8_t j=0;
                do {
            	    avrg1+= (*q1++);
            	    avrg2+= (*q2++);
                } while(++j);
                int8_t avrg8=avrg1>>8;
                ONGRN();
                eeprom_write_byte((uint8_t *)&offset8CH1[Srate][i], avrg8);
                j = 32+avrg8; // add 32 to center on screen
                if(j<64) lcd_line(s,32,s,j);
                else ONRED();
                if(Srate) {
                    avrg8=avrg2>>8;
                    eeprom_write_byte((uint8_t *)&offset8CH2[Srate-1][i], avrg8);
                    j = 32+avrg8; // add 32 to center on screen
                    if(j<64) lcd_line(s+64,32,s+64,j);
                    else ONRED();
                }
                dma_display(); WaitDisplay();
            } while(i--);
        }
        // Calculate offset for Meter in VDC
        avrg1=0;
        avrg2=0;
        ADCA.CTRLB = 0x10;          // signed mode, no free run, 12 bit right adjusted
        ADCA.PRESCALER = 0x07;      // Prescaler 512 (500kHZ ADC clock)
        i=0;
        do {
            ADCA.CH0.CTRL     = 0x83;   // Start conversion, Differential input with gain
            ADCA.CH1.CTRL     = 0x83;   // Start conversion, Differential input with gain
            delay_ms(1);
            avrg1+= (int16_t)ADCA.CH0.RES;  // Measuring 0V, should not overflow 16 bits
            avrg2+= (int16_t)ADCA.CH1.RES;  // Measuring 0V, should not overflow 16 bits
        } while(++i);
        eeprom_write_word((uint16_t *)&offset16CH1, avrg1/*+0x08*/);
        eeprom_write_word((uint16_t *)&offset16CH2, avrg2/*+0x08*/);
    }
    Buttons=0;
    eeprom_write_byte(&EECalibrated, 1);    // Calibration complete!
}

// Calibrate gain, inputs must be connected to 4.000V
void CalibrateGain(void) {
    #ifndef NODISPLAY
        Buttons=0;
        clr_display();
        print3x6(PSTR("NOW CONNECT 4.000V"));
        tiny_printp(116,7,PSTR("GO"));
        dma_display();
        while(!Buttons);
    #else
        setbit(Buttons,K3);
    #endif
    if(testbit(Buttons,K3)) {
        int16_t offset;
        int32_t avrg1=0, avrg2=0;
        clr_display();
        // Calculate offset for Meter in VDC
        ADCA.CTRLB = 0x90;          // signed mode, no free run, 12 bit right adjusted
        ADCA.PRESCALER = 0x07;      // Prescaler 512 (500kHZ ADC clock)
        uint8_t i=0;
        do {
            ADCA.CH0.CTRL     = 0x83;   // Start conversion, Differential input with gain
            ADCA.CH1.CTRL     = 0x83;   // Start conversion, Differential input with gain
            delay_ms(1);
            avrg1-= (int16_t)ADCA.CH0.RES;
            avrg2-= (int16_t)ADCA.CH1.RES;
        } while(++i);
        // Vcal = 4V
        // Amp gain = 0.18
        // ADC Reference = 1V
        // 12 bit signed ADC -> Max = 2047
        // ADC cal = 4*.18*2047*256 = 377303
        // ADCcal = ADCmeas * (2048+cal)/2048
		offset=(int16_t)eeprom_read_word((uint16_t *)&offset16CH1);      // CH1 Offset Calibration
		avrg1+=offset;
        avrg1 = (377303*2048l-avrg1*2048)/avrg1;
        eeprom_write_byte((uint8_t *)&gain8CH1, avrg1);
		offset=(int16_t)eeprom_read_word((uint16_t *)&offset16CH2);      // CH2 Offset Calibration
		avrg2+=offset;
        eeprom_write_byte((uint8_t *)&gain8CH2, avrg2);
    }
    Buttons=0;
}

// Fill up channel data buffers
void SimpleADC(void) {
	Apply();
	delay_ms(64);
    StartDMAs();
	delay_ms(16);
    ADCA.CTRLB = 0x14;          // Stop free run of ADC (signed mode, no free run, 8 bit)
    // Disable DMAs
    clrbit(DMA.CH0.CTRLA, 7);
    clrbit(DMA.CH2.CTRLA, 7);
    clrbit(DMA.CH1.CTRLA, 7);
}

/*
// Calibrate DAC gain and offset, connect AWG to CH1
// Adjust with rotary encoders
static void CalibrateDAC(void) {
    uint8_t i, step=0, data, average;
    uint8_t test, bestoffset, bestgain, bestmeasure1;
    uint16_t sum, bestmeasure2;
    clr_display();

    ADCA.CH0.CTRL = 0x03 | (6<<2);       // Set gain 6
    CH1.offset=(signed char)eeprom_read_byte(&offsetsCH1[6]);

    AWGAmp=127;         // Amplitude range: [0,127]
    AWGtype=1;          // Waveform type
    AWGduty=256;        // Duty cycle range: [0,512]
    AWGOffset=0;        // 0V offset
    desiredF = 100000;  // 1kHz
    BuildWave();
    while(step<7) {
        while(!testbit(TCD0.INTFLAGS, TC1_OVFIF_bp));   // wait for refresh timeout
        setbit(TCD0.INTFLAGS, TC1_OVFIF_bp);
        // Acquire data

        // Display waveform
        i=0; sum=0;
        do {
            data=addwsat(CH1.data[i],CH1.offset);
            sum+=data;
            set_pixel(i>>1, data>>2);    // CH1
        } while(++i);
        average=(uint8_t)(sum>>8);

        switch(step) {
            case 0: // Connect AWG to CH1
                tiny_printp(0,0,PSTR("AWG Calibration Connect AWG CH1 Press 5 to start"));
                step++;
            break;
            case 1:
                if(key) {
                    if(key==KC) step++;
                    else step=7;         // Did not press 5 -> exit
                }
            break;
            case 2: // Output 0V from AWG
                AWGAmp=1;         // Amplitude range: [0,127]
                AWGtype=1;        // Waveform type
                BuildWave();
                tiny_printp(0,3,PSTR("Adjusting offset"));
                // ADS931 power, output enable, CH gains
//                PORTE.OUT = 0;
                CH1.offset=(signed char)eeprom_read_byte(&offsetsCH1[0]);
                step++;
                bestoffset = 0;
                test = 0;
                bestmeasure1=0;
                DACB.OFFSETCAL = 0;
            break;
            case 3: // Adjust Offset
                if(abs((int16_t)average-128)<abs((int16_t)bestmeasure1-128)) {    // Current value is better
                    bestoffset = test;
                    bestmeasure1=average;
                    lcd_goto(0,4);
                    if(bestoffset>=0x40) printN(0x40-bestoffset);
                    else printN(bestoffset);
                }
                lcd_line(0,bestmeasure1>>1,127,bestmeasure1>>1);
                test++;
                DACB.OFFSETCAL = test;
                if(test>=128) {
                    step++;
                    DACB.OFFSETCAL = bestoffset;   // Load DACA offset calibration
                }
            break;
            case 4: // Output -1.75V from AWG
                AWGAmp=0;           // Full Amplitude
                AWGtype=1;          // Waveform type
                AWGOffset=112;      // Offset = -1.75
                BuildWave();
                tiny_printp(0,5,PSTR("Adjusting gain"));
//                PORTE.OUT = 4;  // 0.5V / div
                CH1.offset=(signed char)eeprom_read_byte(&offsetsCH1[4]);
                step++;
                bestgain = 0;
                test=0;
                bestmeasure2=0;
                DACB.GAINCAL = 0;
            break;
            case 5: // Adjust gain
                // (1.75/0.5)*32+128)*256 = 61440
                if(abs((int32_t)sum-61696)<abs((int32_t)bestmeasure2-61696)) {    // Current value is better
                    bestgain = test;
                    bestmeasure2=sum;
                    lcd_goto(0,6);
                    if(bestgain>=0x40) printN(0x40-bestgain);
                    else printN(bestgain);
                }
                test++;
                DACB.GAINCAL = test;
                if(test>=128) {
                    step++;
                    DACB.GAINCAL = bestgain;
                }
            break;
            case 6: // Calibration complete
                // Save calibration results
                AWGAmp=0;
                eeprom_write_byte(&EEDACoffset, bestoffset);    // Save offset calibration
                eeprom_write_byte(&EEDACgain, bestgain);        // Save gain calibration
                tiny_printp(0,15,PSTR("Cal complete"));
                step++;
            break;
        }
    }
    // Restore Waveform
    LoadAWGvars();              // Load AWG settings
    BuildWave();                // Construct AWG waveform
}*/

extern const NVMVAR FLM;

// Restore default settings from flash memory
void Restore(void) {
    memcpy_P(0,  &FLGPIO, 12);              // Load Oscilloscope Settings into GPIO
    memcpy_P(&M, &FLM, sizeof(NVMVAR));     // Load Oscilloscope Settings into NVMVAR
    ONGRN();
    SaveEE();
    OFFGRN();
    Buttons=0;
}

extern NVMVAR EEMEM EEM;

// Load Oscilloscope settings from EEPROM
static inline void LoadEE(void) {
    eeprom_read_block(0, &EEGPIO, 12);
    eeprom_read_block(&M, &EEM, sizeof(NVMVAR));
}

// RTC clock, this function is called when the sleep timeout has been reached
ISR(RTC_OVF_vect) {
    PowerDown();
}

// RTC Compare, Menu timeout
ISR(RTC_COMP_vect) {
    if(!Buttons) Menu=Mdefault; // Go to default menu if no key is being pressed
}

void PowerDown(void) {
	USB.CTRLB = 0;          // USB Disattach
	USB.ADDR = 0;
	USB.CTRLA = 0;
    OFFGRN();               // In case the LED was on
    SaveEE();               // Save MSO settings
    GLCD_LcdOff();
    while(Buttons);
	clrbit(VPORT1.OUT,0);   // Power up clear
	#ifdef XMINIPORTABLE    // Portable version just powers off
	while(1);
	#else
    SLEEP.CTRL = SLEEP_SMODE_PDOWN_gc | SLEEP_SEN_bm;
    asm("sleep");
    SLEEP.CTRL = 0x00;
    GLCD_LcdInit();
	setbit(VPORT1.OUT,0);   // Power up
	Buttons=0;
	USB_ResetInterface();
	#endif
}

// Wake up from sleep
ISR(TCD2_HUNF_vect) {
    SLEEP.CTRL = 0x00;
}

void delay_ms(uint8_t n) {
    WDR();  // Clear watchdog
    for(;n;n--) _delay_ms(1);
}
