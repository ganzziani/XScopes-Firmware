#include <stdint.h>
#include <avr/interrupt.h>
#include "logic.h"
#include "main.h"
#include "mso.h"
#include "USB\usb_xmega.h"
#include "utils.h"

void ConfigLogicDMA(void);
void DisplayData(uint8_t side, uint8_t page);

//    0   1   2   3   4   5   6   7
//  |...|...|...|...|...|...|...|...|
//   ^
//   |
//   sample bit 3 times
//
// Display the HEX value of the digital stream
void HEXSerial(void) {
    uint8_t start,end;
    uint16_t increment;
    // The vertical cursors define the section to decode
    start = M.VcursorA;
    end   = M.VcursorB;
    // If the vertical cursors are off, use the entire screen
    if(!testbit(Mcursors, cursorv)) {
        start = 0;
        end   = 127;
    }
    increment = (uint16_t)(end-start);
	increment<<=3;
    if(Srate>=11) {
        increment*=2;
        start=start<<1;
    }
    uint8_t result;							// Does not need to be initialized (all bits are shifted in)
    uint8_t data,bit=0,yPos,mask;
    yPos = M.CHDpos>>3;
    for(mask=0x80; mask; mask=mask>>1) {	// Loop 8 digital channels
        uint16_t index = (uint16_t)start<<8;
        if(Srate<11) index+=(uint16_t)M.HPos<<8;
        index+=increment;
        if(CHDmask&mask) {					// Is this channel on?
            for(uint8_t i=1; i<=32; i++) {	// Scan buffer
                data = T.SCOPE.DC.CHDdata[hibyte(index)];
                if(!(i&0x03)) {				// Time to check bits when i is multiple of 4
                    result<<=1;
                    if(bit) result+=1;
                    bit=0;
                }
                else {
                    if(data&mask) bit++;
                }
                index+=increment;
            }
            lcd_goto(120,yPos);				// Print on right edge of the screen
            printhex3x6(result);
            yPos++;
        }
    }
}

const uint8_t BAUDA[8] PROGMEM = {
    3,      // 1200
    1,      // 2400
    253,    // 4800
    245,    // 9600
    229,    // 19200
    197,    // 38400
    110,    // 57600
    23,      // 115200
};

const uint8_t BAUDB[8] PROGMEM = {
    253,    // 1200
    237,    // 2400
    220,    // 4800
    204,    // 9600
    188,    // 19200
    172,    // 38400
    168,    // 57600
    164     // 115200
};

const uint16_t TCC1val[8] PROGMEM = {
    26665,  // 1200
    13332,  // 2400
    6665,   // 4800
    3332,   // 9600
    1666,   // 19200
    832,    // 38400
    554,    // 57600
    277,    // 115200
};

void ConfigLogicDMA(void) {
    setbit(DMA.CH0.CTRLA,6);    // reset DMA CH0
    setbit(DMA.CH1.CTRLA,6);    // reset DMA CH1
    // Flush receive buffers
    while(testbit(USARTC0.STATUS,7)) {
        (void)USARTC0.DATA;             // Dummy read to flush buffer
    }
    if(SPIC.STATUS) (void)SPIC.DATA;	// dummy read

    uint8_t src_addr_lo;
    uint8_t src_addr_hi;
    uint8_t trig_src = 0;

    // Configure CH0 for RX data
    DMA.CH0.ADDRCTRL  = 0b10000101;     // reload source addr after each burst, incr dest, reload dest end block
    DMA.CH0.TRFCNT    = BUFFER_SERIAL;            // buffer size

    // Set destination address for CH0 RX data (ensure proper casting)
    DMA.CH0.DESTADDR0 = (((uint16_t) T.LOGIC.data.Serial.RX)     ) & 0xFF;
    DMA.CH0.DESTADDR1 = (((uint16_t) T.LOGIC.data.Serial.RX)>>1*8) & 0xFF;

    // Configure based on protocol type (RS232 or SPI)
    if(M.CHDdecode == rs232) {
        trig_src   = 0x4B;               // USARTC0 trigger source
        src_addr_lo = (((uint16_t)(&USARTC0.DATA))     ) & 0xFF;
        src_addr_hi = (((uint16_t)(&USARTC0.DATA))>>1*8) & 0xFF;
    }
    else if(M.CHDdecode == spi) {
        trig_src   = 0x4A;               // SPIC trigger source
        src_addr_lo = (((uint16_t)(&SPIC.DATA))     ) & 0xFF;
        src_addr_hi = (((uint16_t)(&SPIC.DATA))>>1*8) & 0xFF;
    }
    DMA.CH0.TRIGSRC   = trig_src;
    DMA.CH0.SRCADDR0  = src_addr_lo;
    DMA.CH0.SRCADDR1  = src_addr_hi;

    // Set CH0 control register based on single sniff mode
    if(testbit(Mcursors, singlesniff)) {
        DMA.CH0.CTRLA     = 0b10000100;   // no repeat, 1 byte burst
    } else {
        DMA.CH0.CTRLA     = 0b10100100;   // repeat, 1 byte burst
    }

    // For UART and SPI Sniffer, bit-bang / DMA; for I2C the decoder's bytes
    DMA.CH1.ADDRCTRL  = 0b10000101;     // reload source addr after each burst, incr dest, reload dest end block
    //DMA.CH1.TRIGSRC   = 0x00;               // Manual
    if(M.CHDdecode == i2c) {
        DMA.CH1.TRFCNT    = BUFFER_I2C;
        DMA.CH1.DESTADDR0 = (((uint16_t) T.LOGIC.data.I2C.decoded)     ) & 0xFF;
        DMA.CH1.DESTADDR1 = (((uint16_t) T.LOGIC.data.I2C.decoded)>>1*8) & 0xFF;
        DMA.CH1.SRCADDR0  = (((uint16_t)(&T.LOGIC.i2c_byte))     ) & 0xFF;
        DMA.CH1.SRCADDR1  = (((uint16_t)(&T.LOGIC.i2c_byte))>>1*8) & 0xFF;
    }
    else {
        DMA.CH1.TRFCNT    = BUFFER_SERIAL;            // buffer size
        DMA.CH1.DESTADDR0 = (((uint16_t) T.LOGIC.data.Serial.TX)     ) & 0xFF;
        DMA.CH1.DESTADDR1 = (((uint16_t) T.LOGIC.data.Serial.TX)>>1*8) & 0xFF;
        DMA.CH1.SRCADDR0  = (((uint16_t)(&T.LOGIC.addr_ack_pos))     ) & 0xFF;
        DMA.CH1.SRCADDR1  = (((uint16_t)(&T.LOGIC.addr_ack_pos))>>1*8) & 0xFF;
    }

    if(testbit(Mcursors, singlesniff)) DMA.CH1.CTRLA     = 0b10000100;    // no repeat, 1 byte burst
    else DMA.CH1.CTRLA     = 0b10100100;                            // repeat, 1 byte burst
}

// Protocol Sniffer
void Sniff(void) {
    uint8_t page=0; // 16 pages to display 1024 bytes of data
                    // one page has 8 lines, each line will display
                    // 8 bytes in this format: "FF+ "
    clr_display();
    uint8_t *p=T.LOGIC.data.decoded;
    for(uint16_t i=0; i<sizeof(T.LOGIC.data)+2*sizeof(uint16_t); i++) *p++=0;  // Erase the data union and both uint16_t indices
    if(testbit(Trigger, round)) page=0x0F;  // Go to last page
    // Setup
    PR.PRPC  = 0x00;        // Enable PORTC peripherals
    RTC.INTCTRL = 0x01;     // Disable Menu Timeout interrupt
    uint8_t spictrl=0;
    if(M.CHDdecode==i2c) {
        print3x6(PSTR("I2C SNIFFER\nBIT0:SDA BIT1:SCL"));
        T.LOGIC.addr_ack_ptr =  T.LOGIC.data.I2C.addr_ack;
        T.LOGIC.addr_ack_pos = 0x80;      // counter for keeping track of all bits
        T.LOGIC.i2c_acc = 1; T.LOGIC.i2c_first = 1;       // as after a START
        PORTC.PIN0CTRL = (PORTC.PIN0CTRL & 0b11111000) | 0b00000010;    // Sense falling edge of SDA
        PORTC.PIN1CTRL = (PORTC.PIN1CTRL & 0b11111000) | 0b00000001;    // Sense rising edge of SCL
        PORTC.INT0MASK = 0x02;  // INT0: SCL, a bit
        PORTC.INT1MASK = 0x01;  // INT1: SDA, a START
    }
    else if(M.CHDdecode==rs232) {
        print3x6(PSTR("UART SNIFFER\nBIT2:RX BIT3:TX"));
        // Initialize USART for RX
        uint8_t baud, databits;
        baud=Sniffer&0x07;
        USARTC0.BAUDCTRLA = pgm_read_byte_near(BAUDA+baud);
        USARTC0.BAUDCTRLB = pgm_read_byte_near(BAUDB+baud);
        databits=(Sniffer&0x18)>>3;
        T.LOGIC.databits=databits+5;
        if(testbit(Sniffer,parmode)) {     // Check parity
            if(testbit(Sniffer,parity)) databits |= 0x30;
            else databits |= 0x20;
        }
        if(testbit(Sniffer,stopbit)) databits |= 0x08;    // Second stop bit
        USARTC0.CTRLC = databits;
                
        USARTC0.CTRLB = 0x10;       // Enable RX
        T.LOGIC.baud = (uint16_t)pgm_read_word_near(TCC1val+baud);
        TCC1.PER = T.LOGIC.baud;        // One bit period
        TCC1.CCA = T.LOGIC.baud/2;      // Compare A: middle of the bit
        PORTC.PIN3CTRL &= 0b11111000;   // TX: sense both edges (Apply() set rising)
        EVSYS.CH2MUX = EVSYS_CHMUX_PORTC_PIN3_gc;   // CH2 is free outside the frequency counter
        TCC1.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc; // Every TX edge restarts the bit timer
        TCC1.CTRLA = 0x01;  // DIV1
        TCC1.INTCTRLB = 0x03;           // Enable TCC1 CCA interrupt, High Priority
    }
    else if(M.CHDdecode==spi) {
        uint8_t inv = testbit(CHDctrl,digchinvert) ? PORT_INVEN_bm : 0;
        uint8_t pin7ctrl=inv+1;   // Sense rising edge
        print3x6(PSTR("SPI SNIFFER\nBIT4:/SS\nBIT5:MOSI\nBIT6:MISO\nBIT7:SCK"));
        if(testbit(Sniffer,CPOL)) {    // CPOL == 1
            pin7ctrl ^= PORT_INVEN_bm;    // Invert SCK
        }
        if(testbit(Sniffer,CPHA)) {
            spictrl=0x04;  // CPHA == 1
        }
        PORTC.PIN7CTRL = pin7ctrl;
        if(testbit(Sniffer,SSINV)) {    // Invert Slave select
            PORTC.PIN4CTRL = PORT_INVEN_bm^inv;
        }
    }
    ConfigLogicDMA();
    if(M.CHDdecode==i2c) {
        PORTC.INTFLAGS = 0x03;  // Clear flags
        PORTC.INTCTRL   = 0x0F; // Enable PortC INT 0 and INT 1, High Priority
    }
    dma_display(); WaitDisplay();
    uint8_t armed=0;
    while((T.LOGIC.indrx==0) && (T.LOGIC.indtx==0)) {
        WDR();
        if(M.CHDdecode!=i2c) {
            T.LOGIC.indrx=BUFFER_SERIAL-DMA.CH0.TRFCNT;
            T.LOGIC.indtx=BUFFER_SERIAL-DMA.CH1.TRFCNT;            
        }            
        else T.LOGIC.indrx=BUFFER_I2C-DMA.CH1.TRFCNT;
        // Arm once, while SS is not asserted
        if(M.CHDdecode==spi && !armed) {
            if(testbit(VPORT2.IN, 4)) {     // SS not asserted
                SPIC.CTRL = 0b01000000 + spictrl;       // Enable SPI, Slave mode
                PORTC.INTFLAGS = 0x03;      // Clear flags
                PORTC.INTCTRL   = 0x0C;     // Enable PortC INT 1, High Priority
                armed=1;
            }                
        }
        if(!testbit(MStatus,gosniffer)) goto exitSniffer;
        if (testbit(Buttons,KM)) clrbit(MStatus,gosniffer);
    }
////////////////////////////////////////////////////////////////////////////////////
// DISPLAY
////////////////////////////////////////////////////////////////////////////////////
    uint16_t i=0;
    uint8_t stopped=0;              // MSO() cleared MStatus.stop before calling Sniff()
    do {
        dma_display(); WaitDisplay();
        clr_display();
        OFFGRN();
		// USB - Send new data if previous transfer complete
		if((endpoints[1].in.STATUS & USB_EP_TRNCOMPL0_bm)) {
    		endpoints[1].in.AUXDATA = 0;				// New transfer must clear AUXDATA
            if(!testbit(Misc,sacquired)) {
               	endpoints[1].in.DATAPTR = (uint16_t)T.LOGIC.data.decoded;
        		endpoints[1].in.CNT = 640;	    // Send first half
            }
            else {
               	endpoints[1].in.DATAPTR = (uint16_t)T.LOGIC.data.decoded+640;
        		endpoints[1].in.CNT = 649;	    // Send second half
            }
    		endpoints[1].in.STATUS &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm | USB_EP_OVF_bm);
            togglebit(Misc,sacquired);
		}
        // Display I2C data
        if(M.CHDdecode==i2c) {
            uint16_t n=BUFFER_I2C-DMA.CH1.TRFCNT;
            if(n!=T.LOGIC.indrx) ONGRN();       // activity (WaitDisplay() turns it off)
            T.LOGIC.indrx=n;
            if(testbit(Mcursors, singlesniff) && !testbit(DMA.CH1.CTRLA, 7)) {  // full
                T.LOGIC.indrx=BUFFER_I2C;
                setbit(MStatus, stop);
            }
            if(testbit(Trigger, round)) {
                if(T.LOGIC.indrx>=DATA_IN_PAGE_I2C) {
                    i=T.LOGIC.indrx-DATA_IN_PAGE_I2C;
                }
                else {
                    i=(BUFFER_I2C-DATA_IN_PAGE_I2C)+T.LOGIC.indrx;
                }
                i+=(uint16_t)(page+1)*DATA_IN_PAGE_I2C;
                if(i>=BUFFER_I2C) i-=BUFFER_I2C;
            }
            else {
                i=(uint16_t)page*DATA_IN_PAGE_I2C;
            }
            for(uint8_t n=0; n<DATA_IN_PAGE_I2C; n++) {
                if(!testbit(Trigger, round)) {
                    if(i>=T.LOGIC.indrx) break;
                }
                uint8_t shift;
                shift = (i&0x0003)*2;
                uint8_t data = T.LOGIC.data.I2C.decoded[i];
                uint8_t addrdata = T.LOGIC.data.I2C.addr_ack[i/4];
                uint8_t ack = (addrdata<<(shift+1))&0x80;
                addrdata= (addrdata<<shift)&0x80;
                if(addrdata) {  // Address
                    printhex3x6(data>>1);
                    if(testbit(data,0)) {   // Read
                        if(ack) putchar3x6('<');  // Ack
                        else    putchar3x6('(');  // No Ack
                    }
                    else {                  // Write
                        if(ack) putchar3x6('>');  // Ack
                        else    putchar3x6(')');  // No Ack
                    }
                }
                else {          // Data
                    printhex3x6(data);
                    if(ack) putchar3x6('+');  // Ack
                    else    putchar3x6('-');  // No Ack
                }
                putchar3x6(' ');
                i++;
                if(i>=BUFFER_I2C) i=0;
            }
        }
        // Display UART or SPI data
        else {
            T.LOGIC.indrx=BUFFER_SERIAL-DMA.CH0.TRFCNT;
            T.LOGIC.indtx=BUFFER_SERIAL-DMA.CH1.TRFCNT;
            if(testbit(Mcursors, singlesniff)) {
                if(!testbit(DMA.CH0.CTRLA, 7)) {    // Hardware decoding complete
                    DMA.CH0.TRFCNT    = 0;
                    USARTC0.CTRLB = 0x00;           // Disable RX
                    clrbit(SPIC.CTRL,6);            // Disable SPI
                }
                if(!testbit(DMA.CH1.CTRLA, 7)) {    // Software decoding complete
                    DMA.CH1.TRFCNT    = 0;
                    setbit(MStatus, stop);
                }
            }
            if(testbit(MStatus, stop)) {
                set_line(63,8,63,30);
                const char *c=STR_STOP;
                for(uint8_t y=TEXT_LAST_LINE-3; y<=TEXT_LAST_LINE; y++) {   // Print STOP vertically
                    lcd_goto(62,y); putchar3x6(pgm_read_byte(c++));
                }
            }
            else set_line(63,8,63,DISPLAY_MAX_Y);
            uint8_t printpage=page;
            if(testbit(CHDctrl,ascii)) printpage&=0xFE;     // Only display even pages
            lcd_goto(62,0); putchar3x6(NibbleToChar(printpage));
            if(M.CHDdecode==spi) {
                lcd_goto(59,1); putchar3x6('I'); u8CursorX=65; putchar3x6('O');
            }
            else {
                lcd_goto(59,1); putchar3x6('R'); u8CursorX=65; putchar3x6('T');
            }
            DisplayData(0,page); // Display RX or SDI data
            DisplayData(1,page); // Display TX or SDO data
        }
        if(testbit(Buttons, userinput)) {
            clrbit(Buttons, userinput);
            if (testbit(Buttons,K2) && testbit(Buttons,K3)) togglebit(CHDctrl,ascii);
            if (testbit(Buttons,K1)) togglebit(MStatus, stop);
            if (testbit(Buttons,K2)) if(page)    page--;
            if (testbit(Buttons,K3)) if(page<15) page++;
            if (testbit(Buttons,KM)) clrbit(MStatus,gosniffer);  // Exit sniffer
        }
        // Act on any change of MStatus.stop: K1, 'f'/'g' from USB or UART, single sniff complete
        if((MStatus ^ stopped) & _BV(stop)) {
            stopped ^= _BV(stop);
            if(stopped) {                   // Stop sniffer
                USARTC0.CTRLB = 0x00;       // Disable RX
                clrbit(SPIC.CTRL,6);        // Disable SPI
                if(M.CHDdecode==i2c) PORTC.INTCTRL = 0x00;   // Disable PORTC interrupts
            }
            else {                          // Restart Sniffer
                if(M.CHDdecode==i2c) {
                    if(testbit(Mcursors, singlesniff)) {    // a new capture from the start
                        T.LOGIC.addr_ack_ptr = T.LOGIC.data.I2C.addr_ack;
                        T.LOGIC.addr_ack_pos = 0x80;
                        T.LOGIC.data.I2C.addr_ack[0] = 0;
                        T.LOGIC.i2c_acc = 1; T.LOGIC.i2c_first = 1;
                    }
                    PORTC.INTFLAGS  = 0x03;
                    PORTC.INTCTRL   = 0x0F;     // Enable PortC INT 0 and INT 1, High Priority
                }
                else if(M.CHDdecode==rs232) {
                    USARTC0.CTRLB = 0x10;       // Enable RX
                }
                else if(M.CHDdecode==spi) {
                    setbit(SPIC.CTRL,6);        // Enable SPI
                }
                if(testbit(Mcursors, singlesniff)) {
                    ConfigLogicDMA();
                }
            }
        }
    } while (testbit(MStatus,gosniffer));
exitSniffer:
    Buttons=0;
   	endpoints[1].in.DATAPTR = (uint16_t)T.SCOPE.DC.CH1data;
	endpoints[1].in.STATUS = USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm;
    clrbit(Misc,sacquired);
    USARTC0.CTRLA = 0x00;   // Disable receive interrupt
    USARTC0.CTRLB = 0x00;   // Disable UART RX
    TCC1.INTCTRLB = 0x00;   // Disable timer interrupt
    TCC1.CTRLA = 0x00;      // Disable timer
    TCC1.CTRLD = 0x00;      // No restart on event (TCC1 counts post trigger samples next)
    PORTC.INTCTRL = 0x00;   // Disable PORTC interrupts
    PORTC.INT0MASK  = 0x01; // PC0 (SDA) is the interrupt 0 source, as main() set it
    PORTC.INT1MASK  = 0x80; // PC7 (SCK) is the interrupt 1 source
    PORTC.PIN1CTRL &= 0b11111000;   // SCL: sense both edges again
    SPIC.CTRL = 0x00;       // Disable SPI
    PR.PRPC  = 0x7C;        // Stop: TWI, USART0, USART1, SPI, HIRES
    RTC.INTCTRL = 0x05;     // Re enable Time out interrupt
}

// Display SPI or UART data
void DisplayData(uint8_t side, uint8_t page) {
    uint8_t Xoff, size;
    uint16_t index;
    uint8_t *p;
    if(side==0) {   // Left side (RX)
        Xoff=0;
        index = T.LOGIC.indrx;
        p = T.LOGIC.data.Serial.RX;
    } else {        // Right side (TX)
        Xoff=71;
        index = T.LOGIC.indtx;
        p = T.LOGIC.data.Serial.TX;
    }
    u8CursorX=Xoff;
    u8CursorY=0;
    if(testbit(CHDctrl,ascii)) {
        size=DATA_IN_PAGE_SERIAL*2;         // In ASCII mode, twice the data fits on the page
        page=page/2;
    } else size=DATA_IN_PAGE_SERIAL;
    uint16_t i=(uint16_t)page*size;
    if(testbit(Trigger, round)) {
        if(index>=size) i=index-size;
        else          i=(BUFFER_SERIAL-size)+index;
        i+=(uint16_t)(page+1)*size;
        if(i>=BUFFER_SERIAL) i-=BUFFER_SERIAL;
    }
    while(u8CursorY<=TEXT_LAST_LINE) {    // Print until the cursor is out of the screen
        if(!testbit(Trigger, round)) {
            if(i>=index) break;
        }
        uint8_t data=p[i];
        i++; if(i>=BUFFER_SERIAL) i=0;    // Increase data index
        if(testbit(CHDctrl,ascii)) {
                 if(data==0x0A) putchar3x6(0x14); // Line Feed
            else if(data==0x0D) putchar3x6(0x15); // Carriage Return
            else if(data <0x20) putchar3x6('_');  // Other special character
            else putchar3x6(data);
            u8CursorX+=2;
        }
        else {
            printhex3x6(data);
            putchar3x6(' ');
        }            
        if(side==0) {
            if(u8CursorX>=59) {
                u8CursorX = 0; u8CursorY++;
            }            
        }
        else if(u8CursorX<71) {
            u8CursorX = 71;
        }
    }    
}

// I2C and SPI MISO decoders: naked assembler, one short interrupt per bus edge.
// The main loop runs between edges, so a long or continuous stream slows the
// instrument down instead of locking it (Tester DUT SNIFF LOAD).
//
// I2C: INT0 = SCL (PC1) rising, one bit each; INT1 = SDA (PC0) falling, a START
// when SCL is high. A bit is sampled ~10 cycles after its SCL edge (Fast-mode
// guarantees >= 19 high). T.LOGIC.i2c_acc collects the bits behind a marker that reaches
// C at the 8th; 0 means "the next clock is the ACK". The complete byte goes to
// T.LOGIC.i2c_byte, which DMA CH1 copies into decoded[] (wrap or single sniff, as the
// serial sniffers); the ACK clock adds the byte's two flags to addr_ack[].
// T.LOGIC.i2c_first: 1 = the byte after a START (the address).
// SPI: INT1 = SCK (PC7), one byte per interrupt (see the SPI notes below).
// INT1 tells them apart by its pin mask: PC7 (SCK) is only in it for SPI.

ISR(PORTC_INT0_vect, ISR_NAKED) {     // I2C: SCL rose
    asm volatile(
    "     push  r24"                  "\n\t"
    "     in    r24, %[vin]"          "\n\t"   /* SDA (bit 0) at the SCL edge */
    "     push  r25"                  "\n\t"
    "     in    r25, %[sreg]"         "\n\t"
    "     push  r25"                  "\n\t"
    // An SDA fall still pending at this SCL rise came while SCL was low: a data
    // change, not a START. Its INT1 must not run now, with SCL high: behind the ACK
    // path it would, and restart the byte (the fall is ~1 bit before this rise)
    "     ldi   r25, 2"               "\n\t"
    "     sts   %[intf], r25"         "\n\t"   /* PORTC.INTFLAGS = INT1IF */
    "     lds   r25, %[acc]"          "\n\t"
    "     tst   r25"                  "\n\t"
    "     breq  5f"                   "\n\t"   /* the ACK clock */
    "     lsl   r25"                  "\n\t"   /* C = the marker after the 8th bit */
    "     sbrc  r24, 0"               "\n\t"
    "     ori   r25, 1"               "\n\t"   /* (ORI leaves C alone) */
    "     brcs  4f"                   "\n\t"
    "     sts   %[acc], r25"          "\n\t"
    // SDA sampled high and low now with SCL still high: a (repeated) START that
    // came while this interrupt held INT1 off
    "1:   sbrs  r24, 0"               "\n\t"
    "     rjmp  9f"                   "\n\t"
    "     sbic  %[vin], 0"            "\n\t"
    "     rjmp  9f"                   "\n\t"
    "     sbis  %[vin], 1"            "\n\t"
    "     rjmp  9f"                   "\n\t"
    "     ldi   r25, 1"               "\n\t"
    "     sts   %[acc], r25"          "\n\t"
    "     sts   %[first], r25"        "\n\t"
    "9:   pop   r25"                  "\n\t"
    "     out   %[sreg], r25"         "\n\t"
    "     pop   r25"                  "\n\t"
    "     pop   r24"                  "\n\t"
    "     reti"                       "\n\t"
    // the 8th bit: the byte to the DMA (no request while stopped), the ACK next
    "4:   sts   %[byte], r25"         "\n\t"
    "     ldi   r25, 0"               "\n\t"
    "     sts   %[acc], r25"          "\n\t"
    "     sbic  %[gpb], %[stp]"   "\n\t"
    "     rjmp  1b"                   "\n\t"
    "     lds   r25, %[dmaa]"         "\n\t"
    "     ori   r25, 0x10"            "\n\t"   /* TRFREQ */
    "     sts   %[dmaa], r25"         "\n\t"
    "     rjmp  1b"                   "\n\t"
    // the ACK clock (SDA low = ACK): the byte's address and ACK flags, two bits per
    // byte, MSB first, four bytes to an addr_ack[] byte, wrapping with the DMA. Only
    // this byte's two bits are cleared and set: after a wrap the rest of its addr_ack
    // byte still holds the flags of three older bytes in the buffer. A single sniff
    // stops after the flags of the byte that filled it (CH1 turns itself off on it)
    "5:   push  r23"                  "\n\t"
    "     push  r26"                  "\n\t"
    "     push  r27"                  "\n\t"
    "     bst   r24, 0"               "\n\t"   /* T = SDA: 1 = NACK */
    "     lds   r26, %[ptr]"          "\n\t"
    "     lds   r27, %[ptr]+1"        "\n\t"
    "     lds   r23, %[pos]"          "\n\t"
    "     tst   r23"                  "\n\t"
    "     brne  6f"                   "\n\t"
    "     adiw  r26, 1"               "\n\t"   /* this addr_ack byte is full */
    "     cpi   r26, lo8(%[aaend])"   "\n\t"
    "     ldi   r25, hi8(%[aaend])"   "\n\t"
    "     cpc   r27, r25"             "\n\t"
    "     brne  2f"                   "\n\t"
    "     ldi   r26, lo8(%[aa])"      "\n\t"
    "     ldi   r27, hi8(%[aa])"      "\n\t"
    "2:   sts   %[ptr], r26"          "\n\t"
    "     sts   %[ptr]+1, r27"        "\n\t"
    "     ldi   r23, 0x80"            "\n\t"
    "6:   ld    r25, X"               "\n\t"
    "     mov   r24, r23"             "\n\t"
    "     lsr   r24"                  "\n\t"
    "     or    r24, r23"             "\n\t"   /* this byte's two bits */
    "     com   r24"                  "\n\t"
    "     and   r25, r24"             "\n\t"   /* cleared: an old byte's, after a wrap */
    "     lds   r24, %[first]"        "\n\t"
    "     sbrc  r24, 0"               "\n\t"
    "     or    r25, r23"             "\n\t"   /* an address */
    "     lsr   r23"                  "\n\t"
    "     brts  3f"                   "\n\t"
    "     or    r25, r23"             "\n\t"   /* ACK */
    "3:   lsr   r23"                  "\n\t"
    "     st    X, r25"               "\n\t"
    "     sts   %[pos], r23"          "\n\t"
    "     ldi   r25, 0"               "\n\t"
    "     sts   %[first], r25"        "\n\t"
    "     ldi   r25, 1"               "\n\t"
    "     sts   %[acc], r25"          "\n\t"
    "     lds   r25, %[dmaa]"         "\n\t"
    "     sbrs  r25, 7"               "\n\t"   /* CH1 off: this byte filled a single sniff */
    "     rjmp  7f"                   "\n\t"
    "8:   pop   r27"                  "\n\t"
    "     pop   r26"                  "\n\t"
    "     pop   r23"                  "\n\t"
    "     rjmp  9b"                   "\n\t"
    "7:   ldi   r25, 0"               "\n\t"   /* full: stop, as the old decoder did */
    "     sts   %[intc], r25"         "\n\t"
    "     sbi   %[gpb], %[stp]"   "\n\t"
    "     rjmp  8b"                   "\n\t"
    :
    : [vin] "I" (_SFR_IO_ADDR(VPORT2_IN)), [sreg] "I" (_SFR_IO_ADDR(SREG)),
      [gpb] "I" (_SFR_IO_ADDR(GPIOB)), [stp] "I" (stop),
      [acc] "i" (&T.LOGIC.i2c_acc), [first] "i" (&T.LOGIC.i2c_first), [byte] "i" (&T.LOGIC.i2c_byte),
      [dmaa] "i" (_SFR_MEM_ADDR(DMA_CH1_CTRLA)), [intc] "i" (_SFR_MEM_ADDR(PORTC_INTCTRL)),
      [intf] "i" (_SFR_MEM_ADDR(PORTC_INTFLAGS)),
      [ptr] "i" (&T.LOGIC.addr_ack_ptr), [pos] "i" (&T.LOGIC.addr_ack_pos),
      [aa] "i" (T.LOGIC.data.I2C.addr_ack), [aaend] "i" (T.LOGIC.data.I2C.addr_ack + BUFFER_I2C/4)
    );
}

// INT1: I2C START (SDA fell) or SPI MISO (SCK's first edge of a byte).
// SPI: the first SCK edge of a byte raises the interrupt, so it is not waited for:
// in CPHA 0 the bit it clocks is the port read at entry (~10 cycles after the edge),
// in CPHA 1 the code goes straight to that edge's other half. Every wait is a
// 5-cycle poll counting down a register; the TCC0 timeout is looked at every 256
// polls. The byte is handed to DMA CH1 in a half period the code only waits through
// (bit 8's high half in CPHA 0, its low half in CPHA 1). One byte per interrupt, on
// purpose: staying in for the next byte decodes back to back to 800 kHz, but a
// continuous SCK then holds the CPU for as long as it runs - no USB, no main loop to
// show or stream the data, and a watchdog reset after 8 s.
// Registers (SPI): r24 = port at the first edge, r23 = data, r22 = bits left,
// r21 = countdown, r25 = scratch.
ISR(PORTC_INT1_vect, ISR_NAKED) {
    asm volatile(
    "     push  r24"                  "\n\t"
    "     in    r24, %[vin]"          "\n\t"   /* SPI: MISO; I2C: SCL when SDA fell */
    "     push  r25"                  "\n\t"
    "     lds   r25, %[i1mask]"       "\n\t"
    "     sbrc  r25, 7"               "\n\t"   /* SCK (PC7) in the mask: SPI */
    "     rjmp  30f"                  "\n\t"
    "     sbrs  r24, 1"               "\n\t"   /* I2C: SCL low, a data change */
    "     rjmp  39f"                  "\n\t"
    "     lds   r25, %[intf]"         "\n\t"   /* SCL high, but its rise not served yet: */
    "     sbrc  r25, 0"               "\n\t"   /* SDA fell before it, a data change */
    "     rjmp  39f"                  "\n\t"
    "     ldi   r25, 1"               "\n\t"   /* SCL high: a (repeated) START */
    "     sts   %[acc], r25"          "\n\t"
    "     sts   %[first], r25"        "\n\t"
    "39:  pop   r25"                  "\n\t"
    "     pop   r24"                  "\n\t"
    "     reti"                       "\n\t"
    "30:  in    r25, %[sreg]"         "\n\t"
    "     push  r25"                  "\n\t"
    "     push  r21"                  "\n\t"
    "     push  r22"                  "\n\t"
    "     push  r23"                  "\n\t"
    // Wait for SCK low / high: 5 cycles per poll; every 256 polls the TCC0 timeout
#define SPI_WAIT(skip, n) \
    n ":  " skip " %[vin], 7"    "\n\t" \
    "     rjmp  " n "f"          "\n\t" \
    "     dec   r21"             "\n\t" \
    "     brne  " n "b"          "\n\t" \
    "     lds   r25, 0x080C"     "\n\t"   /* TCC0.INTFLAGS */ \
    "     sbrc  r25, 0"          "\n\t"   /* LUNFIF: timeout */ \
    "     rjmp  99f"             "\n\t" \
    "     rjmp  " n "b"          "\n\t" \
    n ":"                        "\n\t"
#define WAIT_LOW(n)  SPI_WAIT("sbis", n)    /* skip the exit while SCK is high */
#define WAIT_HIGH(n) SPI_WAIT("sbic", n)    /* skip the exit while SCK is low  */
    // Hand the byte to the DMA (addr_ack_pos, transfer request unless stopped),
    // clear PORTC INT1 (a later rising edge re-enters) and restart the timeout
#define SPI_STORE \
    "     sts   %[aap], r23"     "\n\t" \
    "     ldi   r25, 2"          "\n\t" \
    "     sts   0x064C, r25"     "\n\t"   /* PORTC.INTFLAGS = INT1IF */ \
    "     sts   0x0820, r25"     "\n\t"   /* TCC0.CNTL = 2: 81.92 ms */ \
    "     ldi   r25, 1"          "\n\t" \
    "     sts   0x080C, r25"     "\n\t"   /* clear LUNFIF */ \
    "     sbic  %[gpb], %[stp]" "\n\t" \
    "     rjmp  3f"              "\n\t" \
    "     lds   r25, 0x0120"     "\n\t"   /* DMA.CH1.CTRLA |= TRFREQ */ \
    "     ori   r25, 0x10"       "\n\t" \
    "     sts   0x0120, r25"     "\n\t" \
    "3:"                         "\n\t"
    "     ldi   r25, 2"          "\n\t"
    "     sts   0x0820, r25"     "\n\t"   /* TCC0.CNTL = 2: 81.92 ms */
    "     ldi   r25, 1"          "\n\t"
    "     sts   0x080C, r25"     "\n\t"   /* clear LUNFIF */
    "     clr   r21"             "\n\t"
    "     sbic  %[gpa], 7"       "\n\t"   /* Sniffer.CPHA */
    "     rjmp  20f"             "\n\t"
    // CPHA 0: sample on the first edge
    "     bst   r24, 6"          "\n\t"   /* the first bit, read at the edge */
    "     clr   r23"             "\n\t"
    "     bld   r23, 0"          "\n\t"
    "     ldi   r22, 7"          "\n\t"
    "11:"                        "\n\t"
    WAIT_LOW("4")
    WAIT_HIGH("5")
    "     lsl   r23"             "\n\t"
    "     sbic  %[vin], 6"       "\n\t"   /* MISO */
    "     ori   r23, 1"          "\n\t"
    "     dec   r22"             "\n\t"
    "     brne  11b"             "\n\t"
    SPI_STORE                             /* in bit 8's high half */
    WAIT_LOW("6")
    "     rjmp  99f"             "\n\t"   /* one byte per interrupt */
    // CPHA 1: sample on the second edge
    "20:  clr   r23"             "\n\t"
    "     ldi   r22, 8"          "\n\t"
    "21:"                        "\n\t"
    WAIT_LOW("7")
    "     lsl   r23"             "\n\t"
    "     sbic  %[vin], 6"       "\n\t"   /* MISO */
    "     ori   r23, 1"          "\n\t"
    "     dec   r22"             "\n\t"
    "     breq  22f"             "\n\t"
    WAIT_HIGH("8")
    "     rjmp  21b"             "\n\t"
    "22:"                        "\n\t"
    SPI_STORE                             /* in bit 8's low half */
    "99:  pop   r23"             "\n\t"
    "     pop   r22"             "\n\t"
    "     pop   r21"             "\n\t"
    "     pop   r25"             "\n\t"
    "     out   %[sreg], r25"    "\n\t"
    "     pop   r25"             "\n\t"
    "     pop   r24"             "\n\t"
    "     reti"                  "\n\t"
#undef SPI_WAIT
#undef WAIT_LOW
#undef WAIT_HIGH
#undef SPI_STORE
    :
    : [vin] "I" (_SFR_IO_ADDR(VPORT2_IN)), [sreg] "I" (_SFR_IO_ADDR(SREG)),
      [gpa] "I" (_SFR_IO_ADDR(GPIOA)), [gpb] "I" (_SFR_IO_ADDR(GPIOB)), [stp] "I" (stop),
      [i1mask] "i" (_SFR_MEM_ADDR(PORTC_INT1MASK)), [aap] "i" (&T.LOGIC.addr_ack_pos),
      [acc] "i" (&T.LOGIC.i2c_acc), [first] "i" (&T.LOGIC.i2c_first),
      [intf] "i" (_SFR_MEM_ADDR(PORTC_INTFLAGS))
    );
}

// Bit banging to decode the TX pin:
// TCC1 runs at the baud rate and every TX edge restarts it (event CH2),
// so compare A (half a bit) lands in the middle of each bit.
// The interrupt checks for a start bit, then the whole character is
// processed inside the interrupt, resynchronizing on every edge.
ISR(TCC1_CCA_vect) {
    uint8_t i, data=0, paritybit=0;

    // Start Bit: low in the middle of a bit
    if(testbit(VPORT2.IN, 3)) return;
    // A start edge right after the compare (during the interrupt entry) also restarted TCC1:
    // this is the beginning of the start bit, wait for its middle (CCA >= 138 at 115200)
    if(TCC1.CNT < 128) return;

    // Data bits
    for(i=0; i<T.LOGIC.databits; i++) {
        TCC1.INTFLAGS = TC1_CCAIF_bm;   // Clear compare flag
        while(!testbit(TCC1.INTFLAGS,TC1_CCAIF_bp));   // wait for next time bit
        data=data>>1;
        if(testbit(VPORT2.IN, 3)) {
            data|=0x80;
            paritybit++;
        }
    }
    for(i=T.LOGIC.databits; i<8; i++) data=data>>1;
    // Check Parity
    if(testbit(Sniffer,parmode)) {
        TCC1.INTFLAGS = TC1_CCAIF_bm;   // Clear compare flag
        while(!testbit(TCC1.INTFLAGS,TC1_CCAIF_bp));   // wait for next time bit
        if(testbit(Sniffer,parity)) {  // Odd Parity
            if( (testbit(paritybit,0) && testbit(VPORT2.IN, 3)) ||
                (!testbit(paritybit,0) && !testbit(VPORT2.IN, 3)))
                goto exituart;   // Parity error
        }
        else {                  // Even Parity
            if( (!testbit(paritybit,0) && testbit(VPORT2.IN, 3)) ||
                (testbit(paritybit,0) && !testbit(VPORT2.IN, 3)))
                goto exituart;   // Parity error
        }
    }

    // First Stop bit
    TCC1.INTFLAGS = TC1_CCAIF_bm;   // Clear compare flag
    while(!testbit(TCC1.INTFLAGS,TC1_CCAIF_bp));   // wait for next time bit
    if(!testbit(VPORT2.IN, 3))
        goto exituart;       // stop bit error

    // Second Stop bit
    if(testbit(Sniffer,stopbit)) {
        TCC1.INTFLAGS = TC1_CCAIF_bm;   // Clear compare flag
        while(!testbit(TCC1.INTFLAGS,TC1_CCAIF_bp));   // wait for next time bit
        if(!testbit(VPORT2.IN, 3))
            goto exituart;  // stop bit error
    }

    T.LOGIC.addr_ack_pos=data;   // data holder for DMA
    // Save data on buffer
    if(!testbit(MStatus, stop)) setbit(DMA.CH1.CTRLA,4);    // Transfer request

exituart:
    TCC1.INTFLAGS = TC1_CCAIF_bm;   // Clear compare flag
}
