#include <stdint.h>
#include <avr/interrupt.h>
#include "logic.h"
#include "main.h"
#include "mso.h"
#include "USB\usb_xmega.h"
#include "utils.h"

void LogicDMA(void);
void DisplayData(uint8_t side, uint8_t page);

//    0   1   2   3   4   5   6   7
//  |...|...|...|...|...|...|...|...|
//   ^
//   |
//   sample bit 3 times
//
// Display the HEX value of the digital stream
void HEXSerial(void) {
    int8_t start,end;
    int16_t increment;
    // The vertical cursors define the section to decode
    start = M.VcursorA;
    end   = M.VcursorB;
    // If the vertical cursors are off, use the entire screen
    if(!testbit(Mcursors, cursorv)) {
        start = 0;
        end   = 127;
    }
    increment = (end-start);
	increment<<=3;
    if(Srate>=11) {
        increment*=2;
        start=start<<1;
    }
    uint8_t result;							// Does not need to be initialized (all bits are shifted in)
    uint8_t data,bit=0,yPos,mask;
    yPos = M.CHDpos>>3;
    for(mask=0x80; mask; mask=mask>>1) {	// Loop 8 digital channels
        int16_t index = (start<<8);
        if(Srate<11) index+=(M.HPos<<8);
        index+=increment;
        if(CHDmask&mask) {					// Is this channel on?
            for(uint8_t i=1; i<=32; i++) {	// Scan buffer
                data = DC.CHDdata[hibyte(index)];
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

void LogicDMA(void) {
    setbit(DMA.CH0.CTRLA,6);    // reset DMA CH0
    setbit(DMA.CH1.CTRLA,6);    // reset DMA CH1
    // Flush receive buffers
    while(testbit(USARTC0.STATUS,7)) {
        (void)USARTC0.DATA;             // Dummy read to flush buffer
    }
    if(SPIC.STATUS) (void)SPIC.DATA;	// dummy read

	uint8_t src_addr_lo;
    uint8_t src_addr_hi;
	uint8_t trig_src;

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

    // For UART and SPI Sniffer, bit-bang / DMA:
    DMA.CH1.ADDRCTRL  = 0b10000101;     // reload source addr after each burst, incr dest, reload dest end block
    DMA.CH1.TRFCNT    = BUFFER_SERIAL;            // buffer size
    DMA.CH1.DESTADDR0 = (((uint16_t) T.LOGIC.data.Serial.TX)     ) & 0xFF;
    DMA.CH1.DESTADDR1 = (((uint16_t) T.LOGIC.data.Serial.TX)>>1*8) & 0xFF;
    //DMA.CH1.TRIGSRC   = 0x00;               // Manual
    DMA.CH1.SRCADDR0  = (((uint16_t)(&T.LOGIC.addr_ack_pos))     ) & 0xFF;
    DMA.CH1.SRCADDR1  = (((uint16_t)(&T.LOGIC.addr_ack_pos))>>1*8) & 0xFF;

    if(testbit(Mcursors, singlesniff)) DMA.CH1.CTRLA     = 0b10000100;    // no repeat, 1 byte burst
    else DMA.CH1.CTRLA     = 0b10100100;                            // repeat, 1 byte burst
}

// Protocol Sniffer
void Sniff(void) {
    uint16_t i=0;
    uint8_t n,addrdata, ack, data;
    uint8_t page=0; // 16 pages to display 1024 bytes of data
                    // one page has 8 lines, each line will display
                    // 8 bytes in this format: "FF+ "
    clr_display();
    uint8_t *p=T.LOGIC.data.All.decoded;
    for(; i<BUFFER_SERIAL*2+2; i++) *p++=0;   // Erase buffers and index
    T.LOGIC.indrx=0; T.LOGIC.indtx=0;
    if(testbit(Trigger, round)) page=0x0F;  // Go to last page
    // Setup
    PR.PRPC  = 0x00;        // Enable PORTC peripherals
    RTC.INTCTRL = 0x01;     // Disable Menu Timeout interrupt
    uint8_t spictrl=0;
    if(M.CHDdecode==i2c) {
        print3x6(PSTR("I2C SNIFFER\nBIT0:SDA BIT1:SCL"));
        T.LOGIC.addr_ack_ptr =  T.LOGIC.data.I2C.addr_ack;
        T.LOGIC.data_ptr = T.LOGIC.data.I2C.decoded;
        T.LOGIC.addr_ack_pos = 0x80;      // counter for keeping track of all bits
        PORTC.PIN0CTRL = (PORTC.PIN0CTRL & 0b11111000) | 0b00000010;    // Sense falling edge of SDA
        PORTC.INTFLAGS = 0x03;  // Clear flags
        PORTC.INTCTRL   = 0x03; // Enable PortC INT 0, High Priority
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
        USARTC0.CTRLC = databits;       // Async, No Parity, 1 stop bit, 8 data bits
                
        USARTC0.CTRLB = 0x10;       // Enable RX
        TCC1.CTRLA = 0x01;  // DIV1
        T.LOGIC.baud = (uint16_t)pgm_read_word_near(TCC1val+baud);
        TCC1.PER = T.LOGIC.baud/4;   // Sample at 4 times the baud rate
        TCC1.INTCTRLA = 0x03;           // Enable TCC1 interrupt, High Priority
    }
    else if(M.CHDdecode==spi) {
        uint8_t pin7ctrl=1;   // Sense rising edge
        print3x6(PSTR("SPI SNIFFER\nBIT4:/SS\nBIT5:MOSI\nBIT6:MISO\nBIT7:SCK"));
        if(testbit(Sniffer,CPOL)) {    // CPOL == 1
            pin7ctrl = PORT_INVEN_bm+1;   // Invert SCK, Sense rising edge
        }
        if(testbit(Sniffer,CPHA)) {
            spictrl=0x04;  // CPHA == 1
        }
        PORTC.PIN7CTRL = pin7ctrl;
        if(testbit(Sniffer,SSINV)) {    // Invert Slave select
            PORTC.PIN4CTRL = PORT_INVEN_bm;
        }
    }
    LogicDMA();
    dma_display(); WaitDisplay();
    while((T.LOGIC.indrx==0) && (T.LOGIC.indtx==0)) {
        WDR();
        if(M.CHDdecode!=i2c) {
            T.LOGIC.indrx=BUFFER_SERIAL-DMA.CH0.TRFCNT;
            T.LOGIC.indtx=BUFFER_SERIAL-DMA.CH1.TRFCNT;            
        }            
        if(M.CHDdecode==spi) {
            if(testbit(VPORT2.IN, 4)) {     // SS not asserted
                SPIC.CTRL = 0b01000000 + spictrl;       // Enable SPI, Slave mode
                PORTC.INTFLAGS = 0x03;  // Clear flags
                PORTC.INTCTRL   = 0x0C; // Enable PortC INT 1, High Priority
            }                
        }
        if(!testbit(MStatus,gosniffer)) goto exitSniffer;
        if (testbit(Buttons,KM)) clrbit(MStatus,gosniffer);
    }
////////////////////////////////////////////////////////////////////////////////////
// DISPLAY
////////////////////////////////////////////////////////////////////////////////////
    do {
        dma_display(); WaitDisplay();
        clr_display();
        OFFGRN();
		// USB - Send new data if previous transfer complete
		if((endpoints[1].in.STATUS & USB_EP_TRNCOMPL0_bm)) {
    		endpoints[1].in.AUXDATA = 0;				// New transfer must clear AUXDATA
            if(!testbit(Misc,sacquired)) {
               	endpoints[1].in.DATAPTR = (uint16_t)T.LOGIC.data.All.decoded;
        		endpoints[1].in.CNT = 640;	    // Send first half
            }
            else {
               	endpoints[1].in.DATAPTR = (uint16_t)T.LOGIC.data.All.decoded+640;
        		endpoints[1].in.CNT = 649;	    // Send second half
            }
    		endpoints[1].in.STATUS &= ~(USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm | USB_EP_OVF_bm);
            togglebit(Misc,sacquired);
		}
        // Display I2C data
        if(M.CHDdecode==i2c) {
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
            for(n=0; n<DATA_IN_PAGE_I2C; n++) {
                if(!testbit(Trigger, round)) {
                    if(i>=T.LOGIC.indrx) break;
                }
                uint8_t shift;
                shift = (i&0x0003)*2;
                data=T.LOGIC.data.I2C.decoded[i];
                addrdata=T.LOGIC.data.I2C.addr_ack[i/4];
                ack=      (addrdata<<(shift+1))&0x80;
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
                lcd_line(63,8,63,30);
                const char *c=STR_STOP;
                for(uint8_t y=TEXT_LAST_LINE-3; y<=TEXT_LAST_LINE; y++) {   // Print STOP vertically
                    lcd_goto(62,y); putchar3x6(pgm_read_byte(c++));
                }
            }
            else lcd_line(63,8,63,DISPLAY_MAX_Y);
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
            if (testbit(Buttons,K1)) {
                togglebit(MStatus, stop);
                if(testbit(MStatus, stop)) {    // Stop sniffer
                    USARTC0.CTRLB = 0x00;       // Disable RX
                    clrbit(SPIC.CTRL,6);        // Disable SPI
                    if(M.CHDdecode==i2c) PORTC.INTCTRL = 0x00;   // Disable PORTC interrupts
                }
                else {                          // Restart Sniffer
                    if(M.CHDdecode==i2c) {
                        PORTC.INTCTRL   = 0x03;     // Enable PortC INT 0, High Priority
                    }
                    else if(M.CHDdecode==rs232) {
                        USARTC0.CTRLB = 0x10;       // Enable RX
                    }
                    else if(M.CHDdecode==spi) {
                        setbit(SPIC.CTRL,6);        // Enable SPI
                    }
                    if(testbit(Mcursors, singlesniff)) {
                        LogicDMA();
                    }
                }
            }
            if (testbit(Buttons,K2)) if(page)    page--;
            if (testbit(Buttons,K3)) if(page<15) page++;
            if (testbit(Buttons,KM)) clrbit(MStatus,gosniffer);  // Exit sniffer
        }
    } while (testbit(MStatus,gosniffer));
exitSniffer:
    Buttons=0;
   	endpoints[1].in.DATAPTR = (uint16_t)DC.CH1data;
	endpoints[1].in.STATUS = USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm;
    clrbit(Misc,sacquired);
    USARTC0.CTRLA = 0x00;   // Disable receive interrupt
    USARTC0.CTRLB = 0x00;   // Disable UART RX
    TCC1.INTCTRLA = 0x00;   // Disable timer interrupt
    TCC1.CTRLA = 0x00;      // Disable timer
    PORTC.INTCTRL = 0x00;   // Disable PORTC interrupts
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

#define SDA_PIN 0
#define SCL_PIN 1
#define SCL (unsigned char) (VPORT2.IN & _BV(SCL_PIN))
#define SDA (unsigned char) (VPORT2.IN & _BV(SDA_PIN))

// Decode I2C (TWI) protocol
// Temp.LOGIC.data.I2C.decoded[1024] holds data
// Temp.LOGIC.data.I2C.addr_ack[257] hold if the byte is address or data and if the byte had an ack
// Interrupt on falling edge of SDA
ISR(PORTC_INT0_vect) {
    uint8_t data;       // byte to accumulate data information
    uint8_t bits;       // current bit being processed
    uint8_t first=1;    // First data after start
    uint8_t *dataptr=T.LOGIC.data_ptr;           // temp data pointer
    uint8_t *bitsptr=T.LOGIC.addr_ack_ptr;       // temp bits pointer
    uint8_t current_pos=T.LOGIC.addr_ack_pos;    // Current bit position in byte
    const uint8_t *queue_max=T.LOGIC.data.I2C.decoded+1024;
    TCC0.CNTL = 24;     // 1.024 second timeout
    setbit(TCC0.INTFLAGS, TC2_LUNFIF_bp);    // Clear trigger timeout interrupt
    ONGRN();
repeated_start:
    bits = 0;
    // wait for SCL to fall
    while(SCL) if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) goto exiti2c;
sample_bit:    // now ready for next bit of the byte
    while(SCL == 0) if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) goto exiti2c;  // wait for rising edge
    // update buffers here
    if (dataptr == queue_max) goto queue_is_full;
    uint8_t dt = SDA;   // Sampling SDA on the rising edge of SCL
    // we might not use this data in the cases of either stop or repeated start conditions
    // SCL is now high, wait for either of two conditions: SCL falls or SDA changes
    // split the cases to reduce the sub-loop overheads
    if (dt == 0) {
        // special case where stop condition might exist because data was zero
look_for_bit_or_stop:
        if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) goto exiti2c;    // timeout
        if (SCL == 0) goto process_bit;
        if (SDA == 0) goto look_for_bit_or_stop;
        if (SCL == 0) goto process_bit; // double check, in case both signals changed at the same time
        // SCL and SDA are now high (stop condition occurred)
        goto exiti2c;
    }
    else {
        // special case where we might have a repeated start where SDA falls while SCL high
look_for_bit_or_repeated_start:
        if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) goto exiti2c;    // timeout
        if (SCL == 0) goto process_bit;
        if (SDA) goto look_for_bit_or_repeated_start;
        if (SCL == 0) goto process_bit; // double check, in case both signals changed at the same time
        first=1;
        goto repeated_start;
    }
process_bit:
    // process another data bit
    if (bits == 8) {
        bits = 0;  // next SCL rise starts a new byte of data
        // looking at the acknowledge bit
        *dataptr++ = data;  // store the data

        // addr/data bits and acknowledge bits are accumulated into a byte to save memory
        // so, there are 4 addr/data and 4 ack/nack bits before we fill 1 byte
        uint8_t bits;
        bits=*bitsptr;

        if(first) bits |= current_pos;
        current_pos = current_pos>>1;
        if(!dt) bits |= current_pos;   // Ack by pulling line low
        current_pos = current_pos>>1;
        *bitsptr=bits;
        if (current_pos == 0) {  // 8 bits done
            bitsptr++;
            *bitsptr=0;
            current_pos = 0x80;
        }
        first=0;    // subsequent bytes are data bytes
    }
    else {
        // accumulate bits of the byte's information (MSB to LSB)
        data = (data << 1) | dt;
        bits++;
    }
    goto sample_bit;

    // the program comes here when the memory is full
queue_is_full:
    // If single trigger, now go to stop
    if(testbit(Mcursors, singlesniff)) {
        PORTC.INTCTRL = 0x00;   // Disable PORTC interrupts
        setbit(MStatus, stop);
        T.LOGIC.indrx=BUFFER_I2C;
        goto exiti2cb;
    }
    bitsptr=T.LOGIC.data.I2C.addr_ack;
    *bitsptr=0;
    dataptr=T.LOGIC.data.I2C.decoded;   // pointer to data
    goto sample_bit;
    
exiti2c:
    T.LOGIC.indrx=dataptr-T.LOGIC.data.I2C.decoded;
exiti2cb:    
    T.LOGIC.data_ptr=dataptr;
    T.LOGIC.addr_ack_ptr=bitsptr;
    T.LOGIC.addr_ack_pos=current_pos;
    PORTC.INTFLAGS = 0x01;  // Clear rising edge interrupt
}

// Decode SPI protocol, SCK triggers this interrupt
// Temp.LOGIC.addr_ack_pos holds data
ISR(PORTC_INT1_vect) {
    uint8_t data, i=8;
    TCC0.CNTL = 2;      // Max 81.92ms timeout
    setbit(TCC0.INTFLAGS, TC2_LUNFIF_bp);    // Clear trigger timeout interrupt

    if(testbit(Sniffer,CPHA)) {    // CPHA == 1 - Sample on second edge
        do {
            // wait for clock rising
            while(!testbit(VPORT2.IN, 7)) if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) return;
            data=data<<1;
            // wait for clock falling
            while(testbit(VPORT2.IN, 7)) if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) return;
            if(testbit(VPORT2.IN, 6)) data|=0x01; // Read MISO
        } while(--i);

    }
    else {                  // CPHA == 0    - Sample on first edge
        do {
            // wait for clock rising
            while(!testbit(VPORT2.IN, 7)) if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) return;
            data=data<<1;
            if(testbit(VPORT2.IN, 6)) data|=0x01; // Read MISO
            // wait for clock falling
            while(testbit(VPORT2.IN, 7)) if(testbit(TCC0.INTFLAGS, TC2_LUNFIF_bp)) return;
        } while(--i);
    }
    T.LOGIC.addr_ack_pos=data;      // data holder for DMA
    PORTC.INTFLAGS = 0x02;      // Clear rising edge interrupt
    // Save data on buffer
    if(!testbit(MStatus, stop)) setbit(DMA.CH1.CTRLA,4);    // Transfer request
}

// Bit banging to decode the TX pin:
// The interrupt monitors the TX line at four times the baud rate,
// when the start bit is sensed, the timer period is set to the baud rate
// then all the incoming characters are processed inside the interrupt.
// Before exiting, the timer is set to 4 times the baud rate again.
ISR(TCC1_OVF_vect) {
    uint8_t i, data=0, paritybit=0;

    // Start Bit
    if(testbit(VPORT2.IN, 3)) return;
    TCC1.INTFLAGS = 0x01;   // Clear overflow flag
    while(!testbit(TCC1.INTFLAGS,0));   // wait for next time slice
    // Set actual baud rate
    TCC1.PER = T.LOGIC.baud;

    // Data bits
    for(i=0; i<T.LOGIC.databits; i++) {
        TCC1.INTFLAGS = 0x01;   // Clear overflow flag
        while(!testbit(TCC1.INTFLAGS,0));   // wait for next time bit
        data=data>>1;
        if(testbit(VPORT2.IN, 3)) {
            data|=0x80;
            paritybit++;
        }
    }
    for(i=T.LOGIC.databits; i<8; i++) data=data>>1;
    // Check Parity
    if(testbit(Sniffer,parmode)) {
        TCC1.INTFLAGS = 0x01;   // Clear overflow flag
        while(!testbit(TCC1.INTFLAGS,0));   // wait for next time bit
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
    TCC1.INTFLAGS = 0x01;   // Clear overflow flag
    while(!testbit(TCC1.INTFLAGS,0));   // wait for next time bit
    if(!testbit(VPORT2.IN, 3))
        goto exituart;       // stop bit error

    // Second Stop bit
    if(testbit(Sniffer,stopbit)) {
        TCC1.INTFLAGS = 0x01;   // Clear overflow flag
        while(!testbit(TCC1.INTFLAGS,0));   // wait for next time bit
        if(!testbit(VPORT2.IN, 3))
            goto exituart;  // stop bit error
    }

    T.LOGIC.addr_ack_pos=data;   // data holder for DMA
    // Save data on buffer
    if(!testbit(MStatus, stop)) setbit(DMA.CH1.CTRLA,4);    // Transfer request

exituart:
    TCC1.PER = T.LOGIC.baud/4;
    TCC1.INTFLAGS = 0x01;   // Clear overflow flag
}
