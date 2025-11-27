#include <avr/io.h>
#include <util/delay.h>
#include <util/crc16.h>
#include <avr/interrupt.h>
#include "main.h"
#include "mso.h"
#include "logic.h"
#include "display.h"
#include "awg.h"
#include "interface.h"
#include "usb\usb_xmega.h"

#define SOH     0x01    // start of heading
#define STX     0x02    // start of text
#define EOT     0x04    // end of transmission
#define ACK     0x06    // acknowledge
#define NAK     0x15    // negative acknowledge
#define CAN     0x18    // cancel

// Monochrome 128x64 bitmap header
const uint8_t BMP[62] PROGMEM = {
    0x42,0x4D,0x3E,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x28,0x00,
    0x00,0x00,0x80,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,
    0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0x00 };

static inline void SendBMP(void);

static uint8_t read(void);

typedef struct _fifo {
    uint8_t	idx_w;      // write index
    uint8_t	idx_r;      // read index
    uint8_t	count;
    uint8_t buff[8];
} FIFO;

static volatile FIFO txfifo;

ISR(USARTE0_RXC_vect) {
    uint8_t i=0,n;
    RTC.CNT=0;  // Clear screen saver timer
    n=ProcessCommand(USARTE0.DATA, 0);          // Process command
    for(i=0; i<n; i++) send(ep0_buf_in[i]);     // Send response
}

// Waits for a character from the serial port for a certain time, 
// If no character is received, it returns 0
static uint8_t read(void) {
    uint16_t timeout=25000; // 250 ms
    while(!testbit(USARTE0.STATUS,USART_RXCIF_bp)) {
        _delay_us(10);
        if(--timeout==0) return 0;
    }
    return USARTE0.DATA;
}

// Test pixel on display buffer
static inline uint8_t test_pixel(uint8_t x, uint8_t y) {
    if(Disp_send.display_data[((uint16_t)(y<<4)&0xFF80) + x] & (uint8_t)(0x01 << (y & 0x07))) return 0;
    return 1;
}

static inline uint8_t transpose(uint8_t x, uint8_t y) {
    uint8_t data=0;
	for(uint8_t i=0; i<8; i++) {
		data=data<<1;
        data  |= test_pixel(x+i,y); 
	}
    return data;
}

// Send data from LCD to PC via RS-232 using the XMODEM protocol
static inline void SendBMP(void) {
    uint8_t rx,data,Packet=1, n=0,i,j;
    uint16_t crc=0;

    // First Block
    send(SOH);
    send(1);		// Send Packet number
    send(254);		// Send Packet number 1's complement
    // Send BMP Header
    for(i=0; i<62; i++) {
        n++;
        data = pgm_read_byte_near(BMP+i);
        crc=_crc_xmodem_update(crc,data);
        send(data);
    }

    // Send LCD data, 64 lines
    for(i=63; i!=255; i--) {
        // Each LCD line consists of 16 bytes
        for(j=0; j<16; j++) {
            data=transpose(j*8,i);
            crc=_crc_xmodem_update(crc,data);
            send(data);
            n++;
            if(n==128)  {   // end of 128byte block
                n=0;
                send(hibyte(crc));
                send(lobyte(crc));
                // Wait for ACK
                rx = read();
                if(rx!=ACK) return; // Error -> cancel transmission
                Packet++;
                send(SOH);
                send(Packet);		// Send Packet number
                send(255-Packet);	// Send Packet number 1's complement
                crc=0;
            }
        }
    }

    // End of last block
    for(; n<128; n++) { // send remainder of block
        data= 0x1A;     // pad with 0x1A which marks the end of file
        crc=_crc_xmodem_update(crc,data);
        send(data);
    }
    send(hibyte(crc));
    send(lobyte(crc));
    rx = read();
    if(rx!=ACK) return;
    send(EOT);
    rx = read();
    if(rx!=NAK) return;
    send(EOT);
    rx = read();
    if(rx!=ACK) return;
}

// Put a character in the transmit queue
void send (uint8_t d) {
    uint8_t i = txfifo.idx_w;
    // Check if buffer is full, if so wait...
    while(txfifo.count >= sizeof(txfifo.buff));
    txfifo.buff[i++] = d;
    cli();
    txfifo.count++;
    USARTE0.CTRLA = USART_RXCINTLVL0_bm | USART_DREINTLVL_gm;  // Enable DRE high level interrupt
    sei();
    if(i >= sizeof(txfifo.buff)) i = 0;
    txfifo.idx_w = i;
}

// UART UDRE interrupt
ISR(USARTE0_DRE_vect) {
    uint8_t n;
    n = txfifo.count;
    if(n) {
        txfifo.count = --n;
        uint8_t i = txfifo.idx_r;
        USARTE0.DATA = txfifo.buff[i++];
        if(i >= sizeof(txfifo.buff)) i = 0;
        txfifo.idx_r = i;
    }
    else USARTE0.CTRLA = USART_RXCINTLVL0_bm;  // All data sent, disable DREIF interrupt
}

// Returns the amount of data that needs to be sent from the ep0 buffer
uint8_t ProcessCommand(uint8_t Command, uint8_t usb) {
	uint8_t *p;
	uint8_t i=0,n=0;
    uint8_t index,value;
    USB_Request_Header_t* req = (void *) ep0_buf_out;
    RTC.CNT=0;  // Clear screen saver timer    
    OFFGRN();   // In case the LED was on
    switch(Command) {
        case 'a':   // Send firmware version
            p = (uint8_t *)(VERSION+3);
			for(;i<4;i++) ep0_buf_in[i]=pgm_read_byte(p++); // Read from the Version string
            n=4;
        break;
		case 'b':	// Write byte to GPIO or M
            if(usb) {
                index=lobyte(req->wIndex);
                value=lobyte(req->wValue);
            }
		    else {
                index=read();   // Read index
		        value=read();   // Read value
            }
            // Write to the corresponding register
            if(index==0 && Srate>10) clrbit(MStatus, triggered);    // prevents bad wave when changing from 20 to 10ms/div
            if(index<=5 || index==35 || index==38 || index==12 || index==13 ||
            (index>=24 && index<=28))  setbit(MStatus, update);      // Changing trigger
            if(index<=13) {
                T.IN.METER.Freq = 0;			// Prevent sending outdated data
                setbit(MStatus, updatemso);		// Settings are changing
            }
            if(index>=36) setbit(MStatus, updateawg);
            if(index<12) p=(uint8_t *)index;	    // Accessing GPIO
            else {
                index-=12;
                p=(uint8_t *)(&M);			// Accessing M
                p+=index;
            }
            *p=value;		                // Write data
        break;
		case 'c':	// Set Frequency (4 bytes)
			p=(uint8_t *)&M.AWGdesiredF;
            if(usb) {
       			*p++=lobyte(req->wIndex);
				*p++=hibyte(req->wIndex);
				*p++=lobyte(req->wValue);
				*p++=hibyte(req->wValue);
            } else for(;i<4;i++) *p++=read();
			setbit(MStatus, updateawg);
        break;
		case 'd':   // Store settings
            OFFRED();
		    SaveEE();
		break;
		case 'e':   // Save AWG in RAM to EE
            OFFRED();
		    eeprom_write_block(AWGBuffer, EEwave, 256);
		break;
        case 'f':   // Stop
            setbit(MStatus,update);
            setbit(MStatus,stop);
		break;
        case 'g':   // Start
            setbit(MStatus,update);
            clrbit(MStatus,stop);
		break;
        case 'h':   // Force Trigger
            setbit(MStatus,update);
            setbit(MStatus,triggered);
		break;
        case 'i':   // Auto Setup
            setbit(MStatus,update);
            AutoSet();
		break;            
		case 'j':	// Set Post Trigger (2 bytes)
		    p=(uint8_t *)&M.Tpost;
            if(usb) {
                *p++=lobyte(req->wValue);
                *p++=hibyte(req->wValue);
            } else {
		        *p++=read();
		        *p++=read();
            }                
            CheckPost();    // Check Post trigger value
            setbit(MStatus,update);
		break;
        case 'k':   // Restore factory settings
            Restore();
            setbit(MStatus,update);
            setbit(MStatus, updatemso);     // Apply settings
            setbit(MStatus, updateawg);     // Generate wave                       
        break;        
        case 'm':   // Send METER measurement
            p = (uint8_t *)(&T.IN.METER.Freq);
            for(uint8_t i=0; i<4; i++) ep0_buf_in[i]=*p++;
            n=4;
        break;            
        case 'p': clrbit(Misc,autosend); break;    // Do not automatically send data to UART
        case 'q': setbit(Misc,autosend); break;    // Automatically send data to UART
        case 'u':   // Send settings to PC
            p=(uint8_t *)0;  for(; i<12; i++) ep0_buf_in[i]=*p++;   // GPIO
            p=(uint8_t *)&M; for(; i<44; i++) ep0_buf_in[i]=*p++;   // M
            n=44;
        break;
        case 'w':   // Send waveform stored in EE
            do { send(eeprom_read_byte(EEwave+i)); } while(++i);
        break;
        case 'x':   // Read data and store in AWG buffer
            send('G');   // confirmation
            p=AWGBuffer;
            do { *p++ = read(); } while(++i);
            setbit(MStatus, updateawg);
            send('T');   // confirmation
        break;
        case 'C': SendBMP(); break; // Send BMP
		case 0xBB: // disconnect from USB, jump to bootloader
			USB_ep0_in_start(0);
			USB_ep0_wait_for_complete();
			cli();
			delay_ms(10);
			USB.CTRLB &= ~USB_ATTACH_bm;    // disconnects the device from the USB lines
			delay_ms(100);
			RST.STATUS = 0xFF;      // Clear reset flags to signal bootloader
			void (*enter_bootloader)(void) = (void*) 0x4000; /*0x8000/2*/ //(void*) 0x47fc /*0x8ff8/2*/;
			enter_bootloader();
		break;        
    }
    return n;
}
