#include <avr/io.h>
#include <util/delay.h>
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

// The screenshot as a byte stream: the 62 byte BMP header, then the display
// transposed into 64 rows of 16 bytes, bottom row first (BMP order). Shared by
// the XMODEM transfer below and the USB block read in ProcessCommand('C').
#define BMP_SIZE    (62 + 1024)
static uint8_t bmp_byte(uint16_t o) {
    if(o < 62) return pgm_read_byte_near(BMP + o);
    o -= 62;
    return transpose((uint8_t)((o & 15) * 8), (uint8_t)(63 - (o >> 4)));
}

// Send data from LCD to PC via RS-232 using the XMODEM protocol: 128 byte blocks,
// the last one padded with 0x1A (end of file).
// The CRC module's CRC-16 is CRC-16/XMODEM (poly 0x1021, MSB first)
static inline void SendBMP(void) {
    uint8_t data, Packet=1;
    uint16_t o=0;
    for(;;) {
        send(SOH);
        send(Packet);           // Send Packet number
        send(255-Packet);       // Send Packet number 1's complement
        CRC.CTRL = CRC_RESET_RESET0_gc;     // CHECKSUM = 0
        CRC.CTRL = CRC_SOURCE_IO_gc;        // CRC on data written to DATAIN
        for(uint8_t k=0; k<128; k++, o++) {
            data = (o < BMP_SIZE) ? bmp_byte(o) : 0x1A;
            CRC.DATAIN = data;
            send(data);
        }
        send(CRC.CHECKSUM1);
        send(CRC.CHECKSUM0);
        if(read()!=ACK) return; // Error -> cancel transmission
        if(o >= BMP_SIZE) break;
        Packet++;
    }
    send(EOT);
    if(read()!=NAK) return;
    send(EOT);
    if(read()!=ACK) return;
}

// Put a character in the transmit queue
void send (uint8_t d) {
    uint8_t i;
    for(;;) {   // Wait for room in the buffer, letting the DRE interrupt drain the queue
        cli();  // Room check and txfifo updates need to occur together with updates disabled
        i = txfifo.idx_w;
        if(txfifo.count < sizeof(txfifo.buff)) break;
        sei();
    }
    txfifo.buff[i++] = d;
    if(i >= sizeof(txfifo.buff)) i = 0;
    txfifo.idx_w = i;
    txfifo.count++;
    USARTE0.CTRLA = USART_RXCINTLVL0_bm | USART_DREINTLVL_gm;  // Enable DRE high level interrupt
    sei();
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
    const USB_Request_Header_t* req = (void *) ep0_buf_out;
    RTC.CNT=0;  // Clear screen saver timer    
    OFFGRN();   // In case the LED was on
    // Commands that read or send on USARTE0 run for the serial port only: over USB they
    // would block the control request on the UART (CR-039), and with autosend on that can
    // starve the main loop into a watchdog reset (CR-045). Their replies go to the UART
    // anyway, so a USB host has no use for them: it reads SCOPE_DATA, and the sniffer's
    // decoded buffers while Sniff() runs, from bulk endpoint 1.
    if(!usb) switch(Command) {
        case 's':   // Send SCOPE_DATA: CH1, CH2, CHD, frame, index
            p=(uint8_t *)T.SCOPE.DC.CH1data;
            for(uint16_t i=0; i<sizeof(SCOPE_DATA); i++) send(*p++);
        break;
        case 'w':   // Send waveform stored in EE
            do { send(eeprom_read_byte(EEwave+i)); } while(++i);
        break;
        case 'x':   // Read data and store in AWG buffer
            send('G');   // confirmation
            p=AWGBuffer;
            do { *p++ = read(); } while(++i);
            send('T');   // confirmation
        break;
        case 'C':   // Send BMP, XMODEM
            SendBMP();
        break;
    }
    else if(Command=='C') {
        // Over USB the same BMP stream is read in 64 byte blocks, wIndex = block number
        // (0..16; the last one is 62 bytes), answered from ep0_buf_in at once
        uint16_t o = req->wIndex * 64;
        for(; i<64 && o<BMP_SIZE; i++, o++) ep0_buf_in[i] = bmp_byte(o);
        return i;
    }
#ifdef TESTFIXTURE
    // Bench commands, test-fixture build only, and USB only: the parameters come from
    // the setup packet and the reply is ep0_buf_in, so nothing waits on USARTE0 and
    // the serial port keeps the shipping command set. Over the UART they are unknown
    // bytes (ignored).
    //
    //   'K'  wValue = keys             ->  Misc, Index, Menu, Buttons (GPIORC..F)
    //        Simulated key press, so the fixture can walk the menus and every other
    //        user interaction ('b' cannot: index 12+ maps to M, so GPIORC..F are out of
    //        its reach). keys = the PORTA pins held down, as on the board: bit 1 MENU,
    //        bit 2, bit 3 K2, bit 4 (bits 2 and 4 are K1/K3, swapped by Display.flip).
    //        The pattern goes through KeyInput(), the key ISR's own decode, so the press
    //        raises userinput and update exactly like a debounced key. 0 = release (as
    //        the ISR sees it on key up); bit 7 set = only report the four registers.
    //   'r'  wIndex = address, wValue = count (1..64)  ->  data space (SRAM, I/O)
    //   'R'  wIndex = offset,  wValue = count (1..64)  ->  EEPROM
    //        Read-only by design: the fixture checks what a command actually wrote
    //        (AWGBuffer, TCD1.PER, calibration tables) instead of inferring it from the
    //        analog output, and it must not be able to write DUT memory this way.
    else if(Command=='K') {
        index = lobyte(req->wValue);
        if(!(index & 0x80)) KeyInput((uint8_t)(~index & 0x1E));
        p=(uint8_t *)&Misc; for(; i<4; i++) ep0_buf_in[i]=*p++;   // GPIORC..F
        return 4;
    }
    else if(Command=='r' || Command=='R') {
        uint16_t a = req->wIndex;
        n = lobyte(req->wValue);
        if(n > sizeof(ep0_buf_in)) n = sizeof(ep0_buf_in);
        for(; i<n; i++, a++)
            ep0_buf_in[i] = (Command=='r') ? *(uint8_t *)a : eeprom_read_byte((const uint8_t *)a);
        return n;
    }
#endif
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
            // Check index range: index selects GPIO0..11 and then M[0..sizeof(M)-1];
            if(index >= 12 + sizeof(NVMVAR)) break;
            // Write the byte first, then raise the request bits: they are requests, not
            // settings, so a write to MStatus (index 11) cannot throw them away (CR-037).
            // 'pending' carries the requests that were already queued across that write; for
            // any other index nothing cleared them, so the OR below is a no-op.
            uint8_t pending = MStatus & (_BV(update)|_BV(updatemso)|_BV(updateawg));
            if(index<12) p=(uint8_t *)index;	    // Accessing GPIO
            else p=(uint8_t *)(&M) + (index-12);  // Accessing M (leaves 'index' for the tests)
            *p=value;		                // Write data
            MStatus |= pending;                 // requests queued before this write survive it
            if(index==0 && Srate>10) clrbit(MStatus, triggered);    // prevents bad wave when changing from 20 to 10ms/div
            if(index<=5 || index==35 || index==38 || index==12 || index==13 ||
            (index>=24 && index<=28))  setbit(MStatus, update);      // Changing trigger
            T.SCOPE.MeterFreq = 0;              // Any write invalidates the cached meter reading
            if(index<=13 || index==28) {        // Only settings that need Apply()
                setbit(MStatus, updatemso);		// Settings are changing
            }
            if(index>=36) setbit(MStatus, updateawg);
            if(index==8) T.SCOPE.adjusting=0;   // Mode change: cancel auto setup (as RestorefromMeter does)
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
            T.SCOPE.adjusting=0;            // Don't auto setup over the factory settings
            setbit(MStatus,update);
            setbit(MStatus, updatemso);     // Apply settings
            setbit(MStatus, updateawg);     // Generate wave                       
        break;
        case 'm':   // Send METER measurement
            p = (uint8_t *)(&T.SCOPE.MeterFreq);
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
		case 0xBB: // disconnect from USB, jump to bootloader
            // USB_ep0_wait_for_complete
			USB_ep0_in_start(0);
            endpoints[0].out.STATUS &= ~(USB_EP_SETUP_bm | USB_EP_TRNCOMPL0_bm | USB_EP_BUSNACK0_bm);
            while (!(endpoints[0].out.STATUS & USB_EP_TRNCOMPL0_bm) && !(endpoints[0].in.STATUS & USB_EP_TRNCOMPL0_bm)){};
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
