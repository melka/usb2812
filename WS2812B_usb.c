/*
 * Modifications by Kamel 'melka' Makhloufi // blaste.net
 *   -- based off of --
 * Copyright 2011-2014 Charles Lohr
 *   -- based off of --
 * USB Keyboard Example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2009 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define USB_SERIAL_PRIVATE_INCLUDE
#include "WS2812B_usb.h"

#define NOOP asm volatile("nop" ::)

static uint8_t buf[64];

/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

// You can change these to give your code its own name.
#define STR_MANUFACTURER  L"[manufacturer]"
#define STR_PRODUCT   L"[product]"
#define STR_SERIAL   L"[serial]"

// Mac OS-X and Linux automatically load the correct drivers.  On
// Windows, even though the driver is supplied by Microsoft, an
// INF file is needed to load the driver.  These numbers need to
// match the INF file.
#define VENDOR_ID   0xabcd
#define PRODUCT_ID    0xf003

// USB devices are supposed to implment a halt feature, which is
// rarely (if ever) used.  If you comment this line out, the halt
// code will be removed, saving 102 bytes of space (gcc 4.3.0).
// This is not strictly USB compliant, but works with all major
// operating systems.
#define SUPPORT_ENDPOINT_HALT


/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

#define ENDPOINT0_SIZE    64

#define KEYBOARD_INTERFACE  0
#define KEYBOARD_ENDPOINT 3
#define KEYBOARD_SIZE   8
#define KEYBOARD_BUFFER   EP_DOUBLE_BUFFER

static const uint8_t PROGMEM endpoint_config_table[] = {
  0,
  0,
  0,
  0
};


/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

// Descriptors are the data that your computer reads when it auto-detects
// this USB device (called "enumeration" in USB lingo).  The most commonly
// changed items are editable at the top of this file.  Changing things
// in here should only be done by those who've read chapter 9 of the USB
// spec and relevant portions of any USB class specifications!


static const uint8_t PROGMEM device_descriptor[] = {
  18,         // bLength
  1,          // bDescriptorType
  0x00, 0x02,       // bcdUSB
  0,          // bDeviceClass
  0,          // bDeviceSubClass
  0,          // bDeviceProtocol
  ENDPOINT0_SIZE,       // bMaxPacketSize0
  LSB(VENDOR_ID), MSB(VENDOR_ID),   // idVendor
  LSB(PRODUCT_ID), MSB(PRODUCT_ID), // idProduct
  0xcd, 0xab,       // bcdDevice
  1,          // iManufacturer
  2,          // iProduct
  3,          // iSerialNumber
  1         // bNumConfigurations
};

#define CONFIG1_DESC_SIZE (9+9+0)

static const uint8_t PROGMEM config1_descriptor[] = {
  // configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
  9,          // bLength;
  2,          // bDescriptorType;
  LSB(CONFIG1_DESC_SIZE),      // wTotalLength
  MSB(CONFIG1_DESC_SIZE),
  1,          // bNumInterfaces
  1,          // bConfigurationValue
  0,          // iConfiguration
  0xC0,         // bmAttributes
  50,         // bMaxPower
  9,          // bLength
  4,          // bDescriptorType
  0,      // bInterfaceNumber (unused, would normally be used for HID)
  0,          // bAlternateSetting
  0,          // bNumEndpoints
  0xff,         // bInterfaceClass (0xff = vendor-specific)
  0x01,         // bInterfaceSubClass
  0x01,         // bInterfaceProtocol
  0          // iInterface
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
  uint8_t bLength;
  uint8_t bDescriptorType;
  int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
  4,
  3,
  {0x0409}
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
  sizeof(STR_MANUFACTURER),
  3,
  STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
  sizeof(STR_PRODUCT),
  3,
  STR_PRODUCT
};
static const struct usb_string_descriptor_struct PROGMEM string3 = {
  sizeof(STR_SERIAL),
  3,
  STR_SERIAL
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
static const struct descriptor_list_struct {
  uint16_t  wValue;
  uint16_t  wIndex;
  const uint8_t *addr;
  uint8_t   length;
} PROGMEM descriptor_list[] = {
  {0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
  {0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
  {0x0300, 0x0000, (const uint8_t *)&string0, 4},
  {0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
  {0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},
  {0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL)}
};
#define NUM_DESC_LIST (sizeof(descriptor_list)/sizeof(struct descriptor_list_struct))


/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

// zero when we are not configured, non-zero when enumerated
static volatile uint8_t usb_configuration=0;

/**************************************************************************
 *
 *  Public Functions - these are the API intended for the user
 *
 **************************************************************************/


// initialize USB
void usb_init(void)
{
  HW_CONFIG();
  USB_FREEZE(); // enable USB
  PLL_CONFIG();       // config PLL
  while (!(PLLCSR & (1<<PLOCK))) ;  // wait for PLL lock
  USB_CONFIG();       // start USB clock
  UDCON = 0;        // enable attach resistor
  usb_configuration = 0;
  UDIEN = (1<<EORSTE)|(1<<SOFE);
  sei();
}

// return 0 if the USB is not configured, or the configuration
// number selected by the HOST
uint8_t usb_configured(void)
{
  return usb_configuration;
}

/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/



// USB Device Interrupt - handle all device-level events
// the transmit buffer flushing is triggered by the start of frame
//
ISR(USB_GEN_vect)
{
  uint8_t intbits;
  intbits = UDINT;
  UDINT = 0;
  if (intbits & (1<<EORSTI)) {
    UENUM = 0;
    UECONX = 1;
    UECFG0X = EP_TYPE_CONTROL;
    UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
    UEIENX = (1<<RXSTPE);
    usb_configuration = 0;
  }
}



// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready(void)
{
  while (!(UEINTX & (1<<TXINI))) ;
}
static inline void usb_send_in(void)
{
  UEINTX = ~(1<<TXINI);
}
static inline void usb_wait_receive_out(void)
{
  while (!(UEINTX & (1<<RXOUTI))) ;
}
static inline void usb_ack_out(void)
{
  UEINTX = ~(1<<RXOUTI);
}


// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
ISR(USB_COM_vect)
{
  uint8_t intbits;
  const uint8_t *list;
  const uint8_t *cfg;
  uint8_t i, n, len, en;
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
  uint16_t desc_val;
  const uint8_t *desc_addr;
  uint8_t desc_length;
  static unsigned char frame;

  UENUM = 0;
  intbits = UEINTX;
  //SPIPutChar( 'x' );
  if (intbits & (1<<RXSTPI)) {
    bmRequestType = UEDATX;
    bRequest = UEDATX;
    wValue = UEDATX;
    wValue |= (UEDATX << 8);
    wIndex = UEDATX;
    wIndex |= (UEDATX << 8);
    wLength = UEDATX;
    wLength |= (UEDATX << 8);
    UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));
    if (bRequest == GET_DESCRIPTOR) {
      list = (const uint8_t *)descriptor_list;
      for (i=0; ; i++) {
        if (i >= NUM_DESC_LIST) {
          UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
          return;
        }
        desc_val = pgm_read_word(list);
        if (desc_val != wValue) {
          list += sizeof(struct descriptor_list_struct);
          continue;
        }
        list += 2;
        desc_val = pgm_read_word(list);
        if (desc_val != wIndex) {
          list += sizeof(struct descriptor_list_struct)-2;
          continue;
        }
        list += 2;
        desc_addr = (const uint8_t *)pgm_read_word(list);
        list += 2;
        desc_length = pgm_read_byte(list);
        break;
      }
      len = (wLength < 256) ? wLength : 255;
      if (len > desc_length) len = desc_length;
      do {
        // wait for host ready for IN packet
        do {
          i = UEINTX;
        } while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
        if (i & (1<<RXOUTI)) return;  // abort
        // send IN packet
        n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
        for (i = n; i; i--) {
          UEDATX = pgm_read_byte(desc_addr++);
        }
        len -= n;
        usb_send_in();
      } while (len || n == ENDPOINT0_SIZE);
      return;
    }
    if (bRequest == SET_ADDRESS) {
      usb_send_in();
      usb_wait_in_ready();
      UDADDR = wValue | (1<<ADDEN);
      return;
    }
    if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
      usb_configuration = wValue;
      usb_send_in();
      cfg = endpoint_config_table;
      for (i=1; i<5; i++) {
        UENUM = i;
        en = pgm_read_byte(cfg++);
        UECONX = en;
        if (en) {
          UECFG0X = pgm_read_byte(cfg++);
          UECFG1X = pgm_read_byte(cfg++);
        }
      }
            UERST = 0x1E;
            UERST = 0;
      return;
    }
    if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
      usb_wait_in_ready();
      UEDATX = usb_configuration;
      usb_send_in();
      return;
    }

    if (bRequest == GET_STATUS) {
      usb_wait_in_ready();
      i = 0;
      #ifdef SUPPORT_ENDPOINT_HALT
      if (bmRequestType == 0x82) {
        UENUM = wIndex;
        if (UECONX & (1<<STALLRQ)) i = 1;
        UENUM = 0;
      }
      #endif
      UEDATX = i;
      UEDATX = 0;
      usb_send_in();
      return;
    }
    #ifdef SUPPORT_ENDPOINT_HALT
    if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
      && bmRequestType == 0x02 && wValue == 0) {
      i = wIndex & 0x7F;
      if (i >= 1 && i <= MAX_ENDPOINT) {
        usb_send_in();
        UENUM = i;
        if (bRequest == SET_FEATURE) {
          UECONX = (1<<STALLRQ)|(1<<EPEN);
        } else {
          UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
          UERST = (1 << i);
          UERST = 0;
        }
        return;
      }
    }
    #endif

    // WS2812BCODE BEGIN
    if( bRequest == 0xA3 ) { //My request/Device->Host
      // wait for host ready for IN packet
      do {
        i = UEINTX;
      } while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
      if (i & (1<<RXOUTI)) return;  // abort

      n = 11;
      for (i = n; i; i--) {
        UEDATX = i;
      }
      usb_send_in();

      //printf( "0xA3 / data:\n" );
      //SPIPutChar( 'X' );
      //SPIPutChar( '\n' );
    }

    if( bRequest == 0xA4 ) { //Control-In (Us to CPU)
      usb_wait_in_ready();
      UEDATX = 4;UEDATX = 5;UEDATX = 6;UEDATX = 7;
      UEDATX = 4;UEDATX = 5;UEDATX = 6;UEDATX = 7;
      usb_send_in();
      //SPIPutChar( 'Y' );
      //SPIPutChar( '\n' );
      return;
    }

    if( bRequest == 0xA6 ) { //Jumbo  Us to CPU Frame
      unsigned short len;
      //SPIPutChar( 'y' );
      //SPIPutChar( '\n' );

      //_delay_ms(2);
      //_delay_ms(500);
      len = 128;
      do {
        // wait for host ready for IN packet
        do {
          i = UEINTX;
        } while (!(i & ((1<<TXINI)|(1<<RXOUTI))));
        if (i & (1<<RXOUTI)) return;  // abort
        // send IN packet
        n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
        for (i = n; i; i--)
        {
          UEDATX = frame++;
        }
        len -= n;
        usb_send_in();
      } while (len || n == ENDPOINT0_SIZE);
      return;
    }

    if( bRequest == 0xA5 ) { //Control-Out (CPU To Us)  (Only works on control-type messages, else we crash.)
      unsigned i, l;
      unsigned char j;
      unsigned char isPow2 = 0;
      usb_wait_receive_out();
      // SPIPutChar( 'Q' );
      // unsigned char c;// = UEDATX;
      // volatile unsigned char v;
      volatile register uint8_t v asm("r3");
      volatile register uint8_t mask asm("r2");

      DDRD |= _BV(3);
      PORTD &= ~_BV(3);

      if( ( wLength & (ENDPOINT0_SIZE-1)) == 0 ) {
        isPow2 = 1;
      }
      #define EXTRA_HOLD
      #define SEND_WS( var ) \
        mask = 0x80; \
        v = var; \
        while( mask ) { \
          if( mask & v ) { \
            PORTD |= _BV(3);  mask>>=1;   \
            NOOP; NOOP; NOOP; NOOP; \
            NOOP; NOOP; NOOP; NOOP; \
            PORTD &= ~_BV(3);\
          } else { \
            PORTD |= _BV(3); NOOP; \
            PORTD &= ~_BV(3); \
            mask>>=1; \
            NOOP; NOOP; NOOP; NOOP; \
          } \
        }

      #if ENDPOINT0_SIZE==64
        l = wLength>>6;
        int remainder = wLength & 63;
      #else
        l = wLength>>5;
        int remainder = wLength & 31;
      #endif

      for( i = 0; i < l; i++ ) {
        buf[0] = UEDATX;
        SEND_WS(buf[0]);
        buf[1] = UEDATX;
        buf[2] = UEDATX;
        buf[3] = UEDATX;
        buf[4] = UEDATX;
        SEND_WS(buf[1]);
        buf[5] = UEDATX;
        buf[6] = UEDATX;
        buf[7] = UEDATX;
        buf[8] = UEDATX;
        SEND_WS(buf[2]);
        buf[9] = UEDATX;
        buf[10] = UEDATX;
        buf[11] = UEDATX;
        buf[12] = UEDATX;
        SEND_WS(buf[3]);
        buf[13] = UEDATX;
        buf[14] = UEDATX;
        buf[15] = UEDATX;
        buf[16] = UEDATX;
        buf[17] = UEDATX;
        SEND_WS(buf[4]);
        buf[18] = UEDATX;
        buf[19] = UEDATX;
        buf[20] = UEDATX;
        buf[21] = UEDATX;
        SEND_WS(buf[5]);
        buf[22] = UEDATX;
        buf[23] = UEDATX;
        buf[24] = UEDATX;
        SEND_WS(buf[6]);
        buf[25] = UEDATX;
        buf[26] = UEDATX;
        buf[27] = UEDATX;
        SEND_WS(buf[7]);
        buf[28] = UEDATX;
        buf[29] = UEDATX;
        buf[30] = UEDATX;
        buf[31] = UEDATX;
        SEND_WS(buf[8]);
      #if ENDPOINT0_SIZE==64
        buf[32] = UEDATX;
        buf[33] = UEDATX;
        buf[34] = UEDATX;
        buf[35] = UEDATX;
        SEND_WS(buf[9]);
        buf[36] = UEDATX;
        buf[37] = UEDATX;
        buf[38] = UEDATX;
        buf[39] = UEDATX;
        SEND_WS(buf[10]);
        buf[40] = UEDATX;
        buf[41] = UEDATX;
        buf[42] = UEDATX;
        buf[43] = UEDATX;
        SEND_WS(buf[11]);
        buf[44] = UEDATX;
        buf[45] = UEDATX;
        buf[46] = UEDATX;
        buf[47] = UEDATX;
        SEND_WS(buf[12]);
        buf[48] = UEDATX;
        buf[49] = UEDATX;
        buf[50] = UEDATX;
        buf[51] = UEDATX;
        SEND_WS(buf[13]);
        buf[52] = UEDATX;
        buf[53] = UEDATX;
        buf[54] = UEDATX;
        buf[55] = UEDATX;
        SEND_WS(buf[14]);
        buf[56] = UEDATX;
        buf[57] = UEDATX;
        buf[58] = UEDATX;
        buf[59] = UEDATX;
        SEND_WS(buf[15]);
        buf[60] = UEDATX;
        buf[61] = UEDATX;
        buf[62] = UEDATX;
        buf[63] = UEDATX;
      #endif
        if( l-i > 1 )
          usb_ack_out();

      #if ENDPOINT0_SIZE != 64
        SEND_WS(buf[9]);
        SEND_WS(buf[10]);
        SEND_WS(buf[11]);
        SEND_WS(buf[12]);
        SEND_WS(buf[13]);
        SEND_WS(buf[14]);
        SEND_WS(buf[15]);
      #endif

      #if ENDPOINT0_SIZE == 64
        for( j = 16; j < 63; j++ ) {
          SEND_WS( (buf[j]) );
      #ifdef EXTRA_HOLD
          _delay_us(4);
      #endif
        }
      #else
        for( j = 16; j < 31; j++ ) {
          SEND_WS( (buf[j]) );
      #ifdef EXTRA_HOLD
          _delay_us(4);
      #endif
        }
      #endif
        if( l - i > 1 )
          usb_wait_receive_out();

      #if ENDPOINT0_SIZE == 64
        SEND_WS(buf[63]);
      #else
        SEND_WS(buf[31]);
      #endif
      }
      if( remainder ) {
        usb_ack_out();
        usb_wait_receive_out();
      }
      PORTD &= ~_BV(3);
      for( i = 0; i < remainder; i++ )
        v = UEDATX;  //read extra byte?
      usb_ack_out();
      usb_send_in();
      return;
    } else {
        // DO NOTHING
    }
  }
  UECONX = (1<<STALLRQ) | (1<<EPEN);  // stall
}