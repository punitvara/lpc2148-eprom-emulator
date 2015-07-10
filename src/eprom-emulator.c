/*
  lpc2148-eprom-emulator

  An 27-series EPROM emulator based on a LPC2148 board.

  Based on LPCUSB code for USB boilerplate communication.
  
  Copyright (C) 2015 Cecill Etheredge / ijsf (c@ijsf.nl)
  Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  3. The name of the author may not be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
  Minimal implementation of a USB serial port, using the CDC class.
  This example application simply echoes everything it receives right back
  to the host.

  Windows:
  Extract the usbser.sys file from .cab file in C:\WINDOWS\Driver Cache\i386
  and store it somewhere (C:\temp is a good place) along with the usbser.inf
  file. Then plug in the LPC214x and direct windows to the usbser driver.
  Windows then creates an extra COMx port that you can open in a terminal
  program, like hyperterminal.

  Linux:
  The device should be recognised automatically by the cdc_acm driver,
  which creates a /dev/ttyACMx device file that acts just like a regular
  serial port.
*/

#include "eprom.h"

#include <string.h>			// memcpy

#include "type.h"
#include "usbdebug.h"

#include "armVIC.h"

#include "console.h"
#include "usbapi.h"
#include "startup.h"

#include "serial_fifo.h"
#include "LPC214x.h"

#define BAUD_RATE	115200

#define INT_IN_EP		0x81
#define BULK_OUT_EP		0x05
#define BULK_IN_EP		0x82

#define MAX_PACKET_SIZE	64

#define LE_WORD(x)		((x)&0xFF),((x)>>8)

// CDC definitions
#define CS_INTERFACE			0x24
#define CS_ENDPOINT				0x25

#define	SET_LINE_CODING			0x20
#define	GET_LINE_CODING			0x21
#define	SET_CONTROL_LINE_STATE	0x22

#define	INT_VECT_NUM	0

#define IRQ_MASK 0x00000080

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
  U32		dwDTERate;
  U8		bCharFormat;
  U8		bParityType;
  U8		bDataBits;
} TLineCoding;

static BOOL USB_Online = FALSE;
static TLineCoding LineCoding = {115200, 0, 0, 8};
static U8 abBulkBuf[64];
static U8 abClassReqData[8];

static U8 txdata[VCOM_FIFO_SIZE];
static U8 rxdata[VCOM_FIFO_SIZE];

static fifo_t txfifo;
static fifo_t rxfifo;

// forward declaration of interrupt handler
static void USBIntHandler(void) __attribute__ ((interrupt("IRQ")));


static const U8 abDescriptors[] = {

  // device descriptor
  0x12,
  DESC_DEVICE,
  LE_WORD(0x0101),			// bcdUSB
  0x02,						// bDeviceClass
  0x00,						// bDeviceSubClass
  0x00,						// bDeviceProtocol
  MAX_PACKET_SIZE0,			// bMaxPacketSize
  LE_WORD(0xFFFF),			// idVendor
  LE_WORD(0x0005),			// idProduct
  LE_WORD(0x0100),			// bcdDevice
  0x01,						// iManufacturer
  0x02,						// iProduct
  0x03,						// iSerialNumber
  0x01,						// bNumConfigurations

  // configuration descriptor
  0x09,
  DESC_CONFIGURATION,
  LE_WORD(67),				// wTotalLength
  0x02,						// bNumInterfaces
  0x01,						// bConfigurationValue
  0x00,						// iConfiguration
  0xC0,						// bmAttributes
  0x32,						// bMaxPower
  // control class interface
  0x09,
  DESC_INTERFACE,
  0x00,						// bInterfaceNumber
  0x00,						// bAlternateSetting
  0x01,						// bNumEndPoints
  0x02,						// bInterfaceClass
  0x02,						// bInterfaceSubClass
  0x01,						// bInterfaceProtocol, linux requires value of 1 for the cdc_acm module
  0x00,						// iInterface
  // header functional descriptor
  0x05,
  CS_INTERFACE,
  0x00,
  LE_WORD(0x0110),
  // call management functional descriptor
  0x05,
  CS_INTERFACE,
  0x01,
  0x01,						// bmCapabilities = device handles call management
  0x01,						// bDataInterface
  // ACM functional descriptor
  0x04,
  CS_INTERFACE,
  0x02,
  0x02,						// bmCapabilities
  // union functional descriptor
  0x05,
  CS_INTERFACE,
  0x06,
  0x00,						// bMasterInterface
  0x01,						// bSlaveInterface0
  // notification EP
  0x07,
  DESC_ENDPOINT,
  INT_IN_EP,					// bEndpointAddress
  0x03,						// bmAttributes = intr
  LE_WORD(8),					// wMaxPacketSize
  0x0A,						// bInterval
  // data class interface descriptor
  0x09,
  DESC_INTERFACE,
  0x01,						// bInterfaceNumber
  0x00,						// bAlternateSetting
  0x02,						// bNumEndPoints
  0x0A,						// bInterfaceClass = data
  0x00,						// bInterfaceSubClass
  0x00,						// bInterfaceProtocol
  0x00,						// iInterface
  // data EP OUT
  0x07,
  DESC_ENDPOINT,
  BULK_OUT_EP,				// bEndpointAddress
  0x02,						// bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
  0x00,						// bInterval
  // data EP in
  0x07,
  DESC_ENDPOINT,
  BULK_IN_EP,					// bEndpointAddress
  0x02,						// bmAttributes = bulk
  LE_WORD(MAX_PACKET_SIZE),	// wMaxPacketSize
  0x00,						// bInterval
	
  // string descriptors
  0x04,
  DESC_STRING,
  LE_WORD(0x0409),

  0x0E,
  DESC_STRING,
  'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

  0x14,
  DESC_STRING,
  'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

  0x12,
  DESC_STRING,
  'D', 0, 'E', 0, 'A', 0, 'D', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,

  // terminating zero
  0
};


/**
Local function to handle incoming bulk data
		
@param [in] bEP
@param [in] bEPStatus
*/
static void BulkOut(U8 bEP, U8 bEPStatus)
{
  int i, iLen;

  if (fifo_free(&rxfifo) < MAX_PACKET_SIZE) {
    // may not fit into fifo
    return;
  }

  // get data from USB into intermediate buffer
  iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
  for (i = 0; i < iLen; i++) {
    // put into FIFO
    if (!fifo_put(&rxfifo, abBulkBuf[i])) {
      // overflow... :(
      ASSERT(FALSE);
      break;
    }
  }
}


/**
Local function to handle outgoing bulk data
		
@param [in] bEP
@param [in] bEPStatus
*/
static void BulkIn(U8 bEP, U8 bEPStatus)
{
  int i, iLen;

  if (fifo_avail(&txfifo) == 0) {
    // no more data, disable further NAK interrupts until next USB frame
    USBHwNakIntEnable(0);
    return;
  }

  // get bytes from transmit FIFO into intermediate buffer
  for (i = 0; i < MAX_PACKET_SIZE; i++) {
    if (!fifo_get(&txfifo, &abBulkBuf[i])) {
      break;
    }
  }
  iLen = i;
	
  // send over USB
  if (iLen > 0) {
    USBHwEPWrite(bEP, abBulkBuf, iLen);
  }
}


/**
Local function to handle the USB-CDC class requests
		
@param [in] pSetup
@param [out] piLen
@param [out] ppbData
*/
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
  switch (pSetup->bRequest) {

    // set line coding
    case SET_LINE_CODING:
    memcpy((U8 *)&LineCoding, *ppbData, 7);
    *piLen = 7;
    break;

    // get line coding
    case GET_LINE_CODING:
    *ppbData = (U8 *)&LineCoding;
    *piLen = 7;
    break;

    // set control line state
    case SET_CONTROL_LINE_STATE:
    // bit0 = DTR, bit = RTS
    break;

    default:
    return FALSE;
  }
  return TRUE;
}


/**
Initialises the VCOM port.
Call this function before using VCOM_putchar or VCOM_getchar
*/
void VCOM_init(void)
{
  fifo_init(&txfifo, txdata);
  fifo_init(&rxfifo, rxdata);
}


/**
Writes one character to VCOM port
	
@param [in] c character to write
@returns character written, or EOF if character could not be written
*/
int VCOM_putchar(int c)
{
  if( USB_Online )
  {
    return fifo_put(&txfifo, c) ? c : EOF;
  }
  else
  {
    return -1;
  }
}


/**
Reads one character from VCOM port
	
@returns character read, or EOF if character could not be read
*/
int VCOM_getchar(void)
{
  U8 c;
	
  return fifo_get(&rxfifo, &c) ? c : EOF;
}


/*
  Below console functions simply redirect all console traffic to USB serial console
*/
int putchar(int ch)
{
  return VCOM_putchar(ch);
}

int getchar()
{
  return VCOM_getchar();
}

int puts(const char *s)
{
	while (*s) {
		putchar(*s++);
	}
	putchar('\n');
	return 1;
}


/**
Interrupt handler
	
Simply calls the USB ISR, then signals end of interrupt to VIC
*/
static void USBIntHandler(void)
{
  USBHwISR();
  VICVectAddr = 0x00;    // dummy write to VIC to signal end of ISR 	
}


static void USBFrameHandler(U16 wFrame)
{
  if (fifo_avail(&txfifo) > 0) {
    // data available, enable NAK interrupt on bulk in
    USBHwNakIntEnable(INACK_BI);
  }
}


static inline BOOL eprom_oe_rd()
{
  return (FIO0PIN & 0b100000000000000000) > 0;
}
static inline BOOL eprom_ce_rd()
{
  return (FIO0PIN & 0b010000000000000000) > 0;
}
static inline U16 eprom_address_rd()
{
  // read address from P0.0 and P0.15, excluding P0.14
  U16 data = (FIO0PIN & 0b1011111111111111);
  return ((data & 0b1000000000000000) >> 1) | (data & 0b0011111111111111);
}
static inline void eprom_data_wr(U8 data)
{
  // write data to P1.16 to P1.23
  // mask off these pins to avoid overriding other values
  FIO1MASK = ~(0b111111110000000000000000);
  FIO1PIN = data << 16;
  
  // reset mask
  FIO1MASK = 0;
}

static void eprom_wait_request_start()
{
  // busy wait for CE and OE to be pulled low
  while(!(!eprom_oe_rd() && !eprom_ce_rd()));
}

static void eprom_wait_request_end()
{
  // busy wait for CE and OE to be pulled high
  while(!(eprom_oe_rd() && eprom_ce_rd()));
}

static void eprom_emulate()
{
  // emulates an EPROM and prints any debugging information to the USB serial console
  
  printf("Starting EPROM emulation (%s series)", sizeof(eprom_bin) == 32768 ? "27256" : (sizeof(eprom_bin) == 16384 ? "27128" : "Unknown") );
  
  // main loop
  while(1)
  {
    eprom_wait_request_start();
    
    // address lines should be valid, so read out address
    U16 address = eprom_address_rd();
    
    // perform bounds check
    if(address < sizeof(eprom_bin))
    {
      // set up data lines with the appropriate data
      printf("Requested address 0x%x\n", address);
      eprom_data_wr(eprom_bin[address]);
    }
    else
    {
      // invalid address
      printf("Requested address 0x%x is out of bounds (firmware size: 0x%x)\n", address, sizeof(eprom_bin));
    }

    // wait for CE and OE to be pulled high
    eprom_wait_request_end();
  };
}

static void eprom_initialize()
{
  // enable fast GPIO on ports 0 and 1
  SCS |= 0x00000003;
  
  // enable all GPIO pins on ports 0 and 1
  PINSEL0 = 0;
  PINSEL1 = 0;
  
  // mask all pins as writable
  FIO0MASK = 0;
  
  // set P0.0 to P0.15 (corresponding to A0 to A15 on EPROM) as inputs
  // this excludes P0.14, which is used as an ISR pin by the LPC2148
  FIO0DIR &= ~(0b1011111111111111);
  
  // set P0.16 and P0.17 (corresponding to CE and OE) as inputs
  FIO0DIR &= ~(0b110000000000000000);
  
  // set P1.16 to P1.23 (corresponding to Q0 to Q7 on EPROM) as outputs
  // initialize to LOW
  FIO1DIR |= 0b111111110000000000000000;
  FIO1CLR |= 0b111111110000000000000000;
}

/*************************************************************************
main
====
**************************************************************************/
int main(void)
{
  int c;
	
  // initialize PLL and MAM
  // this assumes (and requires) a 12MHz clock on the board, and sets the board to run at 60 MHz
  Initialize();

  // initialise stack
  USBInit();

  // register descriptors
  USBRegisterDescriptors(abDescriptors);

  // register class request handler
  USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

  // register endpoint handlers
  USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
  USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
  USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);
	
  // register frame handler
  USBHwRegisterFrameHandler(USBFrameHandler);

  // enable bulk-in interrupts on NAKs
  USBHwNakIntEnable(INACK_BI);

  // initialise VCOM
  VCOM_init();

  // set up USB interrupt
  VICIntSelect &= ~(1<<22);               // select IRQ for USB
  VICIntEnable |= (1<<22);

  (*(&VICVectCntl0+INT_VECT_NUM)) = 0x20 | 22; // choose highest priority ISR slot 	
  (*(&VICVectAddr0+INT_VECT_NUM)) = (int)USBIntHandler;
	
  enableIRQ();

  // connect to bus
  USBHwConnect(TRUE);
  USB_Online = TRUE;

  // initialize eprom reader settings
  eprom_initialize();
  
  // eprom emulation loop
  eprom_emulate();

  return 0;
}

