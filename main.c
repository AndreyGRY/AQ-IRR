/* Name: main.c
 * Project: Data input based on AVR USB driver with TINY45
 * V3: 2 inputs ADC2/PB4 ADC3/PB3, pas de switch, led sur PB1 (on=0)
 * Author: Jacques Lepot
 * Creation Date: 2008-04-19
 * Copyright: (c) 2008 by Jacques LEpot
 * License: Proprietary, free under certain conditions. 

	Project on Gamepad source
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv/usbdrv.h"
#include "oddebug.h"

#include "RC_func/ir_funcs.h"

/*
Pin assignment:
project - PB3 = LED output 
project - PB1, PB2 = USB data lines 
*/

#define BIT_LED 3

#ifndef NULL
#define NULL    ((void *)0)
#endif


	//	test
#define USB_ACTIVE 	0
#define USB_SLEEPING 	1
#define USB_WAKING 	2

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[8];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */
uint16_t   key, lastKey = 0, repeats=1;

uint8_t  pulse_counts = 0, nec_state = 0, nec_ok = 0;
uint8_t  command = 0, inv_command;

unsigned int address;
unsigned long nec_code;

/* ------------------------------------------------------------------------- */


const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { // USB report descriptor 

	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x01,					   // Report ID (1)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     Report Count (8)
    0x75, 0x01,                    //     Report Size (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs) ,1 byte
    0x95, 0x01,                         //     Report Count (1)
    0x75, 0x08,                         //     Report Size (8)
    0x81, 0x01,                         //     Input (Constant) reserved byte(1)
    0x95, 0x05,                         //	   Report Count (5)	(4?)
    0x75, 0x08,                         //     Report Size (8)
    0x15, 0x00, 		    //   LOGICAL_MINIMUM (0)
    0x25, 0x70,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07, 		    //   USAGE_PAGE (Key codes)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	0xC0,    						/* End Collection                           		*/

	    // system controls, like power, needs a 3rd different report and report descriptor
	    0x05, 0x01,             // USAGE_PAGE (Generic Desktop)
	    0x09, 0x80,             // USAGE (System Control)
	    0xA1, 0x01,             // COLLECTION (Application)
	    0x85, 0x02, 	//   REPORT_ID
	    0x95, 0x01,             //   REPORT_COUNT (1)
	    0x75, 0x02,             //   REPORT_SIZE (2)
	    0x15, 0x01,             //   LOGICAL_MINIMUM (1)
	    0x25, 0x03,             //   LOGICAL_MAXIMUM (3)
	    0x09, 0x82,             //   USAGE (System Sleep)
	    0x09, 0x81,             //   USAGE (System Power)
	    0x09, 0x83,             //   USAGE (System Wakeup)
	    0x81, 0x60,             //   INPUT
	    0x75, 0x06,             //   REPORT_SIZE (6)
	    0x81, 0x03,             //   INPUT (Cnst,Var,Abs)
	    0xC0                   // END_COLLECTION
};    /* End Collection                           		*/


	
/* 	Modified!
   We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

static const uint8_t/*uint16_t*/  keyReport/*[46]*/[61][2] PROGMEM = {

		//	Table from Trueconf config
		//NUMBER IR_CODE HID_USB_CODE	//Keyboard analog (?/!) IC Button
		/*0 */{	0,			0x2E},      // = key ! zoom+ !
		/*1 */{	0xD8,		0x66},		//Power ! Power button
		/*2 */{	0x78,		0x3B},		//F2 KEY ! Mute button
		/*3 */{	0x20,		0x2D},		// - key ! Zoom- !
		/*4 */{	0xC2,		0x3A},		// F1 key ! Menu button !
		/*5 */{	0x94,		0x45},		// F12 key ! Home button !
		/*6 */{	0xC8,		0x4A},		// Home key ! PC button !
		/*7 */{	0x30,		0x1E},		//1
		/*8 */{	0x88,		0x1F},		//2
		/*9 */{	0x88,		0x04},		//A
		/*10*/{	0x88,		0x05},		//B
		/*11*/{	0x88,		0x06},		//C
		/*12*/{	0x70,		0x20},		//3
		/*13*/{	0x70,		0x07},		//D
		/*14*/{	0x70,		0x08},		//E
		/*15*/{	0x70,		0x09},		//F
		/*16*/{	0x08,		0x21},		//4
		/*17*/{	0x08,		0x0A},		//G
		/*18*/{	0x08,		0x0B},		//H
		/*19*/{	0x08,		0x0C},		//I
		/*20*/{	0xA8,		0x22},		//5
		/*21*/{	0xA8,		0x0D},		//J
		/*22*/{	0xA8,		0x0E},		//K
		/*23*/{	0xA8,		0x0F},		//L
		/*24*/{	0x48,		0x23},		//6
		/*25*/{	0x48,		0x10},		//M
		/*26*/{	0x48,		0x11},		//N
		/*27*/{	0x48,		0x12},		//O
		/*28*/{	0x28,		0x24},		//7
		/*29*/{	0x28,		0x13},		//P
		/*30*/{	0x28,		0x14},		//Q
		/*31*/{	0x28,		0x15},		//R
		/*32*/{	0x28,		0x16},		//S
		/*33*/{	0x98,		0x25},		//8
		/*34*/{	0x98,		0x17},		//T
		/*35*/{	0x98,		0x18},		//U
		/*36*/{	0x98,		0x19},		//V
		/*37*/{	0x68,		0x26},		//9
		/*38*/{	0x68,		0x1A},		//W
		/*39*/{	0x68,		0x1B},		//X
		/*40*/{	0x68,		0x1C},		//Y
		/*41*/{	0x68,		0x1D},		//Z
		/*42*/{	0xB8,		0x27},		//0
		/*43*/{	0xB8,		0x2C},		//Spacebar
		/*44*/{	0x18,		0x37},		//"." key
		/*45*/{	0x18,		0x25},		//"*" key	(Shift + 8)
		/*46*/{	0x58,		0x20},		//# key	(Shift + 3)
		/*47*/{	0x58,		0x1F},		//@ key	(Shift + 2)
		/*48*/{	0x54,		0x2A},		//Backspace ! Backspace button
		/*49*/{	0xA0,		0x52},		//up key
		/*50*/{	0x10,		0x50},		//left key
		/*51*/{	0x50,		0x4F},		//right key
		/*52*/{	0xB0,		0x51},		//down key
		/*53*/{	0x90,		0x28},		//Enter (Return) ! Enter button
		/*54*/{	0x40,		0x42},		//F9 key ! Call button
		/*55*/{	0xF8,		0x29},		//Escape ! Return button
		/*56*/{	0xF0,		0x41},		//F8 key ! Decline button
		/*57*/{	0x60,		0x30},		//keypad SQBRAK RIGHT ! Vol+ button
		/*58*/{	0x80,		0x2F},		//keypad SQBRAK LEFT ! Vol- button
		/*59*/{	0x82,		0x2C},		// Spacebar ! Far/Near button !
		/*60*/{	0xD0,		0x3C},		// F3! LAYOUT !
		};


/*
Modifiers:
Bit	Bit Length	Description
0	1	Left Ctrl.
1	1	Left Shift.
2	1	Left Alt.
3	1	Left GUI (Windows/Super key.)
4	1	Right Ctrl.
5	1	Right Shift.
6	1	Right Alt.
7	1	Right GUI (Windows/Super key.)*/

/* ------------------------------------------------------------------------- */
//	project
static void buildReport(/*uint16_t key*/void)
	{
	reportBuffer[0] = 0x01;	// Keyboard Report ID
	//reportBuffer[1] = 0;	//Modifier keys status
	reportBuffer[2] = 0;	//Reserved field
	//reportBuffer[3] = 0x06;	//	Keypress #1
	reportBuffer[4] = 0;	//	Keypress #2
	reportBuffer[5] = 0;	//	Keypress #3
	reportBuffer[6] = 0;	//	Keypress #4
	reportBuffer[7] = 0;	//	Keypress #5
	//reportBuffer[7] = 0;		//	Keypress #6
	
	// This (not so elegant) cast saves us 10 bytes of program memory 
	//*(uint16_t *)reportBuffer = pgm_read_word(lastKey);
	}

/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
												   /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }
        //	right ver
//		if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
//			if(rq->wValue.word == 0x0101) {
//				usbMsgPtr = (usbMsgPtr_t)(uchar *)&reportBuffer;
//				return sizeof(reportBuffer);
//			} else if(rq->wValue.word == 0x0102) {
//				usbMsgPtr = (usbMsgPtr_t)(uchar *)&reportBuffer2;
//				return 2;
//			}

        		//	set leds?
//								else if(rq->bRequest == USBRQ_HID_SET_REPORT){
//											if (rq->wLength.word == 2) {
//												/* We expect two-byte reports */
//												expectReport = 1;
//												return USB_NO_MSG; /* Call usbFunctionWrite with data */
//											}
//								}
        else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

//	set leds in example?
///* usbFunctionWrite() is called when the host sends a chunk of data to the
// * device. For more information see the documentation in usbdrv/usbdrv.h.
// */
//uchar usbFunctionWrite(uchar *data, uchar len)
//{
//	if (expectReport && (len == 2) && data[0] == 1) {
//		if (data[1] & 1) PORTD|=(1<<7);
//		else PORTD&=~(1<<7);
//		if (data[1] & 2) PORTD|=(1<<6);
//		else PORTD&=~(1<<6);
//		if (data[1] & 4) PORTD|=(1<<5);
//		else PORTD&=~(1<<5);
//	}
//	expectReport = 0;
//	return 1;
//}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}
	
void all_init() //static inline 
{      	
	GIMSK |= (1 << INT0) | (1 << PCIE);		//(!)GIMSK(6) enables INT0 interrupt, GIMSK(5) enables PCINT[0:5] changes interrupt ! NO FCK, GIMSK(5)-PCIE
	MCUCR |= (1 << ISC00) | (0 << ISC01);	//The rising edge of INT0 generates an interrupt request.
	TIMSK = (1 << TOIE0);	//enable timer0 overrun interrupt
	PCMSK |= (1 << PCINT0);    //PB0 turn on interrupts on pins PB0, PB1, &amp;amp; PB4
	TCCR0B |= (1 << 2)|(1 << 0);	// set	16.5MHz/1024 prescaler
	TCCR0B &= ~(1 << 1);
    //TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
    DDRB |= 1 << BIT_LED;   /* output for LED */
    DDRB &= ~(1 << PB0);   /* input for TSOP */
	PORTB |= 1 << PB0;   /* poll input down */    
	sei();	//== SREG(1) = 1 - let the global interrupt
	//GIFR(5) - set if PCINT[0:5] changes, GIFR(6) - if INT0 pin changes (GIMSK is needed)
	ir_start_timer();
}

void ir_start_timer()//static
{
	TIMSK = (1 << TOIE0);	//enable timer0 overrun interrupt
	TCNT0 = 0;	// timer counter = 0
	TCCR0B |= (1 << 2)|(1 << 0);	// set	16.5MHz/1024 prescaler
	TCCR0B &= ~(1 << 1);}

void remote_read() {
unsigned int timer_value;
  if(nec_state != 0){
    timer_value = TCNT0;                         // Store Timer1 value
    TCNT0 = 0;                                   // Reset Timer1
  }
  switch(nec_state){
   case 0 :                                      // Start receiving IR data (we're at the beginning of 9ms pulse)
    TCNT0  = 0;                                  // Reset Timer1
	TCCR0B |= (1 << 2)|(1 << 0);	// set	16.5MHz/1024 prescaler
	TCCR0B &= ~(1 << 1);
	nec_state = 1;                               // Next state: end of 9ms pulse (start of 4.5ms space)
    pulse_counts = 0;
    return;
   case 1 :                                      // End of 9ms pulse
    if((timer_value > 152) || (timer_value < 138)){         // Invalid interval ==> stop decoding and reset

    	nec_state = 0;                             // Reset decoding process
      TCCR0B = 0;                                // Disable Timer1 module
    }
    else
      nec_state = 2;                             // Next state: end of 4.5ms space (start of 562µs pulse)
    return;
   case 2 :                                      // End of 4.5ms space
    if((timer_value > 80) || (timer_value < 65)){

    	repeats++;	//	repeat sequence if 2.2 ms space

      nec_state = 0;                             // Reset decoding process
      TCCR0B = 0;                                // Disable Timer1 module
    }
    else
      nec_state = 3;                             // Next state: end of 562µs pulse (start of 562µs or 1687µs space)
    return;
   case 3 :                                      // End of 562µs pulse
    if((timer_value > 11) || (timer_value < 7)){           // Invalid interval ==> stop decoding and reset
      TCCR0B = 0;                                // Disable Timer1 module
      nec_state = 0;                             // Reset decoding process
    }
    else
      nec_state = 4;                             // Next state: end of 562µs or 1687µs space
    return;
   case 4 :                                      // End of 562µs or 1687µs space
    if((timer_value > 29) || (timer_value < 7)){           // Time interval invalid ==> stop decoding
      TCCR0B = 0;                                // Disable Timer1 module
      nec_state = 0;                             // Reset decoding process
      return;
    }
    if( timer_value > 16)                     // If space width > 1ms (short space)
	{
		nec_code |= (1 << (31 - pulse_counts) );
	}
    else                                        // If space width < 1ms (long space)
	{
		nec_code &= ~(1 << (31 - pulse_counts) );
	}
    pulse_counts++;
    if(pulse_counts > 31){                                  // If all bits are received

    nec_ok = 1;                                // Decoding process OK
	GIMSK &= ~(1 << PCIE);		
				  return;
    }
    nec_state = 3;                               // Next state: end of 562µs pulse (start of 562µs or 1687µs space)
  }
}

ISR(PCINT0_vect) 	//ISR interrupt handler (pin rising down)
{
remote_read();
}	

ISR(TIMER0_OVF_vect)	//ISR interrupt handler (timer0 overrun)
{       
  	nec_state = 0;                                 // Reset decoding process
	TCCR0B = 0;
}
// âíåøíåå ïðåðûâàíèå ïî ôðîíòó è ñïàäó

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
	uint16_t   key, lastKey = 0, last_repeats = 1;	//	saves decoded keys and long presses with more than 5 pulses
	uchar   state = 0;
	uchar   i, idleCounter = 0, keyDidChange = 0, old_sof = usbSofCount;
	uchar   calibrationValue;
	char usb_sleep = USB_ACTIVE;

	all_init();
	ir_start_timer();

	int tmp = 0, ir_address = 0, ir_code = 0, alt_counter = 0;
	uint8_t _address = 0, _code = 0, isPowerSleep = 0;
	uint32_t timer_poll_switchkey = 0, timer_poll_repeats = 0, timer_poll_no_usbSOF = 0;				//	no other timers, use increment loops for button striking delay & counting repeats
																			//	timer_poll = 0 - stop
																			//	timer_poll = 1 - now started, increment in the end
																			//	timer_poll = var - var loops gone, increment in the end

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
   // odDebugInit();
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
        _delay_ms(15);
    }
    usbDeviceConnect();
    //wdt_enable(WDTO_1S);
    usbInit();
    sei();
    while (1)
	{    /* main event loop */

        //wdt_reset();
        usbPoll();

		// during normal operation, a SOF marker is sent by the host every 1ms
		// if no SOF markers are received for a longer period of time, it
		// indicates that the USB bus has been suspended (or disconnected, but
		// then we lose power as well)

		if(usbSofCount == old_sof)
		{
			// I was too lazy to use a timer, so I count how many iterations of
			// the main loop have passed without a SOF marker
			timer_poll_no_usbSOF++;
			if(timer_poll_no_usbSOF > 30 /*&& usb_sleep == USB_ACTIVE*/)	//125ms long?
				usb_sleep = USB_SLEEPING;
		}
		else
		{
			old_sof = usbSofCount;
			timer_poll_no_usbSOF = 0;
			usb_sleep = USB_ACTIVE;
		}

		if(usb_sleep != USB_ACTIVE) {
			if(usb_sleep == USB_SLEEPING && _code) {
				// send USB remote wakeup by signalling SE0 (both D+ and D- low) for 10 ms
				unsigned char tmp0 = PORTB;
				unsigned char tmp1 = DDRB;

				usbPoll();

				cli(); // disable interrupts, prevent V-USB from interfering
				PORTB=(tmp0 & ~(1<<USB_CFG_DMINUS_BIT)) | (1<<USB_CFG_DPLUS_BIT);
				DDRB|=((1<<USB_CFG_DPLUS_BIT)|(1<<USB_CFG_DMINUS_BIT));
				for(i=0;i<10;i++) {
					//wdt_reset();
					_delay_ms(1);
				}
				PORTB = tmp0;
				DDRB = tmp1;
				usb_sleep = USB_WAKING;
				sei();

		        usbPoll();	//	test
			}
		}

		GIMSK |= (1 << PCIE);		//  enables IR receiver

			for (uint8_t k = 0; k < 6; k++)		//	125! ms with enabled interrupts (108 ms - repeat sequences period).
			{
		        usbPoll();
				_delay_ms(25);
				if (repeats > last_repeats)
					{
					last_repeats = repeats;
					k = 0;
					}
			}
			usbPoll();



 //       if (!nec_state)
        	GIMSK &= ~(1 << PCIE);

    	if (repeats>1)
    		repeats--;
    	if (repeats>5)
    		repeats=5;

		if ( (state && usbInterruptIsReady() && repeats<=1) )		// if key released
		{
			state = 0;
			repeats = 1;
			last_repeats = 1;
			for (int t=1; t<8; t++) reportBuffer[t] = 0;
			nec_state = 0;
			usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
		}

		if(nec_ok || timer_poll_switchkey > 1)
		{                                    // If the mcu receives NEC message with successful
			if (nec_ok && timer_poll_switchkey==0)
				{
				command = nec_code >> 8;
				nec_state = 0;
				TCCR0B = 0;                                  // Disable Timer1 module
				}

				//	IR-Decoder, long light pulse - 1, short - 0
//						PORTB |= 1 << BIT_LED;
//						_delay_ms(1000);
//						PORTB &= ~(1 << BIT_LED);
//						_delay_ms(500);
//				for (int k=0; k<32; k++)
//				{
//					if ( command & (1 << k) ) 				// 1	//check IR command in NEC
//					{
//						PORTB |= 1 << BIT_LED;
//						_delay_ms(1000);
//						PORTB &= ~(1 << BIT_LED);
//						_delay_ms(1000);
//					}
//					else if ( command | (0 << k) ) 				// 	0
//					{
//						PORTB |= 1 << BIT_LED;
//						_delay_ms(100);
//						PORTB &= ~(1 << BIT_LED);
//						_delay_ms(1000);
//					}
//				}
//						PORTB &= ~(1 << BIT_LED);
//						_delay_ms(500);
//						PORTB |= 1 << BIT_LED;
//						_delay_ms(1000);
//						PORTB &= ~(1 << BIT_LED);
//						_delay_ms(500);

				//	IR-Checker
//			for (uint8_t testcode = 0x0; testcode < 0xFF; testcode++){
//				while (!usbInterruptIsReady()) {;}
//				PORTB |= 1 << BIT_LED;
//				for (uint8_t k = 0; k < 50; k++)
//				{usbPoll();
//				_delay_ms(40);}
//				PORTB &= ~(1 << BIT_LED);
//				for (uint8_t k = 0; k < 50; k++)
//				{usbPoll();
//				_delay_ms(40);}

//			if (!nec_state)
//				GIMSK &= ~(1 << PCIE);

					for(int j=0; j<=60; j++)
					{
						if ( command == pgm_read_byte(&keyReport[j][0]) )
						{
//							if ( j>7 && j<=47 )										// multifunc button
//							{

								//	If Trueconf, comment next:
//								if ((j+alt_counter==45) || (j+alt_counter==46) || (j+alt_counter==47))	// +shift button
//									reportBuffer[1] = 0b00000010;	//Modifier keys status
//								else reportBuffer[1] = 0;
//
//									//	Switching digits/letters with queves of button repeats. No need for Trueconf, it's switching works inside.
//								if (timer_poll_switchkey == 0)	// is first
//									{
//										lastKey = pgm_read_byte(&keyReport[j][1]);
//										alt_counter = 0;
//										timer_poll_switchkey++;							//	start _timer_
//										break;
//									}
//								if (timer_poll_switchkey<50 && timer_poll_switchkey>2 && nec_ok)									// choose multibutton
//								{
//									if (lastKey == pgm_read_byte(&keyReport[j+alt_counter][1]) )	// is not first
//									{
//
//										if ((j>=37 && j<42) || (j>27 && j<=32) )		// 7pqrs, 9wxyz
//											{if (alt_counter > 3) alt_counter = 0;		//offset of letter position
//											else alt_counter++;}
//										else if (j>=42 && j<=47)						// *., 0 , #@
//											{if (alt_counter > 0) alt_counter = 0;		//offset of letter position
//											else alt_counter++;}
//										else
//											{if (alt_counter > 2) alt_counter = 0;		// 1,..6,8,a...o,...
//											else alt_counter++;}
//									}
//
//									lastKey = pgm_read_byte(&keyReport[j+alt_counter][1]);
//									reportBuffer[3] = lastKey;
//									break;
//								}
//								else
//									{
//									if (timer_poll_switchkey>=50)		//	end _timer_
//										{
//										//lastKey = pgm_read_byte(&keyReport[j+alt_counter][1]);	// end waiting, send
//										alt_counter = 0;
//										timer_poll_switchkey = 0;							//	start _timer_
//										reportBuffer[3] = lastKey;
//										_code = 1;
//										}
//									break;
//									}
//							}

								//	If not Trueconf, comment next:

																	if ((j+alt_counter==46) || (j+alt_counter==47))	//	need for getting start of quevue, else index on the end
																	{
																		if (timer_poll_switchkey == 0)	// is first
																		{
																			lastKey = pgm_read_byte(&keyReport[j][1]);
																			alt_counter = 0;
																			timer_poll_switchkey++;							//	start _timer_
																			break;
																		}
																		if (timer_poll_switchkey<20 && timer_poll_switchkey>2 && nec_ok)									// choose multibutton
																		{
																			if (lastKey == pgm_read_byte(&keyReport[j+alt_counter][1]) )	// is not first
																			{
																				if (alt_counter > 0) alt_counter = 0;		//offset of letter position
																				else alt_counter++;
																			}

																			lastKey = pgm_read_byte(&keyReport[j+alt_counter][1]);
																			reportBuffer[1] = 0b00000010;	//Modifier keys status
																			reportBuffer[3] = lastKey;
																			break;
																		}
																		else
																			if (timer_poll_switchkey>=20)		//	end _timer_
																				{
																				lastKey = pgm_read_byte(&keyReport[j+alt_counter][1]);	// end waiting, send
																				alt_counter = 0;
																				timer_poll_switchkey = 0;							//	end _timer_
																				reportBuffer[1] = 0b00000010;	//Modifier keys status
																				reportBuffer[3] = lastKey;
																				_code = 1;
																				break;
																				}
																	}
																	else 	//	end _timer_
																		{
																		lastKey = pgm_read_byte(&keyReport[j][1]);	// end waiting, send
																		alt_counter = 0;
																		timer_poll_switchkey = 0;							//	end _timer_
																		reportBuffer[3] = lastKey;
																		_code = 1;
																		if ((j+alt_counter==46) || (j+alt_counter==47)|| (j+alt_counter==48))
																			reportBuffer[1] = 0b00000010;	//Modifier keys status
																		else
																			reportBuffer[1] = 0;	//Modifier keys status
																		break;
																	}
															}
					}

			if (timer_poll_switchkey > 1 || timer_poll_switchkey==0) 								// if first time, one cycle more
				{
				nec_ok = 0;                                  // Reset decoding process
				address = 0;
				nec_code = 0;
				}
		}

		if ((_code == 1 && timer_poll_switchkey==0 && !state) || (repeats > 1))
			{
			state = 1;

				if (usbInterruptIsReady())
					{
					if (reportBuffer[3] == 0x66)
						{
							//isPowerSleep = 1;

							reportBuffer[0] = 0x2;	// ID
							reportBuffer[1] = 0x1;	// SysCtrl Sleep

							usbSetInterrupt(reportBuffer, /*sizeof(reportBuffer)*/2);

							while(!usbInterruptIsReady());

							reportBuffer[1] = 0;
							usbSetInterrupt(reportBuffer, /*sizeof(reportBuffer)*/2);

						state = 0;
						}
					else
						usbSetInterrupt(reportBuffer, /*sizeof(reportBuffer)*/8);

					}
			timer_poll_switchkey = 0;					// stop timer
			buildReport();
			alt_counter = 0;
			_code = 0;
			command = -1;
			}

		if (timer_poll_switchkey > 0) timer_poll_switchkey++;

    }
    return 0;
}
