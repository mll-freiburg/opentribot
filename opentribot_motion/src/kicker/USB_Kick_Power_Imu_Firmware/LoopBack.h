/*
			 LUFA Library
	 Copyright (C) Dean Camera, 2009.
			  
  dean [at] fourwalledcubicle [dot] com
	  www.fourwalledcubicle.com
*/

/*
  Copyright 2009  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for CDC.c.
 */

#ifndef _CDC_H_
#define _CDC_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <string.h>

		#include "Descriptors.h"

		#include <LUFA/Version.h>							// Library Version Information
		#include <LUFA/Drivers/USB/USB.h>			// USB Functionality
//		#include <LUFA/Drivers/Board/LEDs.h>		// LEDs driver

		#include "servo.h"
	/* Defines */
		#define Status_USBNotReady			0	/**< USB is not ready (disconnected from a USB host) */
		#define Status_USBEnumerating		1	/**< USB interface is enumerating */
		#define Status_USBReady				2	/**< USB interface is connected and ready */


	/* Missing WinAVR include defines */
	/* WinAVR does not define these defines for the ATmega??u4*/
	#if (defined(__AVR_ATmega16U4__)  || defined(__AVR_ATmega32U4__))
		#define PB7	7
		#define PB6	6
		#define PB5	5
		#define PB4	4
		#define PB3	3
		#define PB2	2
		#define PB1	1
		#define PB0	0
		#define PC7	7
		#define PC6	6
		#define PD7	7
		#define PD6	6
		#define PD5	5
		#define PD4	4
		#define PD3	3
		#define PD2	2
		#define PD1	1
		#define PD0	0
		#define PE2	2
		#define PE6	6
		#define PF7	7
		#define PF6	6
		#define PF5	5
		#define PF4	4
		#define PF1	1
		#define PF0	0
	#endif

	/* Tasks: */
		TASK(MainTask);

	/* Global Variables: */
		extern uint8_t dataToSend[];
		extern uint8_t dataReceived[];
		extern uint16_t timerval;

	/* Function Prototypes: */
		void EVENT_USB_Connect(void);
		void EVENT_USB_Disconnect(void);
		void EVENT_USB_ConfigurationChanged(void);
		void EVENT_USB_UnhandledControlPacket(void);

		void UpdateStatus(uint8_t CurrentStatus);

#endif
