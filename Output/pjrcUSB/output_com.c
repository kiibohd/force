/* Copyright (C) 2011-2015 by Jacob Alexander
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

// ----- Includes -----

// Compiler Includes
#include <Lib/OutputLib.h>

// Project Includes
#include <cli.h>
#include <led.h>
#include <print.h>

// USB Includes
#if defined(_mk20dx128_) || defined(_mk20dx128vlf5_) || defined(_mk20dx256_) || defined(_mk20dx256vlh7_)
#include "arm/usb_dev.h"
#include "arm/usb_serial.h"
#endif

// Local Includes
#include "output_com.h"



// ----- Macros -----



// ----- Function Declarations -----



// ----- Variables -----

// Indicates whether the Output module is fully functional
// 0 - Not fully functional, 1 - Fully functional
// 0 is often used to show that a USB cable is not plugged in (but has power)
volatile uint8_t  Output_Available = 0;



// ----- Capabilities -----

void Output_flashMode_capability( uint8_t state, uint8_t stateType, uint8_t *args )
{
	// Display capability name
	if ( stateType == 0xFF && state == 0xFF )
	{
		print("Output_flashMode(usbCode)");
		return;
	}

	// Start flash mode
	Output_firmwareReload();
}



// ----- Functions -----

// USB Module Setup
inline void Output_setup()
{
	// Initialize the USB
	// If a USB connection does not exist, just ignore it
	// All usb related functions will non-fatally fail if called
	// If the USB initialization is delayed, then functionality will just be delayed
	usb_init();
}


// Sets the device into firmware reload mode
inline void Output_firmwareReload()
{
	usb_device_reload();
}


// USB Input buffer available
inline unsigned int Output_availablechar()
{
	return usb_serial_available();
}


// USB Get Character from input buffer
inline int Output_getchar()
{
	// XXX Make sure to check output_availablechar() first! Information is lost with the cast (error codes) (AVR)
	return (int)usb_serial_getchar();
}


// USB Send Character to output buffer
inline int Output_putchar( char c )
{
	return usb_serial_putchar( c );
}


// USB Send String to output buffer, null terminated
inline int Output_putstr( char* str )
{
#if defined(_at90usb162_) || defined(_atmega32u4_) || defined(_at90usb646_) || defined(_at90usb1286_) // AVR
	uint16_t count = 0;
#elif defined(_mk20dx128_) || defined(_mk20dx128vlf5_) || defined(_mk20dx256_) || defined(_mk20dx256vlh7_) // ARM
	uint32_t count = 0;
#endif
	// Count characters until NULL character, then send the amount counted
	while ( str[count] != '\0' )
		count++;

	return usb_serial_write( str, count );
}


// Soft Chip Reset
inline void Output_softReset()
{
	usb_device_software_reset();
}

