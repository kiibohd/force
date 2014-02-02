/* Copyright (C) 2011-2014 by Jacob Alexander
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
#include <Lib/delay.h>
#include <Lib/MainLib.h>

// Project Includes
#include <output_com.h>

#include <cli.h>
#include <led.h>
#include <print.h>



// ----- Defines -----



// ----- Macros -----



// ----- Function Declarations -----

void cliFunc_distRead    ( char* args );
void cliFunc_free        ( char* args );
void cliFunc_gaugeHelp   ( char* args );
void cliFunc_imadaComm   ( char* args );
void cliFunc_single      ( char* args );
void cliFunc_start       ( char* args );
void cliFunc_zeroForce   ( char* args );
void cliFunc_zeroPosition( char* args );


// ----- Variables -----

// Force Gauge command dictionary
char*       forceGaugeCLIDictName = "Force Gauge Commands";
CLIDictItem forceGaugeCLIDict[] = {
	{ "distRead",      "Read the current value from the distance gauge.  See \033[35mgaugeHelp\033[0m for more details.", cliFunc_distRead },
	{ "free",          "Enables free reporting, reports every distance unit (as defined by the calipers).", cliFunc_free },
	{ "gaugeHelp",     "Description on how to use the force gauge firmware.", cliFunc_gaugeHelp },
	{ "imadaComm",     "Send specific commands to the Imada force gauge. See \033[35mgaugeHelp\033[0m for more details.", cliFunc_imadaComm },
	{ "single",        "Query a single force/distance measurement.", cliFunc_single },
	{ "start",         "Mark the current distance as the start/end position.", cliFunc_start },
	{ "zeroForce",     "Zero out the force gauge.", cliFunc_zeroForce },
	{ "zeroPosition",  "Mark the minimum distance for this measurement (bottom).", cliFunc_zeroPosition },
	{ 0, 0, 0 } // Null entry for dictionary end
};

uint8_t force_freeRunning;


// ----- Functions -----

// Initial Pin Setup, make sure they are sane
inline void pinSetup()
{
	// Distance Sensor Pin Setup
	// 22 - D6 - Green - Data
	// 23 - C2 - White - Clk
	// Enable pins
	GPIOD_PDDR = 0;
	//GPIOD_PDDR &= ~(1<<6); // Input
	GPIOC_PDDR |=  (1<<2); // Output

	PORTD_PCR6 = PORT_PCR_PFE | PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(1);

}


// UART Setup for Imada Force gauge
inline void uartSetup()
{
	// Setup the the UART interface for keyboard data input
	SIM_SCGC4 |= SIM_SCGC4_UART0; // Disable clock gating

	// Pin Setup for UART0
	PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); // RX Pin
	PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // TX Pin

#define DS2_1
#ifdef DPS_1R
	// DPS-1R - Uses a much slower baud rate
	// Setup baud rate - 2400 Baud
	// 48 MHz / ( 16 * Baud ) = BDH/L
	// Baud: 2400 -> 48 MHz / ( 16 * 2400 ) = 1250
	// Thus baud setting = 1250
	// NOTE: If finer baud adjustment is needed see UARTx_C4 -> BRFA in the datasheet
	uint16_t baud = 1250; // Max setting of 8191
#elif defined(DS2_1)
	// DS2-1 - Uses a more modern baud rate
	// Setup baud rate - 19200 Baud
	// 48 MHz / ( 16 * Baud ) = BDH/L
	// Baud: 19200 -> 48 MHz / ( 16 * 19200 ) = 156.25
	// Thus baud setting = 156
	uint16_t baud = 156; // Max setting of 8191
#else
	erro_print("Force gauge not set...please recompile.");
#endif
	UART0_BDH = (uint8_t)(baud >> 8);
	UART0_BDL = (uint8_t)baud;

	// 8 bit, No Parity, Idle Character bit after stop
	// NOTE: For 8 bit with Parity you must enable 9 bit transmission (pg. 1065)
	//       You only need to use UART0_D for 8 bit reading/writing though
	// UART_C1_M UART_C1_PE UART_C1_PT UART_C1_ILT
	UART0_C1 = UART_C1_ILT;

	// Number of bytes in FIFO before TX Interrupt
	UART0_TWFIFO = 1;

	// Number of bytes in FIFO before RX Interrupt
	UART0_RWFIFO = 1;

	// TX FIFO Disabled, RX FIFO Enabled
	// UART_PFIFO_TXFE UART_PFIFO_RXFE
	UART0_C2 = 0; // Has to be cleared before setting PFIFO (see docs)
	UART0_PFIFO = UART_PFIFO_RXFE;

	// TX/RX FIFO Size:
	//  0x0 - 1 dataword
	//  0x1 - 4 dataword
	//  0x2 - 8 dataword
	//   etc. (see docs)
#ifdef FIFO_DEBUG
	dbug_msg("FIFO Sizes TX: ");
	printHex( UART0_PFIFO & 0x70 >> 4 );
	print(" RX: ");
	printHex( UART0_PFIFO & 0x07 );
	print( NL );
#endif

	// Reciever Inversion Disabled, LSBF
	// UART_S2_RXINV UART_S2_MSBF
	UART0_S2 |= 0x00;

	// Transmit Inversion Disabled
	// UART_C3_TXINV
	UART0_C3 |= 0x00;

	// TX Enabled, RX Enabled
	// UART_C2_TE UART_C2_RE UART_C2_RIE
	UART0_C2 = UART_C2_RE | UART_C2_TE;

	// Add interrupt to the vector table
	NVIC_ENABLE_IRQ( IRQ_UART0_STATUS );
}


// Force gauge setup
inline void forceSetup()
{
	// Configuring Pins
	pinSetup();
	init_errorLED();

	// Setup Output Module
	output_setup();

	// UART Setup
	uartSetup();

	// Enable CLI
	init_cli();

	// Register Force Gauge dictionary
	registerDictionary_cli( forceGaugeCLIDict, forceGaugeCLIDictName );

	// Initialization of force gauge variables
	force_freeRunning = 0;
}


// Main execution function
int main()
{
	// Setup force gauge
	forceSetup();

	// Main loop
	while ( 1 )
	{
		// Process CLI
		process_cli();

		// Query force
		// TODO

		// Read Distance
		// TODO

		// Read force
		// TODO

		// Output data
		// TODO
	}
}


// Send each character from the string (except the NULL), waiting until each character is sent before proceeding
// NOTE: There is no delay after sending the last character
//       If first character is NULL, it will be sent
inline void transmitUART0String( char* str )
{
	// Loop through string
	while ( *str != '\0' )
	{
		while( !( UART0_S1 & UART_S1_TC ) ); // Wait for dataword to transmit
		UART0_D = *str++; // Send dataword (character)
	}
}

// Wait until FIFO has a character (no timeout...)
inline char receiveUART0Char()
{
	// Wait for a response (something to arrive in the FIFO)
	while ( UART0_SFIFO & UART_SFIFO_RXEMPT );

	// Read from RX FIFO
	return UART0_D;
}


// ----- Interrupt Functions -----



// ----- CLI Command Functions -----

void cliFunc_distRead( char* args )
{
	// Prepare to print output
	print( NL );
	info_msg("Distance: ");

	// Data
	uint32_t distInput = 0;

	// Setup distance read parameters for iGaging Distance Scale
	//       freq = 9kHz
	// duty_cycle = 20%
	// high_delay = (1/freq) *       (duty_cycle/100)
	//  low_delay = (1/freq) * ((100-duty_cycle)/100)
	uint8_t  bits       = 21; // 21 clock pulses, for 21 bits
	//uint32_t high_delay = 22; // Clock high time per pulse
	//uint32_t  low_delay = 89; // Clock low  time per pulse
	uint32_t high_delay = 40; // Clock high time per pulse
	uint32_t  low_delay = 60; // Clock low  time per pulse

	// Make sure clock is low initially
	GPIOC_PCOR |= (1<<2); // Set Clock low
/*
while(1)
{
*/
	// Scan each of the bits
	for ( uint8_t bit = bits; bit > 0; bit-- )
	{
		// Begin clock pulse
		GPIOC_PSOR |= (1<<2); // Set Clock high

		// Delay for duty cycle
		delayMicroseconds( high_delay );

		// End clock pulse
		GPIOC_PCOR |= (1<<2); // Set Clock low

		// Read Data Bit
		//distInput |= GPIOD_PDIR & (1<<6) ? (1 << (bit - 1)) : 0;
		//if ( GPIOD_PDIR )
		if ( GPIOD_PDIR & (1<<6) )
		{
			print("1");
		}
		else
		{
			print("0");
		}

		// Delay for duty cycle
		delayMicroseconds( low_delay );
	}
	print("  ");

	// Output result
	printInt32( distInput );

	// Convert to mm
	// As per http://www.shumatech.com/web/21bit_protocol?page=0,1
	// 21 bits is 2560 CPI (counts per inch) (C/inch)
	// 1 inch is 25.4 mm
	// 2560 / 25.4 = 100.7874016... CPMM (C/mm)
	// Or
	// 1 count is 1/2560 = 0.000390625... inches
	// 1 count is (1/2560) * 25.4 = 0.0000153789370078740 mm = 0.0153789370078740 um = 15.3789370078740 nm
	// Since there are 21 bits (2 097 152 positions) converting to um is possible by multiplying by 1000
	//    which is 2 097 152 000, and within 32 bits (4 294 967 295).
	// However, um is still not convenient, so 64 bits (18 446 744 073 709 551 615) is a more accurate alternative.
	// For each nm there are 2 097 152 000 000 positions.
	// And for shits:
	//    pm is 2 097 152                 :          0.000 015 378 937 007 874 0 mm : 32 bit
	//    pm is 2 097 152 000             :          0.015 378 937 007 874 0     um : 32 bit (ideal acc. for 32 bit)
	//    pm is 2 097 152 000 000         :         15.378 937 007 874 0         nm : 64 bit
	//    pm is 2 097 152 000 000 000     :     15 378.937 007 874 0             pm : 64 bit
	//    fm is 2 097 152 000 000 000 000 : 15 378 937.007 874 0                 fm : 64 bit (ideal acc. for 64 bit)
	//uint64_t distNM = distInput * 15;
	//uint64_t distPM = distInput * 15378;
	uint64_t distFM = distInput * 15378937;

	// Calculate um and mm
	//uint32_t distNM = distInput * 15; // XXX
	//uint32_t distUM = distNM / 1000;
	//uint32_t distMM = distNM / 1000000;
	uint32_t distNM = distFM * 1000000;
	uint32_t distUM = distNM / 1000;
	uint32_t distMM = distUM / 1000;

	print("  ");
	printInt32( distMM );
	print(" mm  ");
	printInt32( distUM );
	print(" um  ");
	printInt32( distNM );
	print(" nm");

/*
//Wait
print(NL);
delay( 7 );
distInput = 0;
}
*/
}


void cliFunc_free( char* args )
{
}


void cliFunc_imadaComm( char* args )
{
	// Parse command from arguments
	//  NOTE: Only first argument is used
	char* arg1Ptr;
	char* arg2Ptr;
	argumentIsolation_cli( args, &arg1Ptr, &arg2Ptr );

	// Error out if no argument specified
	if ( *arg1Ptr == '\0' )
	{
		print( NL );
		erro_print("No argument specified...");
		return;
	}

	// Write command to data register
	//  NOTE: Imada only uses single byte commands, this makes it easy
	//        Always followed by a '\r' CR
	char outCmd[] = { *arg1Ptr, '\r', 0x00 };
	transmitUART0String( outCmd );

	// Prepare to print output
	print( NL );
	info_msg("Imada: ");
	char inputChar[] = { 0x00, 0x00 };

	// Read until a CR is read
	while ( 1 )
	{
		inputChar[0] = receiveUART0Char();

		// Stop reading if the character was a CR
		if ( inputChar[0] == '\r' )
		{
			print("[CR]");
			break;
		}

		// Print out the character
		dPrint( inputChar );
	}
}


void cliFunc_gaugeHelp( char* args )
{
}


void cliFunc_single( char* args )
{
}


void cliFunc_start( char* args )
{
}


void cliFunc_zeroForce( char* args )
{
}


void cliFunc_zeroPosition( char* args )
{
}

