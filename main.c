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

