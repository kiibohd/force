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

void cliFunc_contRead    ( char* args );
void cliFunc_distRead    ( char* args );
void cliFunc_free        ( char* args );
void cliFunc_gaugeHelp   ( char* args );
void cliFunc_imadaComm   ( char* args );
void cliFunc_read        ( char* args );
void cliFunc_start       ( char* args );
void cliFunc_stop        ( char* args );
void cliFunc_zeroForce   ( char* args );
void cliFunc_zeroPosition( char* args );

char receiveUART0Char();

void transmitUART0String( char* str );

uint32_t readDistanceGauge();

void continuityTest();


// ----- Variables -----

// Force Gauge command dictionary
char*       forceGaugeCLIDictName = "Force Curve Gauge Commands";
CLIDictItem forceGaugeCLIDict[] = {
	{ "contRead",      "Read the continuity value. Needs to be attachd to two terminals on a switch. Strobe/Sense.", cliFunc_contRead },
	{ "distRead",      "Read the current value from the distance gauge.  See \033[35mgaugeHelp\033[0m for more details.", cliFunc_distRead },
	{ "free",          "Enables free reporting, reports every distance unit (as defined by the calipers).", cliFunc_free },
	{ "gaugeHelp",     "Description on how to use the force gauge firmware.", cliFunc_gaugeHelp },
	{ "imadaComm",     "Send specific commands to the Imada force gauge. See \033[35mgaugeHelp\033[0m for more details.", cliFunc_imadaComm },
	{ "read",          "Query a force/distance measurement. See \033[35mgaugeHelp\033[0m for more details.", cliFunc_read },
	{ "start",         "Mark the current distance as the start/end position, offset is optional.", cliFunc_start },
	{ "stop",          "Stop free reporting or read loop.", cliFunc_stop },
	{ "zeroForce",     "Zero out the force gauge.", cliFunc_zeroForce },
	{ "zeroPosition",  "Mark the minimum distance for this measurement (bottom).", cliFunc_zeroPosition },
	{ 0, 0, 0 } // Null entry for dictionary end
};

uint8_t force_freeRunning;
uint8_t continuityState         = 0; // 0 - None, 1 - Continuity

uint32_t distanceStart          = 0; // Offset is not used
uint32_t distanceOffset         = 0;
uint32_t forceDistanceRead      = 0;
int32_t  forceDistanceReadCount = 0;


// ----- Functions -----

// Initial Pin Setup, make sure they are sane
inline void pinSetup()
{
	// Distance Sensor Pin Setup
	// 22 - C1 - Green - Data
	// 23 - C2 - White - Clk
	// Enable pins
	GPIOC_PDDR &= ~(1<<1); // Input
	GPIOC_PDDR |=  (1<<2); // Output

	PORTC_PCR1 = PORT_PCR_PFE | PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1); // Pullup resistor
	PORTC_PCR2 = PORT_PCR_DSE | PORT_PCR_MUX(1);


	// Continuity Tester Pin Setup
	// 20 - D5 - Strobe
	// 21 - D6 - Sense
	// Enable pins
	GPIOD_PDDR |=  (1<<5); // Output
	GPIOD_PDDR &= ~(1<<6); // Input

	PORTD_PCR5 = PORT_PCR_DSE | PORT_PCR_MUX(1);
	PORTD_PCR6 = PORT_PCR_PFE | PORT_PCR_PE | PORT_PCR_MUX(1); // Pulldown resistor
}


// UART Setup for Imada Force gauge
inline void uartSetup()
{
	// Setup the the UART interface for keyboard data input
	SIM_SCGC4 |= SIM_SCGC4_UART0; // Disable clock gating

	// Pin Setup for UART0
	PORTB_PCR16 = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); // RX Pin
	PORTB_PCR17 = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // TX Pin

//#define DPS_1R
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

	// Loop variables
	uint32_t currentDistance = 0;
	uint32_t    lastDistance = 0;
	char     currentForce[40];
	uint8_t  currentForceLen = 0;
	uint8_t  startEndCount   = 0;

	// Main loop
	while ( 1 )
	{
		// Process CLI
		process_cli();

		// Only enter read loop if activated
		if ( forceDistanceRead || --forceDistanceReadCount > 0 )
		{
			// - Query force -
			// Write command to data register
			//  NOTE: Imada only uses single byte commands, this makes it easy
			//        Always followed by a '\r' CR
			char outCmd[] = { 'D', '\r', 0x00 };
			transmitUART0String( outCmd );


			// - Read Distance -
			currentDistance = readDistanceGauge();


			// - Query Continuity -
			continuityTest();


			// - Read force -
			// Read until a CR is read
			while ( 1 )
			{
				currentForce[currentForceLen] = receiveUART0Char();

				// Stop reading if the character was a CR
				if ( currentForce[currentForceLen++] == '\r' )
				{
					currentForce[currentForceLen] = '\0'; // Cap string with a NULL
					break;
				}
			}

			// Scan Imada force string and remove units (ignore all ascii below '9')
			char *strPtr = currentForce;
			while ( *++strPtr <= '9' );
			*strPtr = '\0';

			// - Output data -
			// If the distance has changed, output the data
			if ( currentDistance != lastDistance || forceDistanceReadCount > 0 )
			{
				// Check to see if start/end marker has been reached
				if ( startEndCount == 0 && currentDistance <= distanceStart )
				{
					print("::Start::" NL);
					startEndCount++;
				}
				else if ( startEndCount == 1 && currentDistance >= distanceStart )
				{
					print("::End::" NL);
					startEndCount = 0;
				}

				// Display the current force/distance pair
				print("::");
				dPrint( currentForce );
				print(" gf:");
				printInt32( ( currentDistance * 9921 ) / 1000 ); // Convert to um, see cliFunc_distRead
				print(" um:");
				printInt8( continuityState );
				print("::" NL);
			}

			// Prepare for next iteration
			lastDistance = currentDistance;
			currentForceLen = 0;
		}
		else
		{
			// So consecutive reads will work
			lastDistance = 0;
		}
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


// Test continuity
inline void continuityTest()
{
	// Make sure strobe is set high
	GPIOD_PSOR |= (1<<5);

	// Sample the sense
	continuityState = GPIOD_PDIR & (1<<6) ? 1 : 0;
}


uint32_t readDistanceGauge()
{
	// Setup distance read parameters for iGaging Distance Scale
	//       freq = 9kHz
	// duty_cycle = 20%
	// high_delay = (1/freq) *       (duty_cycle/100)
	//  low_delay = (1/freq) * ((100-duty_cycle)/100)
	uint8_t  bits       = 21; // 21 clock pulses, for 21 bits
	uint32_t high_delay = 22; // Clock high time per pulse
	uint32_t  low_delay = 89; // Clock low  time per pulse

	// Data
	uint32_t distInput = 0;

	// Make sure clock is low initially
	GPIOC_PCOR |= (1<<2); // Set Clock low

	// Scan each of the bits
	for ( uint8_t bit = 0; bit < bits; bit++ )
	{
		// Begin clock pulse
		GPIOC_PSOR |= (1<<2); // Set Clock high

		// Delay for duty cycle
		delayMicroseconds( high_delay );

		// End clock pulse
		GPIOC_PCOR |= (1<<2); // Set Clock low

		// Read Data Bit
		distInput |= GPIOC_PDIR & (1<<1) ? (1 << bit) : 0;

		// Delay for duty cycle
		delayMicroseconds( low_delay );
	}

	return distInput;
}


// ----- Interrupt Functions -----



// ----- CLI Command Functions -----

void cliFunc_contRead( char* args )
{
	// Query Continuity
	continuityTest();

	// Display Continuity
	print( NL );
	info_msg("Continuity: ");
	print( continuityState ? "High - 1" : "Low - 0" );
}


void cliFunc_distRead( char* args )
{
	// Parse number from argument
	//  NOTE: Only first argument is used
	char* arg1Ptr;
	char* arg2Ptr;
	argumentIsolation_cli( args, &arg1Ptr, &arg2Ptr );

	// Convert the argument into an int
	int read_count = decToInt( arg1Ptr ) + 1;

	// If no argument specified, default to 1 read
	if ( *arg1Ptr == '\0' )
	{
		read_count = 2;
	}

	// Repeat reading as many times as specified in the argument
	print( NL );
	while ( --read_count > 0 )
	{
		// Prepare to print output
		info_msg("Distance: ");

		// Data
		uint32_t distInput = readDistanceGauge() - distanceOffset;

		// Output result
		printInt32( distInput );

		// Convert to mm
		// As per http://www.shumatech.com/web/21bit_protocol?page=0,1
		// 21 bits is 2560 CPI (counts per inch) (C/inch)
		// 1 inch is 25.4 mm
		// 2560 / 25.4 = 100.7874016... CPMM (C/mm)
		// Or
		// 1 count is 1/2560 = 0.000390625... inches
		// 1 count is (1/2560) * 25.4 = 0.00992187500000000 mm = 9.92187500000000 um = 9921.87500000000 nm
		// Since there are 21 bits (2 097 152 positions) converting to um is possible by multiplying by 1000
		//    which is 2 097 152 000, and within 32 bits (4 294 967 295).
		// However, um is still not convenient, so 64 bits (18 446 744 073 709 551 615) is a more accurate alternative.
		// For each nm there are 2 097 152 000 000 positions.
		// And for shits:
		//    mm is 2 097 152                 :          0.009 921 875 000 mm : 32 bit
		//    um is 2 097 152 000             :          9.921 875 000     um : 32 bit (ideal acc. for 32 bit)
		//    nm is 2 097 152 000 000         :      9 921.875 000         nm : 64 bit
		//    pm is 2 097 152 000 000 000     :  9 921 875.000             pm : 64 bit (ideal acc. for 64 bit)

		// XXX Apparently shumatech was sorta wrong about the 21 bits of usage
		// Yes there are 21 bits, but the values only go from ~338 to ~30681 which is less than 16 bits...
		// This means that the conversion at NM can use 32 bits :D
		// It's been noted that the multiplier should be 100.6 (and that it could vary from scale to scale)
		uint32_t distNM = distInput * 9921;;
		uint32_t distUM = distNM / 1000;
		uint32_t distMM = distUM / 1000;

		print("  ");
		printInt32( distMM );
		print(" mm  ");
		printInt32( distUM );
		print(" um  ");
		printInt32( distNM );
		print(" nm  ");

		print( NL );

		// Only delay if still counting
		if ( read_count > 1 )
			delay( 50 );
	}
}


void cliFunc_free( char* args )
{
	// Set the forceDistanceRead to 1, which will read until start has passed twice
	forceDistanceRead = 1;
}


void imadaVerboseRead( char* cmd )
{
	// Write command to data register
	//  NOTE: Imada only uses single byte commands, this makes it easy
	//        Always followed by a '\r' CR
	char outCmd[] = { *cmd, '\r', 0x00 };
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
			break;

		// Print out the character
		dPrint( inputChar );
	}
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

	imadaVerboseRead( arg1Ptr );
}


void cliFunc_gaugeHelp( char* args )
{
	print( NL
"\033[1;32mForce Curve Gauge Help\033[0m" NL
" \033[1;33mUsage Overview\033[0m" NL
"  TODO" NL
" \033[1;33mAdditional Command Details\033[0m" NL
"  \033[1;35mdistRead\033[0m" NL
"     Reads the current value from the distance gauge." NL
"     If specified it will N repeated reads with a delay after each read. Useful for testing the distance gauge." NL
"       e.g. \033[35mdistRead 250\033[0m" NL
"  \033[1;35mfree\033[0m" NL
"     Start free scanning force/distance reads." NL
"     Will continue until the [start] distance point has been past twice." NL
"  \033[1;35mimadaComm\033[0m" NL
"     Sends a command to the Imada force gauge." NL
"       e.g. \033[35mimadaComm D\033[0m" NL
"     The commands supported by the gauge depends on the model. Listed below is for the DS2." NL
"       K  Select g  units (default)" NL
"       N  Select N  units" NL
"       O  Select oz units" NL
"       P  Select peak mode" NL
"       T  Select real time mode (default)" NL
"       Z  Zero out display/reading" NL
"       Q  Turn off power" NL
"       E  Read high/low set points" NL
"       D  Read data from force gauge" NL
"       E\033[35mHHHHLLLL\033[0m" NL
"          Set the high/low setpoints, ignore decimals" NL
"          \033[35mHHHH\033[0m is 4 digit high, \033[35mLLLL\033[0m is 4 digit low" NL
"     Responses from the above commands." NL
"       R  Command successful" NL
"       E  Error/Invalid Command" NL
"       E\033[35mHHHHLLLL\033[0m" NL
"          Current high/low setpoints" NL
"          \033[35mHHHH\033[0m is 4 digit high, \033[35mLLLL\033[0m is 4 digit low" NL
"       \033[35m[value][units][mode]\033[0m" NL
"          Data read response" NL
"          \033[35m[value]\033[0m is force currently showing on the display (peak or realtime)" NL
"          \033[35m[units]\033[0m is the configured force units" NL
"          \033[35m[mode]\033[0m  is the current mode (peak or realtime)" NL
"  \033[1;35mread\033[0m" NL
"     Read the current force/distance value." NL
"     If specified it will N repeated reads with a delay after each read." NL
"       e.g. \033[35mread 125\033[0m" NL
"  \033[1;35mstart\033[0m" NL
"     Distance marker \033[35m[start]\033[0m for the start/end of a force curve measurement." NL
"     While in free running mode, a special message is displayed when reaching the \033[35m[start]\033[0m point." NL
"       \033[35m[start]\033[0m is defined by positioning the distance sensor at the position to start and running this command." NL
"     The argument is an offset integer." NL
		);
}


void cliFunc_read( char* args )
{
	// Parse number from argument
	//  NOTE: Only first argument is used
	char* arg1Ptr;
	char* arg2Ptr;
	argumentIsolation_cli( args, &arg1Ptr, &arg2Ptr );

	// Convert the argument into an int
	int read_count = decToInt( arg1Ptr ) + 1;

	// If no argument specified, default to 1 read
	if ( *arg1Ptr == '\0' )
	{
		read_count = 2;
	}

	// Set the overall read count to read_count
	forceDistanceReadCount = read_count;
}


void cliFunc_start( char* args )
{
	// Parse number from argument
	//  NOTE: Only first argument is used
	char* arg1Ptr;
	char* arg2Ptr;
	argumentIsolation_cli( args, &arg1Ptr, &arg2Ptr );

	// Convert the argument into an int
	int offset = decToInt( arg1Ptr ) + 1;

	// Read the current distance and set the new start/end position
	distanceStart = readDistanceGauge() + offset;

	print( NL );
	info_msg("New start/end position: ");
	printInt32( distanceStart - distanceOffset );
}


void cliFunc_stop( char* args )
{
	// Reset the forceDistanceRead and forceDistanceReadCount
	forceDistanceRead = 0;
	forceDistanceReadCount = 0;
}


void cliFunc_zeroForce( char* args )
{
	// Just use the imadaComm command sending the needed argument
	char* commandArg = "Z";
	imadaVerboseRead( commandArg );
}


void cliFunc_zeroPosition( char* args )
{
	// Read the current distance and set the new offset
	distanceOffset = readDistanceGauge();

	print( NL );
	info_msg("New distance offset: ");
	printInt32( distanceOffset );
}

