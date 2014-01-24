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
	{ "single",        "Query a single force/distance measurement.", cliFunc_single },
	{ "start",         "Mark the current distance as the start/end position.", cliFunc_start },
	{ "zeroForce",     "Zero out the force gauge.", cliFunc_zeroForce },
	{ "zeroPosition",  "Mark the minimum distance for this measurement (bottom).", cliFunc_zeroPosition },
	{ 0, 0, 0 } // Null entry for dictionary end
};


// ----- Functions -----

// Initial Pin Setup, make sure they are sane
inline void pinSetup(void)
{
}


int main(void)
{
	// Configuring Pins
	pinSetup();
	init_errorLED();

	// Setup Output Module
	output_setup();

	// Enable CLI
	init_cli();

	// Register Force Gauge dictionary
	registerDictionary_cli( forceGaugeCLIDict, forceGaugeCLIDictName );

	// Main loop
	while ( 1 )
	{
		process_cli();
	}
}


// ----- Interrupts -----



// ----- CLI Command Functions -----

void cliFunc_free( char* args )
{
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

