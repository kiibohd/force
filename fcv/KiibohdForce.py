#!/usr/bin/python2
#| KiibohdForce
#| Reads in the force curve data from the KiibohdForce microcontroller

# Copyright (C) 2014 by Jacob Alexander
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.




#| Initial Imports
#|  Keep this list at a bare minimum for faster script execution
#|  Delay the imports to their respective function/class if possible
import argparse
import os
import re
import sys



#| Print Decorator Variables
ERROR = '\033[5;1;31mERROR\033[0m:'



#| Python Text Formatting Fixer...
#|  Because the creators of Python are averse to proper capitalization.
textFormatter_lookup = {
	"usage: "            : "Usage: ",
	"optional arguments" : "Optional Arguments",
}

def textFormatter_gettext( s ):
	return textFormatter_lookup.get( s, s )

argparse._ = textFormatter_gettext



#| Variable Container Class
class VariableContainer:
	"""Class for storing variables with a dictionary-like interface"""
	def __init__( self ):
		# Initialize default variables
		self.setDefaults()


	# Item Operators
	def __getitem__( self, index ):
		retVal = self.getitem( index )

		# Could not find the variable
		if retVal == None:
			print "{0} Could not find specified variables: \033[35m{1}\033[0m".format( ERROR, index )

		return retVal


	def __setitem__( self, index, value ):
		# Search the settings list of tuples to see if the variable already exists
		for varset in self.settings:
			if varset[0] == index:
				varset[1] = value
				return

		# Otherwise append variable set to the end of the list
		self.settings.append( [ index, value ] )


	def getitem( self, index ):
		# Search the settings list of tuples for the requested item
		for varset in self.settings:
			if varset[0] == index:
				return varset[1]
		return None


	# Set default variables
	def setDefaults( self ):
		self.settings = [
			#['',''], # List format
		]


	# List of keys
	def keys( self ):
		return tuple( x[0] for x in self.settings )


	# String representation of variable list
	def __str__( self ):
		stringVer = ""
		for varset in self.settings:
			stringVer += '{0} | "{1}"\n'.format( varset[0], varset[1] )

		return stringVer



#| Argument Processing
def processCommandLineArgs( varContainer ):
	# Setup argument processor
	pArgs = argparse.ArgumentParser(
	        usage="%(prog)s [options] <serial device> <switch name>",
	        description="This script records the data from a KiibohdForce gauge and outputs it as an .raw file.\n"
		"The gauge should be powered on at distance 0 and before starting the script, position the gauge at the start position.",
	        epilog="Example: {0} /dev/ttyACM0 switch.raw".format( os.path.basename( sys.argv[0] ) ),
	        formatter_class=argparse.RawTextHelpFormatter,
	        add_help=False,
)

	# Positional Arguments
	pArgs.add_argument( 'serial_port', help=argparse.SUPPRESS ) # Suppressed help output, because Python output is verbosely ugly
	pArgs.add_argument( 'switch_name',  help=argparse.SUPPRESS ) # Suppressed help output, because Python output is verbosely ugly

	# Optional Arguments
	pArgs.add_argument( '-h', '--help', action="help",
		help="This message." )

	# Process Arguments
	args = pArgs.parse_args()

	# Parameters
	varContainer['SwitchName'] = args.switch_name
	varContainer['SerialPort'] = args.serial_port
	varContainer['Filename']   = "{0}.raw".format( varContainer['SwitchName'] )

	# Check file existance, and rename if necessary
	counter = 1
	while os.path.isfile( varContainer['Filename'] ):
		varContainer['Filename'] = "{0}-{1}.raw".format( args.switch_name, counter )
		counter += 1



def sendCmd( serialPort, cmdName ):
	serialPort.write( cmdName )
	serialPort.write( chr( 0x0D ) )
	serialPort.flush()
	inStr = serialPort.read( 1 )
	while serialPort.inWaiting() > 0:
		inStr += serialPort.read( 1 )

	return inStr



def recordData( varContainer, forceData ):
	# Delayed imports
	import serial

	# Initialize serial port, USB serial port is used, speed does not have to be negotiated
	ser = serial.Serial( varContainer['SerialPort'], timeout=2 )

	# Mark the start/end position of the reading
	cmdOut = sendCmd( ser, "start" )

	# Start the free recording
	print "Initiating force curve reading..."
	cmdOut = sendCmd( ser, "free" )

	# Read from the serial port until receiving the ::End::
	inputList = []
	while True:
		inStr = ser.read( 1 )
		while not inStr[-2:] == "\r\n":
			inStr += ser.read( 1 )
		inStr = inStr[:-2]
		print inStr
		inputList.append( inStr )

		if "::End::" in inStr:
			break

	# Once the ::End:: marker is received, stop the free recording
	sendCmd( ser, "stop" )
	print "Force curve reading has finished."

	varContainer['forceData'] = inputList



#| Standalone Execution (Main)
if __name__ == '__main__':

	## Process Args ##
	varContainer = VariableContainer()
	processCommandLineArgs( varContainer )

	## Record Data ##
	forceData = VariableContainer()
	recordData( varContainer, forceData )

	## Write raw data to file ##
	outFile = open( varContainer['Filename'], 'w' )
	for line in varContainer['forceData']:
		outFile.write( "{0}\n".format( line ) )
	outFile.close()

	# Successful Execution
	sys.exit( 0 )

