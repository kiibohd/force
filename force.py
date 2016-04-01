#!/usr/bin/env python3
'''
Kiibohd Force Gauge Python Interface
Requires Python3 version of pyusb
'''

# Copyright (C) 2016 by Jacob Alexander
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


# TODO
# Retrieve data
# Set mode, send commands
# Mode:
#  * Free
#  * Query
# Commands:
#  * Zero
#  * Restart
#  * Up/Down
#  * Auto (/w repeat)
#  * Speed
#
# Features:
#  * Convert raw numbers to units
#  * Convert timestamp to uniform time

### Imports ###

from array       import array
from collections import namedtuple
from datetime    import date
from struct      import unpack

import argparse
import gzip
import json
import os
import sys
import time

# pyusb
import usb.core
import usb.util



# Print Decorator Variables
ERROR = '\033[5;1;31mERROR\033[0m:'



# Python Text Formatting Fixer...
textFormatter_lookup = {
	"usage: "            : "Usage: ",
	"optional arguments" : "Optional Arguments",
}

def textFormatter_gettext( s ):
	'''
	Cleans up argparse help information
	'''
	return textFormatter_lookup.get( s, s )

argparse._ = textFormatter_gettext



### Classes ###

class KiibohdRawIO:
	"""
		KiibohdRawIO
	"""
	def __init__( self, device_index=0, debug_mode=False, timeout=None ):
		# Enumeration index of the USB device
		# This matters if there is more than one GPIB to USB adapter plugged in
		self.device_index = device_index
		self.debug_mode = debug_mode
		self.timeout = timeout

		# Initialize usb read buffer
		self.usb_read_buf = array('B', [])

		# Search for devices
		self.find_usb_devices()

		# Choose which USB device to use (if there are multiple)
		# TODO Select by device id
		self.device = None
		for index, dev in enumerate( self.devices ):
			# Only configure the indexed device
			if index == device_index:
				self.device = dev
				break

		if self.debug_mode:
			print (">>> DEVICE")
			print ( self.device )

		# No device found
		if self.device is None:
			raise ValueError('Device not found')

		# Find the configuration
		self.cfg = self.device.get_active_configuration()

		if self.debug_mode:
			print (">>> CONFIG")
			print ( self.cfg )

		# Find the vendor-specific interface
		for inter in self.cfg.interfaces():
			if (
				inter.bInterfaceClass == 0xFF and
				inter.bInterfaceSubClass == 0xFF and
				inter.bInterfaceProtocol == 0xFF
			):
				self.interface = inter

		# No interface found
		if self.interface is None:
			raise ValueError('RawHID interface not found')

		if self.debug_mode:
			print (">>> INTERFACE")
			print ( self.interface )

		# Make sure the driver hasn't been claimed already
		if self.device.is_kernel_driver_active( self.interface.bInterfaceNumber ):
			print ("Warning: Kernel driver captured this interface...")
			try:
				self.device.detach_kernel_driver( self.interface.bInterfaceNumber )
			except usb.core.USBError as err:
				sys.exit(
					"Could not detatch kernel driver from interface({0}): {1}".format(
						self.interface.bInterfaceNumber, str( err )
					)
				)

		# Get read and write endpoints
		for endpoint in self.interface.endpoints():
			if endpoint.bEndpointAddress & usb.util.ENDPOINT_IN:
				self.read_ep = endpoint
			else:
				self.write_ep = endpoint

		if self.debug_mode:
			print (">>> ENDPOINTS")
			print ( self.read_ep )
			print ( self.write_ep )

		# Make sure that the read_ep has been flushed
		try:
			self.read_ep.read( 64, timeout=1 )
		except:
			pass

	# Matcher for the USB device finding function
	def _device_matcher( self, device ):
		import usb.util as util
		# Make sure that a Vendor Specific Interface is found in the configuration
		# bInterfaceClass    0xff
		# bInterfaceSubClass 0xff
		# bInterfaceProtocol 0xff
		for cfg in device:
			if ( util.find_descriptor( cfg, bInterfaceClass=0xFF ) is not None
				and util.find_descriptor( cfg, bInterfaceSubClass=0xFF ) is not None
				and util.find_descriptor( cfg, bInterfaceProtocol=0xFF ) is not None
			):
				return True

	# Locate all USB devices with the Kiibohd Force Id
	def find_usb_devices( self ):
		# Search for all USB Devices
		# 0x04d8 is the Microchip USB manufacturer ID
		# 0x000c is the specific product id assigned
		self.devices = usb.core.find(
			idVendor=0x1c11,
			idProduct=0xf05c,
			custom_match=self._device_matcher,
			find_all=True,
		)

		# No devices found
		if self.devices is None:
			raise ValueError('Cannot find any devices')

		return self.devices

	# Cleanup
	# Makes sure the interface has been released
	# Prevents ResourceBusy errors/hangs
	def clean( self ):
		usb.util.release_interface( self.device, self.interface )
		usb.util.dispose_resources( self.device )

	# Write data to the USB endpoint
	# data - List of bytes to write
	def usb_write( self, data ):
		assert self.write_ep.write( data, self.timeout ) == len( data )

	# Read byte(s) from USB endpoint
	# datalen - Number of bytes to read
	# Returns a byte array
	def usb_read( self, datalen=64 ):
		# Read USB in 64 byte chunks, store bytes until empty, then read again
		while len( self.usb_read_buf ) < 1 or len( self.usb_read_buf ) < datalen:
			self.usb_read_buf += self.read_ep.read( 64, self.timeout )

		if self.debug_mode:
			print ( self.usb_read_buf, len( self.usb_read_buf ), datalen )

		# Retrieve the requested number of bytes, then remove the items
		data = self.usb_read_buf[0:datalen]
		del self.usb_read_buf[0:datalen]
		return data



### Functions ###

ForceCurveDataPoint = namedtuple(
	'ForceCurveDataPoint',
	'time distance_raw distance speed force_adc force_adc_max continuity direction force_serial'
)

ForceCurveCalibrationPoint = namedtuple(
	'ForceCurveCalibrationPoint',
	'distance force_adc force_serial'
)

forcecurve_filename = "unk.raw"
forcecurve_base_filename = "unk"

def decode_data( data ):
	'''
	Decodes incoming struct data based upon the type (first 8bit character)
	'''
	# Determine type of data
	if data[0] == ord('D'):
		data_unp = unpack( '<LLLHHHBB10s', data[1:31] )
		return ForceCurveDataPoint._make( data_unp )
	elif data[0] == ord('C'):
		data_unp = unpack( '<LH10s', data[1:17] )
		return ForceCurveCalibrationPoint._make( data_unp )
	elif data[0] == ord('M'):
		# Find null terminated portion (or entire)
		return bytearray( data[1:] ).split( b'\x00' )[0].decode("utf-8")

def reinit_read():
	'''
	Reads data from force gauge microcontroller
	Retries automatically if contact is broken/lost

	Check for specific tags to determine state of data recording

	NOTE
	It is important that USB and the files are cleaned up properly before retrying
	There are still some residual USB resource locking issues that may cause the terminal to hang
	'''
	try:
		rawhid = KiibohdRawIO( debug_mode=False, timeout=1000 )

	except ( usb.core.USBError, ValueError ):
		# Just sleep, then we'll try again in a bit
		time.sleep(1)
		print(".", end='')
		sys.stdout.flush()
		return

	write_file = False

	try:
		while True:
			# Decode data
			data_unp_map = decode_data( rawhid.usb_read() )

			# Check if useful
			if data_unp_map == "Starting Test/Calibration":
				# Write file to disk
				# TODO Add option to enable/disable compression
				#outfile = open( forcecurve_filename, 'w' )
				outfile = gzip.open( "{0}.gz".format( forcecurve_filename ), 'wt' )
				write_file = True

			# Write to file if allowed
			if write_file:
				outfile.write( "{0}\n".format( data_unp_map ) )
			print( data_unp_map )

			# Check if this was the last line
			if data_unp_map == "Test Complete" and write_file:
				outfile.close()
				write_file = False

				# Generate skeleton json file
				# TODO parameterize more
				info = {
					"test_name" : forcecurve_base_filename.replace('_', ' '),
					"base_filename" : forcecurve_base_filename,
					"info_box_desc"  : "",
					"line_width" : 3,
					"description" : [
						"",
					],
					"created" : date.today().isoformat(),
					"updated" : date.today().isoformat(),
					"author" : "Jacob Alexander",
					"nick" : "HaaTa",
					"url" : "http://kiibohd.com",

				}
				outfile = open( "{0}.json".format( forcecurve_base_filename ), 'w' )
				json.dump( info, outfile, indent='\t', separators=( ',', ' : ' ) )
				outfile.close()

				break

	except usb.core.USBError:
		# Cleanup first
		rawhid.clean()

		# Just sleep, then we'll try again in a bit
		time.sleep(1)
		print("|", end='')
		sys.stdout.flush()
		return

	# Cleanup
	rawhid.clean()
	sys.exit( 0 )



### Argument Processing ###

def processCommandLineArgs():
	'''
	Process the command line arguments
	'''

	# Setup argument processor
	pArgs = argparse.ArgumentParser(
		usage="%(prog)s [options] <test name>",
		description="This script records the data from a force gauge test and outputs it as a .raw file.\n"
		"Only data from the test will be recorded to the file, free running data will not be recorded.\n"
		"Once the test completes this script will save the file and exit.",
		epilog="Example: {0} switch_test1".format( os.path.basename( sys.argv[0] ) ),
		formatter_class=argparse.RawTextHelpFormatter,
		add_help=False,
	)

	# Positional Arguments
	pArgs.add_argument( 'test_name', help=argparse.SUPPRESS ) # Suppressed help output

	# Optional Arguments
	pArgs.add_argument( '-h', '--help', action="help",
		help="This message."
	)

	# Process Arguments
	args = pArgs.parse_args()

	# Parameters
	base_filename = "{0}".format( args.test_name )
	test_filename = "{0}.raw".format( base_filename )

	# Check file existance, and rename if necessary
	counter = 1
	while os.path.isfile( test_filename ):
		base_filename = "{0}-{1}".format( args.test_name, counter )
		test_filename = "{0}.raw".format( base_filename )
		counter += 1

	global forcecurve_filename
	forcecurve_filename = test_filename
	global forcecurve_base_filename
	forcecurve_base_filename = base_filename



### Main ###

if __name__ == '__main__':
	# Process args
	processCommandLineArgs()

	# Constantly try to re-init
	while True:
		reinit_read()

	# Successful Execution
	sys.exit( 0 )

