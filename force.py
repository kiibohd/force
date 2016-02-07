#!/usr/bin/env python3
# Kiibohd Force Gauge Python Interface
# Requires Python3 version of pyusb

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

import binascii
import time
import usb.core # pyusb
import sys

from array       import array
from collections import namedtuple
from struct      import *



### Classes ###

class KiibohdRawIO:
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
			print (">>> DEVICE");
			print ( self.device )

		# No device found
		if self.device is None:
			raise ValueError('Device not found')

		# Find the configuration
		self.cfg = self.device.get_active_configuration()

		if self.debug_mode:
			print (">>> CONFIG");
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
			print (">>> INTERFACE");
			print ( self.interface )

		# Get read and write endpoints
		for endpoint in self.interface.endpoints():
			if endpoint.bEndpointAddress & usb.util.ENDPOINT_IN:
				self.read_ep = endpoint
			else:
				self.write_ep = endpoint

		if self.debug_mode:
			print (">>> ENDPOINTS");
			print ( self.read_ep )
			print ( self.write_ep )

		# Make sure that the read_ep has been flushed
		try:
			self.read_ep.read( 64, timeout=1 )
		except:
			pass

	# Matcher for the USB device finding function
	def _device_matcher( self, device ):
		import usb.util
		# Make sure that a Vendor Specific Interface is found in the configuration
		# bInterfaceClass    0xff
		# bInterfaceSubClass 0xff
		# bInterfaceProtocol 0xff
		for cfg in device:
			if ( usb.util.find_descriptor( cfg, bInterfaceClass=0xFF ) is not None
				and usb.util.find_descriptor( cfg, bInterfaceSubClass=0xFF ) is not None
				and usb.util.find_descriptor( cfg, bInterfaceProtocol=0xFF ) is not None ):
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

def reinit_read():
	ForceCurveDataPoint = namedtuple( 'ForceCurveDataPoint', 'time distance_raw distance speed force_adc force_adc_max continuity direction force_serial' )

	try:
		rawhid = KiibohdRawIO( debug_mode=False, timeout=1000 )
		#rawhid = KiibohdRawIO( debug_mode=True, timeout=1000 )
	except ( usb.core.USBError, ValueError ):
		# Just sleep, then we'll try again in a bit
		time.sleep(1)
		print(".", end='')
		sys.stdout.flush()
		return

	try:
		while True:
			data = rawhid.usb_read()
			data_unp = unpack( '<LLLHHHBB10s', data[:30] )
			data_unp_map = ForceCurveDataPoint._make( data_unp )
			print( data_unp_map )
	except usb.core.USBError:
		# Just sleep, then we'll try again in a bit
		time.sleep(1)
		print("|", end='')
		sys.stdout.flush()
		return



### Main ###

if __name__ == '__main__':
	# Constantly try to re-init
	while True:
		reinit_read()

