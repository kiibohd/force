#!/usr/bin/env python3
'''
Small script to fix raw data issues
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

from collections import namedtuple

import argparse
import os
import sys


### Decorators ###

## Print Decorator Variables
ERROR = '\033[5;1;31mERROR\033[0m:'
WARNING = '\033[5;1;33mWARNING\033[0m:'


## Python Text Formatting Fixer...
##  Because the creators of Python are averse to proper capitalization.
textFormatter_lookup = {
	"usage: "            : "\033[1mUsage\033[0m: ",
	"optional arguments" : "\033[1mOptional Arguments\033[0m",
}

def textFormatter_gettext( s ):
	return textFormatter_lookup.get( s, s )

argparse._ = textFormatter_gettext



### Argument Parsing ###

def checkFileExists( filename ):
	'''
	Validate that file exists

	@param filename: Path to file
	'''
	if not os.path.isfile( filename ):
		print ( "{0} {1} does not exist...".format( ERROR, filename ) )
		sys.exit( 1 )

def command_line_args():
	'''
	Initialize argparse and process all command line arguments
	'''
	# Setup argument processor
	parser = argparse.ArgumentParser(
		usage="%(prog)s [options..] <file>",
		description="Raw data fixing script, takes raw.gz files.",
		epilog="Example: {0} olivetti.raw.gz".format( os.path.basename( sys.argv[0] ) ),
		formatter_class=argparse.RawTextHelpFormatter,
		add_help=False,
	)

	parser.add_argument( 'input_file', help=argparse.SUPPRESS ) # Suppressed help output

	# Optional Arguments
	parser.add_argument(
		'-h', '--help',
		action="help",
		help="This message."
	)

	# Process Arguments
	args = parser.parse_args()

	return args.input_file



### Processing ###

ForceDataPoint = namedtuple(
	'ForceDataPoint',
	'time distance force_adc force_serial continuity direction'
)

def data_point( line ):
	'''
	Data point processing
	'''
	from force import ForceCurveDataPoint # noqa

	# TODO Do something better than this -HaaTa
	point = eval( line )

	# Don't bother if the distance is 0 (Invalid reading)
	if point.distance == 0:
		return

	# TODO Make options for different point "accumulation" methods
	#      - All
	#      - Average per distance tick
	#      - Increase distance precision using speed, time stamps and force ticks

	data_point = ForceDataPoint(
		point.time,
		point.distance,
		point.force_adc,
		point.force_serial,
		point.continuity,
		point.direction,
	)

	return data_point

def next_test_starting_fix( input_file ):
	'''
	Iterates over the file looking for missing "Next Test Starting" tags

	When the direction changes from 1 (up) to 2 (down), the next tag should be "Next Test Starting"
	Changes are saved to a new file, append with .new

	@param input_file: File to process
	'''
	import gzip
	rawfile = gzip.open( input_file, 'rt' )
	outfile_path = "{0}.new.gz".format( input_file )
	outfile = gzip.open( outfile_path, 'wt' )

	# Event tags
	events = {
		"Starting Test/Calibration" : "start",
		"Next Test Starting"        : "next_test",
		"Test Complete"             : "finish",
	}

	# Data points
	data_points = {
		"ForceCurveCalibrationPoint" : "calibration_point",
		"ForceCurveDataPoint"        : "data_point",
	}

	last_direction = 0
	looking_for_next_test = False
	test_count = 0
	fixes = 0

	# Read file line-by-line
	for line in rawfile:
		if line is None:
			continue

		# Check for event, call the associated function
		event = [ event for event in events.keys() if event in line ]
		# Reset check if we found the tag
		if 'Next Test Starting' in event:
			looking_for_next_test = False
			test_count += 1
			outfile.write( line )
			continue

		# Check to see if we were looking for Next Test Starting
		if looking_for_next_test:
			# Insert Next Test Starting
			outfile.write("Next Test Starting\n")

			# Update stats
			fixes += 1
			looking_for_next_test = False

		outfile.write( line )

		# Ignore other events
		if any( event ):
			continue

		# Check for data point, call the associated function
		point = [ point for point in data_points.keys() if point in line ]

		# Check ForceCurveDataPoint to see the current direction
		if 'ForceCurveDataPoint' in point:
			direction = data_point( line ).direction

			# Check for condition
			if direction == 2 and last_direction == 1:
				looking_for_next_test = True

			# Store for next check
			last_direction = direction
			continue

		# Ignore other points
		if any( point ):
			continue

	rawfile.close()
	outfile.close()

	# Move files
	os.rename( input_file, "{0}.old.gz".format( input_file ) )
	os.rename( outfile_path, input_file )

	# Stats
	print("-- Next Test Fix Stats --")
	print( "Found: {0}".format( test_count ) )
	print( "Fixed: {0}".format( fixes ) )



### Main Entry Point ###

if __name__ == '__main__':
	# Process Command-Line Args
	input_file = command_line_args()

	# Check for missing "Next Test Starting"
	next_test_starting_fix( input_file )

	# Successful Execution
	sys.exit( 0 )

