#!/usr/bin/env python3
'''
fcv format conversion and analysis script
'''

# Copyright (C) 2016-2018 by Jacob Alexander
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
# Raw import
# FCV import
#
# FCV export
# csv export
# xls export
# graph export
# - png
# - svg
# plotly export


### Imports ###

from collections import namedtuple
from datetime    import date

import argparse
import inspect
import os
import sys



# Print Decorator Variables
ERROR = '\033[5;1;31mERROR\033[0m:'
WARNING = '\033[1;33mWARNING\033[0m:'



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



### Convenience Functions ###


def peakdet( v, delta, x=None ):
	"""
	https://gist.github.com/endolith/250860
	Converted from MATLAB script at http://billauer.co.il/peakdet.html

	Returns two arrays

	function [maxtab, mintab]=peakdet(v, delta, x)
	%PEAKDET Detect peaks in a vector
	%        [MAXTAB, MINTAB] = PEAKDET(V, DELTA) finds the local
	%        maxima and minima ("peaks") in the vector V.
	%        MAXTAB and MINTAB consists of two columns. Column 1
	%        contains indices in V, and column 2 the found values.
	%
	%        With [MAXTAB, MINTAB] = PEAKDET(V, DELTA, X) the indices
	%        in MAXTAB and MINTAB are replaced with the corresponding
	%        X-values.
	%
	%        A point is considered a maximum peak if it has the maximal
	%        value, and was preceded (to the left) by a value lower by
	%        DELTA.

	% Eli Billauer, 3.4.05 (Explicitly not copyrighted).
	% This function is released to the public domain; Any use is allowed.

	"""
	from numpy import NaN, Inf, arange, isscalar, asarray, array
	maxtab = []
	mintab = []

	if x is None:
		x = arange(len(v))

	v = asarray(v)

	if len(v) != len(x):
		sys.exit('Input vectors v and x must have same length')

	if not isscalar(delta):
		sys.exit('Input argument delta must be a scalar')

	if delta <= 0:
		sys.exit('Input argument delta must be positive')

	mn, mx = Inf, -Inf
	mnpos, mxpos = NaN, NaN

	lookformax = True

	for i in arange(len(v)):
		this = v[i]
		if this > mx:
			mx = this
			mxpos = x[i]
		if this < mn:
			mn = this
			mnpos = x[i]

		if lookformax:
			if this < mx - delta:
				maxtab.append((mxpos, mx))
				mn = this
				mnpos = x[i]
				lookformax = False
		else:
			if this > mn + delta:
				mintab.append((mnpos, mn))
				mx = this
				mxpos = x[i]
				lookformax = True

	return array(maxtab), array(mintab)



### Classes ###

AnalysisDataPoint = namedtuple(
	'AnalysisDataPoint',
	'force_adc_serial_factor'
)

ForceDataPoint = namedtuple(
	'ForceDataPoint',
	'time distance force_adc force_serial continuity direction'
)

class ForceData:
	'''
	Data container class used to store force data in a standard format.
	Also contains the results of any analysis done to the force data.
	'''
	def __init__( self ):
		self.cur_test = 0
		self.cur_switch = 0
		self.tests = 0
		self.switches = 0

		# Initialize datastructure
		self.data = []

	def get( self, name ):
		'''
		Convenience get access to data
		'''
		return self.data[ self.cur_switch ][ name ][ self.cur_test ]

	def set( self, name, value ):
		'''
		Convenience set access to data
		'''
		self.data[ self.cur_switch ][ name ][ self.cur_test ] = value

	def get_var( self, name ):
		'''
		Convenience get access to data, single variable
		'''
		return self.data[ self.cur_switch ][ name ]

	def set_var( self, name, value ):
		'''
		Convenience set access to data, single variable
		'''
		self.data[ self.cur_switch ][ name ] = value

	def set_options( self, options ):
		'''
		Various options set by the command line
		'''
		self.options = options

	def set_switch( self, num ):
		'''
		Sets the switch datastructure to perform operations on
		'''
		# Add switch storage element if needed
		while len( self.data ) < num + 1:
			self.data.append( {
					'test' : { 0 : [] },
					'analysis' : { 0 : [] },
					'force_adc_serial_factor' : None,
					'usable_distance_range' : None,
					'usable_force_range' : None,
					'extra_info' : dict(),
					'rest_point' : { 0 : [] },
					'actuation_point' : { 0 : [] },
					'release_range' : { 0 : [] },
					'bottom_out_point' : { 0 : [] },
					'reset_point' : { 0 : [] },
					'repeat_range' : { 0 : [] },
					'total_force' : { 0 : [] },
					'actuation_force' : { 0 : [] },
					'total_force_avg' : None,
					'actuation_force_avg' : None,
				} )

		self.cur_switch = num

		# Update total number of switches
		if num + 1 > self.switches:
			self.switches = num + 1

	def set_test( self, num ):
		'''
		Sets the current test to store/access ForceDataPoints
		'''
		self.cur_test = num

		# Prepare list
		if num not in self.data[ self.cur_switch ]['test'].keys():
			self.data[ self.cur_switch ]['test'][ num ] = []
			self.data[ self.cur_switch ]['analysis'][ num ] = []

		# Update total number of tests
		if num + 1 > self.tests:
			self.tests = num + 1

	def add( self, data_point ):
		'''
		Adds a ForceDataPoint to the list
		'''
		self.get('test').append( data_point )


	def calibration_analysis( self ):
		'''
		Analysis collected from calibration data
		'''
		import numpy as np
		import statistics

		midpoint = self.mid_point()

		# Calibration Alignment
		# Unfortunately, press/release aren't entirely distance aligned
		# This algorithm finds the 0 pivot point and the release curve offset
		#
		# And for double annoyance, the ADC force data is different as well
		# So it has to be aligned separately :/
		# Then the serial and adc curves need to be aligned

		# Align Calibration Serial Force Data
		print("Midpoint:", midpoint)
		serial_press = self.force_serial()[ :midpoint ]
		serial_release = self.force_serial()[ midpoint: ]

		serial_alignment = min( max( serial_press ), max( serial_release ) )

		serial_press_point = [ index for index, elem in enumerate( serial_press ) if elem >= serial_alignment ]
		serial_release_point = [ index for index, elem in enumerate( serial_release ) if elem >= serial_alignment ]

		self.cal_serial_press_center = self.distance_raw()[ :midpoint ][ serial_press_point[-1] ]
		self.cal_serial_release_center = self.distance_raw()[ midpoint: ][ serial_release_point[-1] ]

		# Align Calibration ADC Force Data
		adc_press = self.force_adc()[ :midpoint ]
		adc_release = self.force_adc()[ midpoint: ]

		adc_alignment = min( max( adc_press ), max( adc_release ) )

		adc_press_point = [ index for index, elem in enumerate( adc_press ) if elem >= adc_alignment ]
		adc_release_point = [ index for index, elem in enumerate( adc_release ) if elem >= adc_alignment ]

		self.cal_adc_press_center = self.distance_raw()[ :midpoint ][ adc_press_point[-1] ]
		self.cal_adc_release_center = self.distance_raw()[ midpoint: ][ adc_release_point[-1] ]


		# Basic force adc/serial factor calculation
		# Calculate press/release separately
		# Only use up until synchronization point

		# Press calibration
		press_difference = self.cal_adc_press_center - self.cal_serial_press_center
		press_analysis = []
		for index in range( abs( press_difference ), midpoint - abs( press_difference ) ):
			data_adc = self.get('test')[ index ]
			data_serial = self.get('test')[ index + press_difference ]

			# Compute factor
			try:
				force_factor = data_adc.force_adc / data_serial.force_serial
			# Just ignore zero values, they skew results of the average anyways
			except ZeroDivisionError:
				continue

			point = AnalysisDataPoint(
				force_factor
			)

			press_analysis.append( point )

		# XXX Currently unused
		if False:
			press_force_adc_serial_factor = statistics.median_grouped(
				[ elem.force_adc_serial_factor for elem in press_analysis ]
			)

			# Release calibration
			release_difference = self.cal_adc_release_center - self.cal_serial_release_center
			release_analysis = []
			for index in range( midpoint - abs( release_difference ), len( self.get('test') ) - abs( release_difference ) ):
				data_adc = self.get('test')[ index ]
				print( index, release_difference, len( self.get('test') ) )
				print( midpoint )
				data_serial = self.get('test')[ index + release_difference ]

				# Compute factor
				try:
					force_factor = data_adc.force_adc / data_serial.force_serial
				# Just ignore zero values, they skew results of the average anyways
				except ZeroDivisionError:
					continue

				point = AnalysisDataPoint(
					force_factor
				)

				release_analysis.append( point )

			release_force_adc_serial_factor = statistics.median_grouped(
					[ elem.force_adc_serial_factor for elem in release_analysis ]
			)

			print( press_force_adc_serial_factor, release_force_adc_serial_factor )






		for index, datapoint in enumerate( self.get('test') ):
			try:
				force_factor = datapoint.force_adc / datapoint.force_serial
			except ZeroDivisionError:
				force_factor = 0

			point = AnalysisDataPoint(
				force_factor
			)
			self.get('analysis').append( point )

		# Use the grouped median as the conversion factor
		self.set_var('force_adc_serial_factor',
			statistics.median_grouped(
				[ elem.force_adc_serial_factor for elem in self.get('analysis') ]
			)
		)

		# XXX
		# TODO
		# Remove hard-coding of factor
		#print( self.get_var('force_adc_serial_factor') )
		#self.set_var('force_adc_serial_factor', ( press_force_adc_serial_factor, release_force_adc_serial_factor ) )
		press_force_adc_serial_factor = 37.98400556328233
		self.set_var('force_adc_serial_factor', ( press_force_adc_serial_factor, press_force_adc_serial_factor - 1.35 ) )

		# Determine distance range, use raw adc values from press
		adc_diff = list( np.diff( [ elem.force_adc for elem in self.get('test')[ :self.mid_point() ] ] ) )
		peaks = peakdet( adc_diff, 100 )[0] # TODO configurable delta and forced range
		#peaks = peakdet( adc_diff, 200 )[0] # TODO configurable delta and forced range

		# XXX Use the peak_detection_index configuration in the json to tweak peak detection
		# Otherwise just use the first peak
		plot_data = self.get_var('extra_info')
		if 'peak_detection_index' in plot_data.keys():
			peak_index = plot_data['peak_detection_index']
		else:
			peak_index = 0
		print("Peaks:", peaks)

		first = adc_diff.index( peaks[ peak_index ][1] )
		last = adc_diff.index( peaks[-1][1] )

		# Use the max force as 2x the median grouped force over the newly calculated distance range (press)
		force_data = self.force_adc_converted()[ first:last ]
		print("Force Data:", force_data)
		max_force_calc = statistics.median_grouped( force_data ) * 2
		self.set_var('usable_force_range', ( ( 0, max_force_calc ) ) )

		# XXX Override usable force range, needed if statistics from calibration are not usable
		if 'max_cal_force_override' in plot_data.keys():
			self.set_var('usable_force_range', ( ( 0, plot_data['max_cal_force_override'] ) ) )

		# Start from beginning of press + 1/4 mm
		# TODO peak detection algorith has issues with exponential force curves -Jacob
		# TODO use calibration data for start of press instead of first -Jacob
		dist_mm = self.distance_mm()
		start_mm = dist_mm[ first ] + 0.25
		print ( start_mm )
		self.set_var('usable_distance_range', ( start_mm, 0 ) )


	def curve_analysis( self ):
		'''
		Runs analysis on each recorded force curve (including calibration)
		Order matters, as each analysis stage may provide the next with needed data

		Analysis Computed
		1. Distance Points/Ranges
		2. Force Points/Ranges
		'''
		# Iterate over each set of test data, and do analysis
		for index in self.options['curves']:
			# Set datastructures for given test
			self.set_test( index )

			# Run analysis
			self.curve_analysis_distance()
			#self.curve_analysis_force() # TODO
			self.area_under_curve()

		self.analysis_averaging()

	def curve_analysis_distance( self ):
		'''
		-- Distance Analysis --
		Rest       Point - Start of press, when force goes from noise-zero to increase
		Actuation  Point - Distance when switch goes from off to on state
		Release    Range - Distance range between acutation and bottom-out
		Bottom-out Point - End of press, just before force goes to ~infinity
		Reset      Point - Distance when switch goes from on to off state
		Repeat     Range - Distance range between reset and rest

		Each analysis value is accompanied by a dataset index, to easily map force vs. distance
		( index, ( force, distance ) )
		'''
		distance = self.distance_mm()
		force_adc = self.force_adc_converted()

		# Rest Point
		# Find the index of 0 mm, or the first non-negative value
		rest_point = None
		for index, elem in enumerate( distance ):
			if elem >= 0:
				rest_point = ( index, ( force_adc[ index ], elem ) )
				break

		# Actuation and Reset Points
		actuation = self.actuation()
		if len( actuation ) > 0:
			actuation_point = ( actuation[0], ( force_adc[ actuation[0] ], distance[ actuation[0] ] ) )
			reset_point = ( actuation[1], ( force_adc[ actuation[1] ], distance[ actuation[1] ] ) )
		else:
			actuation_point = None
			reset_point = None

		# Bottom-out Point
		# Use the point where force crosses the max_force
		# Default to max force point
		plot_data = self.get_var('extra_info')
		default_index = self.get_var('press_max_force_index')
		bottom_out_point = ( default_index, ( force_adc[ default_index ], distance[ default_index ] ) )
		for index, elem in enumerate( force_adc ):
			if elem >= plot_data['max_force']:
				# Since we have bottomed-out at this point, use the previous index
				index -= 1
				bottom_out_point = ( index, ( elem, distance[ index ] ) )
				break

		# Store analysis with test
		self.set('rest_point', rest_point )
		self.set('actuation_point', actuation_point )
		self.set('release_range', ( actuation_point, bottom_out_point ) )
		self.set('bottom_out_point', bottom_out_point )
		self.set('reset_point', reset_point )
		self.set('repeat_range', ( reset_point, rest_point ) )

	def curve_analysis_force( self ):
		'''
		-- Force Analysis --
		Acuation   Force - Force when switch goes from off to on state
		Reset      Force - Force when switch goes from on to off state
		Bottom-out Force - End of press, force just before peaking towards infinity
		Pre-load   Force - Beginning of press, just after the press has started

		Each analysis value is accompanied by a dataset index, to easily map force vs. distance
		( index, ( force, distance ) )
		'''
		distance = self.distance_mm()
		force_adc = self.force_adc_converted()

		# Pre-load Force
		# TODO - Algorithm
		pre_load_force = 0

		# Bottom-out force
		bottom_out_point = self.get('bottom_out_point')
		bottom_out_force = ( bottom_out_point[0], force_adc[ bottom_out_point[0] ] )

		# Store analysis with test
		self.set('actuation_force', self.get('actuation_point') )
		self.set('reset_force', self.get('reset_point') )
		self.set('bottom_out_force', bottom_out_force )
		self.set('pre_load_force', pre_load_force )

	def area_under_curve( self ):
		'''
		-- Area Under Curve --
		Uses numpy to integrate the area under the force curve

		Two different calculations
		1) Full area, 0 to bottom out
		2) Actuation, 0 to actuation point (if available)
		'''
		from numpy import trapz

		distance = self.distance_mm()
		force_adc = self.force_adc_converted()

		rest_point = self.get('rest_point')
		bottom_out_point = self.get('bottom_out_point')
		actuation_point = self.get('actuation_point')

		print( "Rest, Bottom-out:", rest_point, bottom_out_point )

		# Full Area
		total_force = trapz(
			force_adc[ rest_point[0]:bottom_out_point[0] ],
			distance[ rest_point[0]:bottom_out_point[0] ],
		)
		print("total force:", total_force, "gfmm")

		# Actuation
		actuation_force = None
		if actuation_point is not None:
			actuation_force = trapz(
				force_adc[ rest_point[0]:actuation_point[0] ],
				distance[ rest_point[0]:actuation_point[0] ],
			)
			print("actuation force:", actuation_force, "gfmm")
			if actuation_force <= 1:
				print("{0} Less than 1 gfmm, ignoring...".format( WARNING ) )
				actuation_force = None

		self.set('total_force', total_force )
		self.set('actuation_force', actuation_force )

	def analysis_averaging( self ):
		'''
		Takes analysis from the set of tests and averages it
		'''
		total_tests = len( self.data[ self.cur_switch ]['total_force'] ) - 1

		# Total Force
		total = 0
		for index in self.options['curves']:
			# Set datastructures for given test
			self.set_test( index )

			# Get total_force for this test
			total += self.get('total_force')
		total_force_avg = total / total_tests
		print("total force avg:", total_force_avg, "gfmm")
		self.set_var('total_force_avg', total_force_avg )

		# Actuation Force
		if 'actuation_force' in self.data[ self.cur_switch ].keys():
			total = 0
			for index in self.options['curves']:
				# Set datastructures for given test
				self.set_test( index )

				# Get actuation_force for this test
				value = self.get('actuation_force')

				# Sometimes a single test won't have an actuation...
				# Bad test, or switch, ignore from average
				if value is None:
					total_tests -= 1
					print("{0} Missing actuation from test #{1}...".format(
						WARNING,
						index,
					) )
					continue
				total += value
			# If 0, then just ignore this measurement
			if total != 0:
				actuation_force_avg = total / total_tests
				print("actuation force avg:", actuation_force_avg, "gfmm")
				self.set_var('actuation_force_avg', actuation_force_avg )


	def max_force_points_converted( self ):
		'''
		Returns a tuple of the distance ticks for the max force points
		These values are interpolated, so don't bother trying to look them up in the dataset

		Ideally, this calculation only needs to be done on the calibration data.
		Unfortunately, some datasets...are less clean, so it's recommended to run on each press/release pair
		'''
		# Determine the start and max distance points (using data indices)
		# These points will be the same for all the curves in the dataset
		# This is used in the conversion to mm needed for the usable_distance_range
		# Using the approximate points, interpolate to find the point we want
		# dist = dist_1 + (dist_2 - dist_1)( (force - force_1) / (force_2 - force_1) )
		max_force_calc = self.get_var('usable_force_range')[1]
		print( "MAX FORCE: ", max_force_calc )

		# Press "wave"
		conv_factor = self.get_var('force_adc_serial_factor')[0]
		try:
			press_max_force_index = [
				index
				for index, value in enumerate( self.get('test') )
				if value.force_adc / conv_factor > max_force_calc
			][0]
			self.set_var('press_max_force_index', press_max_force_index )
		except IndexError as err:
			print( "{0} Check data, likely a Next Test Starting tag is missing/in the wrong spot.".format(
				WARNING
			) )
			print( err )
			# Max Detected force
			press_max_force_detected = [
				value.force_adc / conv_factor
				for index, value in enumerate( self.get('test') )
				if value.force_adc / conv_factor
			]
			print( "Max Detected Force:", max(press_max_force_detected ) )
			print( "Max Force Calculated:", max_force_calc )

		point1 = self.get('test')[ press_max_force_index - 1 ]
		point2 = self.get('test')[ press_max_force_index ]
		press_max = point1.distance + ( point2.distance - point1.distance ) * (
			(max_force_calc - point1.force_adc / conv_factor) /
			(point2.force_adc / conv_factor - point1.force_adc / conv_factor)
		)

		# Release "wave"
		conv_factor = self.get_var('force_adc_serial_factor')[1]
		release_max_force_index = [
			index
			for index, value in reversed( list( enumerate( self.get('test') ) ) )
			if value.force_adc / conv_factor > max_force_calc
		][0]
		point1 = self.get('test')[ release_max_force_index - 1 ]
		point2 = self.get('test')[ release_max_force_index ]
		release_max = point1.distance + ( point2.distance - point1.distance ) * (
			(max_force_calc - point1.force_adc / conv_factor) /
			(point2.force_adc / conv_factor - point1.force_adc / conv_factor)
		)

		return (press_max, release_max)


	def force_adc( self ):
		'''
		Returns a list of force adc values for the current test
		'''
		return [ data.force_adc for data in self.get('test') ]

	def force_serial( self ):
		'''
		Returns a list of force serial values for the current test
		'''
		data = [ data.force_serial for data in self.get('test') ]
		return data

	def force_adc_converted( self ):
		'''
		Returns a list of converted force adc values for the current test
		'''
		midpoint = self.mid_point()
		data = [ data.force_adc / self.get_var('force_adc_serial_factor')[0] for data in self.get('test')[ :midpoint ] ]
		data.extend( [ data.force_adc / self.get_var('force_adc_serial_factor')[1] for data in self.get('test')[ midpoint: ] ] )
		return data


	def distance_raw( self ):
		'''
		Returns a list of distance values for the current test
		'''
		data = [ data.distance for data in self.get('test') ]
		return data

	def distance_mm( self ):
		'''
		Returns a list of distance values for the current test converted to mm

		Convert to mm
		As per http://www.shumatech.com/web/21bit_protocol?page=0,1
		21 bits is 2560 CPI (counts per inch) (C/inch)
		1 inch is 25.4 mm
		2560 / 25.4 = 100.7874016... CPMM (C/mm)
		Or
		1 count is 1/2560 = 0.000390625... inches
		1 count is (1/2560) * 25.4 = 0.00992187500000000 mm = 9.92187500000000 um = 9921.87500000000 nm
		Since there are 21 bits (2 097 152 positions) converting to um is possible by multiplying by 1000
		which is 2 097 152 000, and within 32 bits (4 294 967 295).

		However, um is still not convenient, so 64 bits (18 446 744 073 709 551 615) is a more accurate alternative.
		For each nm there are 2 097 152 000 000 positions.
		And for shits:

		mm is 2 097 152                 :          0.009 921 875 000 mm : 32 bit
		um is 2 097 152 000             :          9.921 875 000     um : 32 bit (ideal acc. for 32 bit)
		nm is 2 097 152 000 000         :      9 921.875 000         nm : 64 bit
		pm is 2 097 152 000 000 000     :  9 921 875.000             pm : 64 bit (ideal acc. for 64 bit)

		XXX Apparently shumatech was sorta wrong about the 21 bits of usage
		Yes there are 21 bits, but the values only go from ~338 to ~30681 which is less than 16 bits...
		This means that the conversion at NM can use 32 bits :D
		It's been noted that the multiplier should be 100.6 (and that it could vary from scale to scale)
		'''
		mm_conv = 0.009921875
		plot_data = self.get_var('extra_info')

		# Determine the start and max distance points (using data indices)
		max_force_points = self.max_force_points_converted()
		zero_point_press = max_force_points[0]
		zero_point_release = max_force_points[1]

		# XXX Use the release_mm_offset to deal with algorithm issues, defined in mm when necessary
		plot_data = self.get_var('extra_info')
		if 'release_mm_offset' in plot_data.keys():
			release_offset = plot_data['release_mm_offset']
		else:
			release_offset = 0

		# Apply mm_shift
		if 'mm_shift' in plot_data.keys():
			press_offset = plot_data['mm_shift']
			release_offset += plot_data['mm_shift']
		else:
			press_offset = 0

		# Compute lists with offsets
		mid_point = self.mid_point()
		press = [ ((data.distance - zero_point_press) * mm_conv) + press_offset for data in self.get('test')[:mid_point] ]
		release = [ ((data.distance - zero_point_release) * mm_conv) + release_offset for data in self.get('test')[mid_point:] ]

		press.extend( release )

		# Flip the distance axis at max_distance (e.g. 4 mm)
		max_mm = plot_data['max_distance']
		press = [ (point - max_mm) * -1 for point in press ]

		# Make sure distance points are sequential
		# 3 point, 2 differences (Naive -HaaTa)
#		for index, elem in enumerate( press ):
#			# We need adjacent points to do check
#			if index == 0 or index == len( press ) - 1:
#				continue
#
#			# Look for non-sequential distance points
#			# These may happen due to a firmware bug, transient reading error, or mm conversion error
#			before = press[index - 1]
#			after = press[index + 1]
#			if ( before > elem and after > elem ) or ( before < elem and after < elem ):
#				avg = ( before + after ) / 2
#				press[index] = avg
#				# Warn the user we are modifying data
#				print ( "{0} Found non-sequential point {1} mm  at index '{2}' between {3} mm and {4} mm. "
#					"Averaging to {5} mm.".format(
#						WARNING,
#						elem, index,
#						before, after,
#						avg,
#					)
#				)

		# Make sure distance points are sequential
		# Using 5 points to determine directional intent (curve changes direction)
		for index, elem in enumerate( press ):
			# We need adjacent points to do check
			if index == 0 or index > len( press ) - 4:
				continue

			# Look for non-sequential distances points
			# The algorithm is to look at the 3 differences between the 4 points
			# If 3/4 differences point in one direction *and* changing the current element brings this to 4/4
			# in a single direction, change, otherwise, do nothing
			results = []
			total_pos = 0
			total_neg = 0
			for pos in range( index - 1, index + 3 ):
				results.append( press[pos] - press[pos + 1] )
				if results[-1] > 0:
					total_pos += 1
				elif results[-1] < 0:
					total_neg -= 1

			# Adjustment condition, must have 3/4 in the same direction to make an adjustment
			if (
				( total_pos == 3 and ( results[0] < 0 or results[1] < 0 ) ) or
				( total_neg == -3 and ( results[0] > 0 or results[1] > 0 ) )
			):
				before = press[index - 1]
				after = press[index + 1]
				avg = ( before + after ) / 2

				# Ignore if after position is the 0 mm position
				if after == 0.0 or elem == 0.0:
					continue

				press[index] = avg

				# Warn the user we are modifying data
				print ( "{0} Found non-sequential point {1} mm  at index '{2}' between {3} mm and {4} mm. "
					"Averaging to {5} mm.".format(
						WARNING,
						elem, index,
						before, after,
						avg,
					)
				)



		return press

	def mid_point( self ):
		'''
		Determines the direction change point of a test sequence
		'''
		print( min( self.get('test'), key = lambda t: t.distance ) )
		return self.get('test').index( min( self.get('test'), key = lambda t: t.distance ) )

	def mid_point_dir( self ):
		'''
		Determines the direction change point of a test sequence using the direction field (more accurate than mid_point)
		Look for the transition from 2 to 1
		2 - Down
		1 - Up
		'''
		prev = ForceDataPoint( None, None, None, None, None, None )
		position = None
		for index, elem in enumerate( self.get('test') ):
			if prev.direction == 2 and elem.direction == 1:
				position = index
				break
			prev = elem

		return position

	def mid_point_peak( self ):
		'''
		Determines the direction change using the bottom out force peaks
		Returns, a tuple
		(end of press, start of release)
		'''
		import numpy as np

		from operator import itemgetter

		adc_diff = list( np.diff( [ elem.force_adc for elem in self.get('test') ] ) )

		# XXX Naive version
		#return ( adc_diff.index( max( adc_diff ) ), adc_diff.index( min( adc_diff ) ) )

		# XXX Peak detection version (still a bit naive) -HaaTa
		#peaks = peakdet( adc_diff, 200 )
		#return ( int( peaks[0][0][0] ), int( peaks[1][-1][0] ) )

		# Peak detection with peak analysis
		# For the max (bottom out start), find the highest peak, then check the previous peak
		# If it's force peak is greater than half of the current, then move the peak, and check again
		# For the min (bottom out end), check forward instead
		peaks = peakdet( adc_diff, 100 )
		max_peaks = peaks[0].tolist()
		min_peaks = peaks[1].tolist()
		max_point = max_peaks.index( max( max_peaks, key=itemgetter( 1 ) ) )
		min_point = min_peaks.index( min( min_peaks, key=itemgetter( 1 ) ) )

		# Bottom out start approximation
		for index in range( max_point - 1, -1, -1 ):
			# Check if force peak is at least half as large as the previous
			if max_peaks[index][1] < max_peaks[index - 1][1] / 2:
				break

			# Update new bottom out point
			max_point = index

		# Bottom out end approximation
		for index in range( min_point + 1, len( min_peaks ) ):
			# Check if force peak is at least half as large as the previous
			if min_peaks[index][1] > min_peaks[index - 1][1] / 2:
				break

			# Update new bottom out point
			min_point = index

		peaks = ( int( max_peaks[max_point][0] ), int( min_peaks[min_point][0] ) )

		return peaks

	def actuation( self ):
		'''
		Determines press/release actuation, returns a list of change (index) points.
		This can generally be assumed as (press, release).

		Default state (usually 1), is set initially.
		The first change is the press.
		The second change is the release.
		'''
		prev = ForceDataPoint( None, None, None, None, None, None )
		state = None
		actuation = []
		for index, elem in enumerate( self.get('test') ):
			# Determine initial state
			if state is None:
				state = elem.continuity

			# Determine if the state has changed
			if prev.continuity == state and elem.continuity != state:
				actuation.append( index )
				state = elem.continuity
			prev = elem

		return actuation



class GenericForceData:
	'''
	Common function/variables used for all import/export classes

	Mark contstructor variables as None if not using
	force_data is required
	'''
	def __init__( self, force_data, input_file, output_file ):
		# Input data
		self.force_data = force_data
		self.input_file = input_file
		self.output_file = output_file

		# Processing variables
		self.cur_test = None
		self.calibration = False

		# Check if a valid input file
		if input_file is not None and not self.valid_input_file():
			self.valid_input_file()

	def process_input( self ):
		'''
		Import force data from file into force_data structure
		'''
		raise NotImplementedError("Invalid/Not implemented yet")

	def process_output( self ):
		'''
		Output force data from force_data structure to the output file
		'''
		raise NotImplementedError("Invalid/Not implemented yet")

	def valid_input_file( self ):
		'''
		Checks if the input file is valid.
		'''
		raise NotImplementedError("Invalid/Not implemented yet")



class RawForceData( GenericForceData ):
	'''
	Class used to import and prepare force gauge data
	'''
	def __init__( self, force_data, input_file ):
		# Store extension (used to determine if compressed)
		split = os.path.splitext( input_file )
		self.extension = split[1]

		super( RawForceData, self ).__init__( force_data, input_file, None )

		in_file = input_file
		if self.extension == '.gz':
			in_file = split[0]

		# Just in case we're missing the start
		self.cur_test = 0

		# Read in companion json file if it exists
		json_info = "{0}.json".format( os.path.splitext( in_file )[0] )
		if not os.path.exists( json_info ):
			print ( "{0} JSON info not available '{1}', conversion may not be complete".format( WARNING, json_info ) )
			return

		with open( json_info ) as fp:
			import json
			self.force_data.set_var( 'extra_info', json.load( fp ) )

	def start( self ):
		'''
		Start of test handler
		'''
		self.cur_test = 0
		self.calibration = False
		self.force_data.set_test( self.cur_test )
		self.data_point_cache = [ None ]

		print ( "Processing Test #{0}".format( self.cur_test ) )
		return True

	def next_test( self ):
		'''
		Next test handler
		'''
		self.cur_test += 1

		# Check if we can stop processing
		if self.cur_test > max( self.force_data.options['curves'] ):
			print ( "Skipping Test #{0}".format( self.cur_test ) )
			return False

		self.calibration = False
		self.force_data.set_test( self.cur_test )

		print ( "Processing Test #{0}".format( self.cur_test ) )
		return True

	def finish( self ):
		'''
		Test complete handler
		'''
		self.calibration = False
		return True

	def valid_input_file( self ):
		'''
		Checks if the raw file is valid in the last 5 lines
		Just need to make sure there is a 'Test Complete' at the end of the file
		No need to process corrupted files (takes a long time to figure this out otherwise...)
		'''
		import subprocess

		# Compressed, yes due to gzip, we have to decompress the whole thing first...
		if self.extension == '.gz':
			ps = subprocess.Popen( [ 'gunzip', '-c', self.input_file ], stdout=subprocess.PIPE )
			lines = subprocess.check_output( [ 'tail', '-5' ], stdin=ps.stdout ).decode('utf-8')
		# Using subprocess, because simple and fast
		else:
			lines = subprocess.check_output( [ 'tail', '-5', self.input_file ] ).decode('utf-8')

		# Check for proper finish of the file
		if 'Test Complete' not in lines:
			raise RuntimeError("Corrupted .raw file, missing 'Test Complete'")

	def calibration_point( self, line ):
		'''
		Calibration data point processing
		'''
		from force import ForceCurveCalibrationPoint # noqa

		self.calibration = True
		# TODO Do something better than this -HaaTa
		point = eval( line )

		# Don't bother if the distance is 0 (Invalid reading)
		if point.distance == 0:
			return

		# Convert the force serial strings into ints
		try:
			force_serial = float( point.force_serial.decode('utf-8')[:6] )
		except ValueError:
			force_serial = 0.0

		data_point = ForceDataPoint(
			None, # time
			point.distance,
			point.force_adc,
			force_serial,
			None, # continuity
			None, # direction
		)
		self.force_data.add( data_point )

	def data_point( self, line ):
		'''
		Data point processing
		'''
		import statistics

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

		# Accumulate and average all the data points per distance tick
		if self.data_point_cache[0] != data_point.distance:
			# Make sure this isn't the first data point
			if self.data_point_cache[0] is not None:
				# Calculate averages
				avg_time = statistics.median_grouped(
					[ point.time for point in self.data_point_cache[1:] ]
				)
				avg_force_adc = statistics.median_grouped(
					[ point.force_adc for point in self.data_point_cache[1:] ]
				)

				# Build new data point using averages and constant data
				avg_point = ForceDataPoint(
					avg_time,
					self.data_point_cache[1].distance,
					avg_force_adc,
					self.data_point_cache[1].force_serial,
					self.data_point_cache[1].continuity,
					self.data_point_cache[1].direction,
				)

				# Add point to data store
				self.force_data.add( avg_point )

			# Setup next cache
			self.data_point_cache = [ data_point.distance ]

		# Add to cache
		self.data_point_cache.append( data_point )

	def process_input( self ):
		'''
		Import force data from file into force_data structure
		'''
		# Check to see if this file was compressed
		rawfile = None
		if self.extension == '.gz':
			import gzip
			rawfile = gzip.open( self.input_file, 'rt' )
		else:
			rawfile = open( self.input_file, 'r' )

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

		# Read file line-by-line
		for line in rawfile:
			if line is None:
				continue

			# Check for event, call the associated function
			event = [ event for event in events.keys() if event in line ]
			if any( event ):
				method = getattr( self, events[ event[0] ] )

				# Check whether we are finishing early
				if method() is False:
					break

				continue

			# Check for data point, call the associated function
			point = [ point for point in data_points.keys() if point in line ]
			if any( point ):
				method = getattr( self, data_points[ point[0] ] )
				method( line )
				continue

		rawfile.close()


class CSVForceData( GenericForceData ):
	'''
	Class used to import and export csv force gauge data

	Mark either input_file or output_file None if not using
	'''
	def __init__( self, force_data, input_file, output_file ):
		super( CSVForceData, self ).__init__( force_data, input_file, output_file )

	def process_output( self ):
		'''
		Output force data from force_data structure to the output file
		'''
		raise NotImplementedError("Invalid/Not implemented yet")


class FCVForceData( GenericForceData ):
	'''
	Class used to import and export fcv force gauge data

	Mark either input_file or output_file None if not using
	'''
	def __init__( self, force_data, input_file, output_file ):
		super( FCVForceData, self ).__init__( force_data, input_file, output_file )

	def process_output( self ):
		'''
		Output force data from force_data structure to the output file
		'''
		raise NotImplementedError("Invalid/Not implemented yet")


class XLSForceData( GenericForceData ):
	'''
	Class used to import and export xls/xlsx force gauge data
	'''
	def __init__( self, force_data, input_file, output_file ):
		super( XLSForceData, self ).__init__( force_data, input_file, output_file )
		raise NotImplementedError("Invalid/Not implemented yet")


class PNGForceData( GenericForceData ):
	'''
	Class used to import and export png force gauge data
	'''
	def __init__( self, force_data, input_file, output_file ):
		super( PNGForceData, self ).__init__( force_data, input_file, output_file )
		raise NotImplementedError("Invalid/Not implemented yet")


class SVGForceData( GenericForceData ):
	'''
	Class used to import and export svg force gauge data
	'''
	def __init__( self, force_data, input_file, output_file ):
		super( SVGForceData, self ).__init__( force_data, input_file, output_file )
		raise NotImplementedError("Invalid/Not implemented yet")


class PlotlyForceData( GenericForceData ):
	'''
	Class used to export force gauge data as a plotly graph
	'''
	def __init__( self, force_data, output_file ):
		super( PlotlyForceData, self ).__init__( force_data, None, output_file )

	def process_calibration( self, graphs ):
		'''
		Generates the calibration graphs
		'''
		from plotly.graph_objs import Scatter as Scatter
		#from plotly.graph_objs import Scattergl as Scatter

		# Retrieve misc info about force curve(s)
		plot_data = self.force_data.get_var('extra_info')

		# Acquire calibration data
		self.force_data.set_test( 0 )
		force_adc = self.force_data.force_adc_converted()
		force_serial = self.force_data.force_serial()
		distance = self.force_data.distance_mm()
		mid_point = self.force_data.mid_point()

		# Only hide calibration if there is actual test data
		visible = 'legendonly'
		if self.force_data.tests == 1:
			visible = True

		# TODO convert to Scatter when less buggy -HaaTa
		if 0 in self.force_data.options['curves']:
			graphs.extend([
				Scatter(
					x=distance[:mid_point],
					y=force_serial[:mid_point],
					name="RS232 (Cal) Press",
					legendgroup='rs232',
					line={
						'width' : plot_data['line_width'],
					},
					visible=visible,
				),
				Scatter(
					x=distance[mid_point:],
					y=force_serial[mid_point:],
					name="RS232 (Cal) Release",
					legendgroup='rs232',
					line={
						'width' : plot_data['line_width'],
					},
					visible=visible,
				),
				Scatter(
					x=distance[:mid_point],
					y=force_adc[:mid_point],
					name="Analog (Cal) Press",
					legendgroup='analog',
					line={
						'width' : plot_data['line_width'],
					},
					visible=visible,
				),
				Scatter(
					x=distance[mid_point:],
					y=force_adc[mid_point:],
					name="Analog (Cal) Release",
					legendgroup='analog',
					line={
						'width' : plot_data['line_width'],
					},
					visible=visible,
				),
			])

	def process_graphs( self, graphs, annotations ):
		'''
		Generates the non-calibration graphs
		'''
		from plotly.graph_objs import Scatter as Scatter
		#from plotly.graph_objs import Scattergl as Scatter

		# Retrieve misc info about force curve(s)
		plot_data = self.force_data.get_var('extra_info')

		# Switch
		switch = self.force_data.cur_switch

		curves = [ curve for curve in self.force_data.options['curves'] if curve != 0 ]
		for index in curves:
			self.force_data.set_test( index )
			mid_point = self.force_data.mid_point_dir()
			force_adc = self.force_data.force_adc_converted()

			try:
				distance = self.force_data.distance_mm()
			except IndexError as err:
				print( "SKIPPING #{0}".format( index ) )
				print( err )
				distance = [ 0 * len( force_adc ) ]

			actuation = self.force_data.actuation()

			# Only show the number on the legend if necessary
			number = " {0}".format( index )
			if self.force_data.tests == 2:
				number = ""

			# If more than one switch, add the name as well
			name = ""
			if self.force_data.switches > 1:
				name = "{0}</br>".format( plot_data['test_name'] )

			# Only hide curve if specified
			visible = True
			if index in self.force_data.options['curves_hidden']:
				visible = 'legendonly'
			# Show if overridden
			if 'curves_show' in plot_data.keys() and index in plot_data['curves_show']:
				visible = True

			# Only show if not disabled
			if self.force_data.options['press_disable'] is False:
				graphs.extend([
					Scatter(
						x=distance[:mid_point],
						y=force_adc[:mid_point],
						name="{0}Press{1}".format( name, number ),
						legendgroup='test{0}-{1}'.format( switch, index ),
						line={
							'width' : plot_data['line_width'],
						},
						visible=visible,
					)
				])

			# Only show if not disabled
			if self.force_data.options['release_disable'] is False:
				graphs.extend([
					Scatter(
						x=distance[mid_point:],
						y=force_adc[mid_point:],
						name="{0}Release{1}".format( name, number ),
						legendgroup='test{0}-{1}'.format( switch, index ),
						line={
							'width' : plot_data['line_width'],
						},
						visible=visible,
					),
				])

			# Press
			if len( actuation ) > 0 and index == 1 and self.force_data.options['press_disable'] is False:
				annotations.extend([{
					'text' : '{0}Press'.format( name ),
					'font': {
						'color': 'rgb(37, 37, 37)',
						'family': 'Open Sans, sans-serif',
						'size': 24,
					},
					'x': distance[ actuation[0] ],
					'y': force_adc[ actuation[0] ],
					'xref' : 'x',
					'yref' : 'y',
					'showarrow' : True,
					'arrowhead' : 7,
					'arrowcolor' : 'rgb(88, 156, 189)',
					'ax' : 0,
					'ay' : -70,
				}])

			# Release
			if len( actuation ) > 1 and index == 1 and self.force_data.options['release_disable'] is False:
				annotations.extend([{
					'text' : '{0}Release'.format( name ),
					'font': {
						'color': 'rgb(37, 37, 37)',
						'family': 'Open Sans, sans-serif',
						'size': 24,
					},
					'x': distance[ actuation[1] ],
					'y': force_adc[ actuation[1] ],
					'xref' : 'x',
					'yref' : 'y',
					'showarrow' : True,
					'arrowhead' : 7,
					'arrowcolor' : 'rgb(232, 174, 90)',
					'ax' : 70,
					'ay' : 0,
				}])

	def process_annotations( self, graphs, annotations ):
		'''
		Use force curve analysis to generate annotations
		If not all data is present (such as actuation), not all annotations will be generated

		Annotation is ignored, if field is None
		'''
		from plotly.graph_objs import Scatter

		# Retrieve misc info about force curve(s)
		plot_data = self.force_data.get_var('extra_info')

		# If more than one switch, add the name as well
		name = ""
		if self.force_data.switches > 1:
			name = "{0}</br>".format( plot_data['test_name'] )

		# TODO - Generate data from current test
		# TODO - Add support for multiple data sets
		self.force_data.set_test( 1 )
		number = ""

		# Distance annotations
		rest_point       = self.force_data.get('rest_point')
		actuation_point  = self.force_data.get('actuation_point')
		release_range    = self.force_data.get('release_range')
		bottom_out_point = self.force_data.get('bottom_out_point')
		reset_point      = self.force_data.get('reset_point')
		repeat_range     = self.force_data.get('repeat_range')

		# Rest point - Line at distance between 0 and current force
		if rest_point is not None:
			graphs.extend([
				Scatter(
					x=[ rest_point[1][1], rest_point[1][1] ],
					y=[ 0, rest_point[1][0] ],
					name="{0}Rest point{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Actuation point - Line at actuation between 0 and current force
		if actuation_point is not None:
			graphs.extend([
				Scatter(
					x=[ actuation_point[1][1], actuation_point[1][1] ],
					y=[ 0, actuation_point[1][0] ],
					name="{0}Actuation point{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Release range - Filled force area between distance points
		if release_range[0] is not None:
			graphs.extend([
				Scatter(
					x=[ release_range[0][1][1], release_range[1][1][1] ],
					y=[ release_range[0][1][0], release_range[1][1][0] ],
					name="{0}Release range{1}".format( name, number ),
					fill='tozeroy',
					mode='none',
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Bottom-out point - Line at point before infinite force ramp, 0 and current force
		if bottom_out_point is not None:
			graphs.extend([
				Scatter(
					x=[ bottom_out_point[1][1], bottom_out_point[1][1] ],
					y=[ 0, bottom_out_point[1][0] ],
					name="{0}Bottom-out point{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Reset point - Line at reset between 0 and current force
		if reset_point is not None:
			graphs.extend([
				Scatter(
					x=[ reset_point[1][1], reset_point[1][1] ],
					y=[ 0, reset_point[1][0] ],
					name="{0}Reset point{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Repeat range - Filled force area between distance points
		if repeat_range[0] is not None:
			graphs.extend([
				Scatter(
					x=[ repeat_range[0][1][1], repeat_range[1][1][1] ],
					y=[ repeat_range[0][1][0], repeat_range[1][1][0] ],
					name="{0}Repeat range{1}".format( name, number ),
					fill='tozeroy',
					mode='none',
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])


		# Force annotations
		actuation_force  = self.force_data.get('actuation_force')
		reset_force      = self.force_data.get('reset_force')
		bottom_out_force = self.force_data.get('bottom_out_force')
		pre_load_force   = self.force_data.get('pre_load_force')

		# Actuation force - Line at actuation between current distance and 0
		if actuation_force is not None:
			graphs.extend([
				Scatter(
					x=[ 0, actuation_force[1][1] ],
					y=[ actuation_force[1][0], actuation_force[1][0] ],
					name="{0}Actuation force{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Reset force - Line at reset between current distance and 0
		if reset_force is not None:
			graphs.extend([
				Scatter(
					x=[ 0, reset_force[1][1] ],
					y=[ reset_force[1][0], reset_force[1][0] ],
					name="{0}Reset force{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Bottom-out force - Line at bottom-out between current distance and 0
		if bottom_out_force is not None:
			graphs.extend([
				Scatter(
					x=[ 0, bottom_out_force[1][1] ],
					y=[ bottom_out_force[1][0], bottom_out_force[1][0] ],
					name="{0}Bottom-out force{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

		# Pre-load force - Line at peak pre-load force between current distance and 0
		if pre_load_force is not None:
			graphs.extend([
				Scatter(
					x=[ 0, pre_load_force[1][1] ],
					y=[ pre_load_force[1][0], pre_load_force[1][0] ],
					name="{0}Pre-load force{1}".format( name, number ),
					line={
						'width' : plot_data['annotation_line_width'],
					},
				)
			])

	def read_option_setting( self, force_var, option_var ):
		'''
		Attempts to get a value for the layered dictionaries.
		First, look at the per force data json data.
		Second, look at the default options in the configuration file.
		'''
		value = ""
		if force_var in self.force_data.get_var('extra_info').keys():
			value = self.force_data.get_var('extra_info')[ force_var ]
		elif option_var in self.force_data.options.keys():
			value = self.force_data.options[ option_var ]

		return value

	def process_output( self ):
		'''
		Output force data from force_data structure to the output file
		'''
		# Retrieve misc info about force curve(s)
		plot_data = self.force_data.get_var('extra_info')

		# Setup metadata for grid (only used on uploads)
		updated_date = date.today().isoformat()
		self.metadata = {
			'Created'      : plot_data['created'],
			'Updated'      : updated_date,
			'Author'       : plot_data['author'],
			'Nick'         : plot_data['nick'],
			'Description'  : plot_data['description'],
			'URL'          : plot_data['url'],
			'Info Box'     : plot_data['info_box_desc'],
		}

		# Check if override has been set for scale ranges
		distance_bounds = 0
		distance_range = self.force_data.get_var('usable_distance_range')
		force_range = self.force_data.get_var('usable_force_range')
		distance_tick = 10
		if 'distance_bounds' in plot_data.keys():
			distance_bounds = plot_data['distance_bounds']
		if 'max_distance' in plot_data.keys():
			distance_range = ( 0 - distance_bounds, plot_data['max_distance'] + distance_bounds )
		if 'max_force' in plot_data.keys():
			force_range = ( 0, plot_data['max_force'] )
		if 'distance_tick' in plot_data.keys():
			distance_tick = plot_data['distance_tick']

		# If more than one switch, use a different title
		title = "{0} {1}".format( plot_data['vendor'], plot_data['popular_name'] )
		if self.force_data.switches > 1:
			title = "Switch Comparison"
		else:
			# Only set the vendor and part number if a single switch
			self.metadata['Vendor'] = plot_data['vendor']
			self.metadata['Popular Name'] = plot_data['popular_name']
			self.metadata['Part Number'] = plot_data['part']

		# List switches in description
		test_names = []
		for sw in range( 0, self.force_data.switches ):
			self.force_data.set_switch( sw )
			test_names.append( self.force_data.get_var('extra_info')['part'] )
		description_names = '</br>'.join( test_names )
		self.force_data.set_switch( 0 )

		# Build name line
		name_line = '<a href="{1}">{0}</a>@<a href="{3}">{2}</a></br>'.format(
			self.read_option_setting( 'nick', 'default_nick' ),
			self.read_option_setting( 'url', 'default_url' ),
			self.read_option_setting( 'org', 'default_org' ),
			self.read_option_setting( 'org_url', 'default_org_url' ),
		)

		# Description
		description_box = ''
		if plot_data['info_box_desc'] != "":
			description_box = '{0}</br>'.format( plot_data['info_box_desc'] )


		# Analysis Data
		#  Only using first curve
		rest_point = self.force_data.get_var('rest_point')
		bottom_out_point = self.force_data.get_var('bottom_out_point')

		# Attempt curve 1, then 2
		try:
			rest_position = rest_point[1][1][1]
			bottom_out_position = bottom_out_point[1][1][1]

			rest_force = int( round( rest_point[1][1][0] ) )
			bottom_out_force = int( round( bottom_out_point[1][1][0] ) )
		except:
			rest_position = rest_point[2][1][1]
			bottom_out_position = bottom_out_point[2][1][1]

			rest_force = int( round( rest_point[2][1][0] ) )
			bottom_out_force = int( round( bottom_out_point[2][1][0] ) )

		total_force_avg = int( round( self.force_data.get_var('total_force_avg') ) )
		analysis_data = ""
		analysis_data += "Total Energy: <b>{0} gfmm</b></br>".format(
			total_force_avg
		)
		# XXX (HaaTa) This is not true bottom out force yet...
		#analysis_data += "Max Force: {0} gf</br>".format( bottom_out_force )

		self.metadata['Rest Position'] = rest_position
		self.metadata['Bottom-out Position'] = bottom_out_position

		self.metadata['Rest Force'] = rest_force
		self.metadata['Bottom-out Force'] = bottom_out_force

		self.metadata['Total Energy'] = total_force_avg
		if self.force_data.get_var('actuation_force_avg') is not None:
			actuation_force_avg = int( round(self.force_data.get_var('actuation_force_avg') ) )
			analysis_data += "Actuation Energy: <b>{0} gfmm</b></br>".format(
				actuation_force_avg
			)

			# Get actuation data
			actuation_point = self.force_data.get_var('actuation_point')
			actuation_position = actuation_point[1][1][1]
			actuation_force = int( round( actuation_point[1][1][0] ) )
			reset_point = self.force_data.get_var('reset_point')
			reset_position = reset_point[1][1][1]
			reset_force = int( round( reset_point[1][1][0] ) )

			analysis_data += "Actuation Force: {0} gf</br>".format( actuation_force )
			self.metadata['Actuation Energy'] = actuation_force_avg
			self.metadata['Actuation Position'] = actuation_position
			self.metadata['Actuation Force'] = actuation_force
			self.metadata['Reset Position'] = reset_position
			self.metadata['Reset Force'] = reset_force
		else:
			self.metadata['Actuation Energy'] = ""
			self.metadata['Actuation Position'] = ""
			self.metadata['Actuation Force'] = ""
			self.metadata['Reset Position'] = ""
			self.metadata['Reset Force'] = ""


		# Graph infobox
		info_box = {
			'text' :
				'{0}</br></br>'
				'{5}'
				'{1}'
				'{2}'
				'{3}</br>'
				'<em>{4}</em>'.format(
					description_names,
					description_box,
					name_line,
					plot_data['created'], updated_date,
					analysis_data,
				),
			'bgcolor': 'rgba(0, 0, 0, 0)',
			'bordercolor': 'rgba(0, 0, 0, 0)',
			'borderwidth': 4,
			'borderpad': 0,
			'opacity': 1,
			'font': {
				'color': 'rgb(37, 37, 37)',
				'family': 'Open Sans, sans-serif',
				'size': 12,
			},
			'align' : 'right',
			'x': 1,
			'xanchor': 'right',
			'y': 0,
			'yanchor': 'bottom',
			'xref' : 'paper',
			'yref' : 'paper',
			'showarrow' : False,
		}
		print ( info_box['text'] )

		# Annotations
		annotations = [
			info_box,
		]

		# Setup calibration graphs
		graphs = []
		for switch in range( 0, self.force_data.switches ):
			# Set the switch index
			self.force_data.set_switch( switch )
			self.process_calibration( graphs )

		# Setup normal test graphs
		for switch in range( 0, self.force_data.switches ):
			# Set the switch index
			self.force_data.set_switch( switch )
			self.process_graphs( graphs, annotations )

		# Setup graph analysis annotations
		for switch in range( 0, self.force_data.switches ):
			# Set the switch index
			self.force_data.set_switch( switch )
			#self.process_annotations( graphs, annotations )


		# Layout/Theme Settings
		layout_settings = {
			'annotations' : annotations,
			'dragmode': 'zoom',
			'font': {
				'color': 'rgb(255, 255, 255)',
				'family': 'Open Sans, sans-serif',
				'size': 12
			},
			'hidesources': False,
			'hovermode': 'x',
			'legend': {
				'bgcolor': 'rgba(0, 0, 0, 0)',
				'bordercolor': 'rgba(0, 0, 0, 0)',
				'borderwidth': 10,
				'font': {
					'color': 'rgb(37, 37, 37)',
					'family': 'Open Sans, sans-serif',
					'size': 12
				},
				'traceorder': 'normal',
				'x': 0,
				'xanchor': 'left',
				'y': 1,
				'orientation': 'h',
				'yanchor': 'top'
			},
			'margin': {
				'pad': 15,
				't': 80,
				'l': 60,
				'r': 60,
				'b': 60,
				'autoexpand' : True
			},
			'paper_bgcolor': 'rgb(37, 37, 37)',
			'plot_bgcolor': '#fff',
			'separators': '.,',
			'showlegend': True,
			'title': title,
			'titlefont': {
				'color': 'rgb(255, 255, 255)',
				'family': 'Open Sans, sans-serif',
				'size': 36
			},
			'xaxis': {
				'autorange': False,
				'domain': [0, 1],
				'dtick': 1,
				'exponentformat': 'B',
				'gridcolor': '#eee',
				'gridwidth': 2,
				'linecolor': 'rgb(88, 156, 189)',
				'linewidth': 5,
				'mirror': False,
				'range': distance_range,
				'showexponent': 'all',
				'showgrid': True,
				'showline': False,
				'showticklabels': True,
				'tick0': 0,
				#'tickangle': 'auto',
				'tickcolor': 'rgb(88, 156, 189)',
				'tickfont': {
					'color': 'rgb(255, 255, 255)',
					'family': 'Open Sans, sans-serif',
					'size': 14
				},
				'ticklen': 11,
				'tickmode': 'linear',
				'ticks': 'inside',
				'tickwidth': 4,
				'title': 'Distance (mm)',
				'titlefont': {
					'color': 'rgb(255, 255, 255)',
					'family': 'Open Sans, sans-serif',
					'size': 20
				},
				'type': 'linear',
				'zeroline': True,
				'zerolinecolor': 'rgb(232, 174, 90)',
				'zerolinewidth': 2
			},
			'yaxis': {
				'anchor': 'x',
				'autorange': False,
				'domain': [0, 1],
				'dtick': distance_tick,
				'exponentformat': 'B',
				'gridcolor': '#eee',
				'gridwidth': 2,
				'linecolor': 'rgb(88, 156, 189)',
				'linewidth': 5,
				'mirror': 'ticks',
				'range': force_range,
				'showexponent': 'all',
				'showgrid': True,
				'showline': False,
				'showticklabels': True,
				'side': 'left',
				'tick0': 0,
				#'tickangle': 'auto',
				'tickcolor': 'rgb(88, 156, 189)',
				'tickfont': {
					'color': 'rgb(255, 255, 255)',
					'family': 'Open Sans, sans-serif',
					'size': 14
				},
				'ticklen': 11,
				'tickmode': 'linear',
				'ticks': 'inside',
				'tickwidth': 4,
				'title': 'Force (gf)',
				'titlefont': {
					'color': 'rgb(255, 255, 255)',
					'family': 'Open Sans, sans-serif',
					'size': 20
				},
				'type': 'linear',
				'zeroline': True,
				'zerolinecolor': 'rgb(232, 174, 90)',
				'zerolinewidth': 2
			}
		}

		# Generate plot
		force_curve = {
			'data'        : graphs,
			'layout'      : layout_settings,
		}

		# Generate filename based off of json data
		directory = "{0} Switches".format( plot_data['vendor'] )
		filename = "{0}/{1} {2} {3}".format(
			directory,
			plot_data['vendor'],
			plot_data['popular_name'],
			plot_data['part']
		)

		# Determine which upload API to use
		# Upload to plotly, overwrites if filename already exists
		if self.force_data.options['upload']:
			import plotly.plotly as py
			py.sign_in( self.force_data.options['plotly_user'], self.force_data.options['plotly_api_key'] )

		# Offline viewer (has option to upload)
		else:
			import plotly.offline as py

			# Append .html
			filename = "{0}.html".format( filename )

			# Make sure folder exists
			os.makedirs( directory, exist_ok=True )

		# Plot
		print( filename )
		if self.force_data.options['upload']:
			url = py.plot( force_curve, filename=filename, fileopt="overwrite" )
		else:
			url = py.plot( force_curve, filename=filename )

		# Add meta-data
		# XXX This looks way more complicated than it should be...
		#     Unfortunately Plotly thinks we should upload the grid first, and references that.
		#     But that's dumb, so we upload the plot+data first, figure out the grid, then upload the metadata to that.
		#     1) Upload plot (py.plot)
		#     2) Generate an API file sources to determine grid file id
		#     3) Query REST API with login details
		#     4) Find all associated grids with plot
		#     5) Build actual URLs for each grid
		#     6) Upload metadata to each grid
		if self.force_data.options['upload']:
			import json
			file_id = int( url.split('/')[4] ) # e.g. https://plot.ly/~haata/324/kaihua-bronze/
			new_url = "https://api.plot.ly/v2/files/{0}:{1}/sources".format(
				self.force_data.options['plotly_user'],
				file_id
			)
			print( "Sources:", new_url )
			json_data = json.loads( self.read_url( new_url ).text )

			# Find grid(s)
			for node in json_data['graph']['nodes']:
				if node['type'] == 'grid':
					file_id = node['metadata']['fid'].split(':')
					grid_url = "https://plot.ly/~{0}/{1}/".format( *file_id )

					print( "Grid Url:", grid_url )
					py.meta_ops.upload( self.metadata, grid_url=grid_url )

	def read_url( self, url ):
		import requests
		from requests.auth import HTTPBasicAuth
		r = requests.get(
			url,
			auth=HTTPBasicAuth( self.force_data.options['plotly_user'], self.force_data.options['plotly_api_key'] ),
			headers={ 'Plotly-Client-Platform' : 'python' },
		)
		if r.status_code != 200:
			print( "{0} Could not retrieve Plotly folders: {1}".format( ERROR, r ) )
			print( url )
			sys.exit( 1 )
		return r



### Argument Processing ###

def arg_set( arg, value_dict, name, default, override=None ):
	'''
	Checks if value is evaluated as Python "true"
	If so, use the arg in the dictionary, otherwise set to default
	'''
	if arg:
		if override is not None:
			value_dict[ name ] = override
		else:
			value_dict[ name ] = arg
	elif name not in value_dict.keys():
		value_dict[ name ] = default

def processCommandLineArgs( force_data ):
	'''
	Processes command line arguments and does basic error checking of inputs.
	'''
	# Setup argument processor
	pArgs = argparse.ArgumentParser(
		usage="%(prog)s [options] <input file1> [<output file2>..] -> <output file1> [<output file2>..]",
		description="This script takes a given input file, imports it, then converts to the format\n"
		"specified by the output file(s). If no extension is given, .raw is assumed.\n"
		".gz extension is recognized, and files will be unzip while reading.\n"
		"\n"
		"Supported Input Formats:\n"
		" - .raw\n"
		" - .fcv\n"
		" - .csv (*)\n"
		" - .xls/xlsx (*)\n"
		" - .png (*)\n"
		" - .svg (*)\n"
		"Any formats with a (*) must be in the correct format (i.e. generated by this script).\n"
		"\n"
		"Supported Output Formats:\n"
		" - .fcv\n"
		" - .csv\n"
		" - .xls/xlsx\n"
		" - .png\n"
		" - .svg\n"
		" - plotly\n",
		epilog="Example: {0} switch_test1.raw -> test1.fcv plotly test1.png".format( os.path.basename( sys.argv[0] ) ),
		formatter_class=argparse.RawTextHelpFormatter,
		add_help=False,
	)

	# Positional Arguments
	pArgs.add_argument( 'input_files', nargs='+', help=argparse.SUPPRESS ) # Suppressed help output
	pArgs.add_argument( 'output_files', nargs='*', help=argparse.SUPPRESS ) # Suppressed help output

	# Optional Arguments
	pArgs.add_argument( '-h', '--help', action="help",
		help="This message."
	)
	pArgs.add_argument( '--curves', type=str,
		help="Comma separated list of iterations to process. Will try to optimize processing time if possible (e.g. 0,4,3)"
	)
	pArgs.add_argument( '--no-press', action="store_true",
		help="Disable press curves."
	)
	pArgs.add_argument( '--no-release', action="store_true",
		help="Disable release curves."
	)
	pArgs.add_argument( '--upload', action="store_true",
		help="Uploads processed to remote location. e.g. Plotly"
	)
	pArgs.add_argument( '--annotations', action="store_true",
		help="Apply annotions to graph."
	)
	pArgs.add_argument( '-c', '--config', type=str,
		default=os.path.expanduser("~/.config/fcv_config.json"),
		help="Path to a json configuration file."
	)

	# Process Arguments
	args = pArgs.parse_args()

	error = False

	# Split input and output files
	input_files = args.input_files
	output_files = args.output_files

	# Default to plotly if not set
	if len(output_files) == 0:
		output_files = ['plotly']

	# Build list of curves to process
	curves = range(5) # TODO should be a range which matches the number of total curves
	if args.curves is not None:
		curves = [ int( num ) for num in args.curves.split(',') ]

	# Fill in default options
	options = {}
	if os.path.exists( args.config ):
		import json
		with open( args.config ) as fp:
			options = json.load( fp )
	else:
		print( "{0} '{1}' doesn't exist, config file will not be used.".format( WARNING, args.config ) )

	# Check if options were setA
	arg_set( args.annotations, options, 'annotations', False )
	arg_set( args.curves, options, 'curves', curves, curves )
	arg_set( args.no_press, options, 'press_disable', False )
	arg_set( args.no_release, options, 'release_disable', False )
	arg_set( args.upload, options, 'upload', False )

	# Set force data options
	force_data.set_options( options )

	# Get list of available classes
	classes = inspect.getmembers( sys.modules[ __name__ ], inspect.isclass )

	# Parameters
	valid_input_ext = {
		'.fcv'  : 'FCVForceData',
		'.csv'  : 'CSVForceData',
		'.xls'  : 'XLSForceData',
		'.xlsx' : 'XLSForceData',
		'.png'  : 'PNGForceData',
		'.svg'  : 'SVGForceData',
	}
	inputs = []
	for filename in input_files:
		split = os.path.splitext( filename )
		ext = split[1]

		# Setup datastructure for switch
		force_data.set_switch( force_data.switches )

		# Check if .gz file
		if split[1] == '.gz':
			ext = os.path.splitext( split[0] )[1]

		# Raw is handled differently as it's only an input
		if ext == '.raw':
			inputs.append( RawForceData( force_data, filename ) )

		elif ext not in valid_input_ext.keys():
			print( "{0} '{1}' is an invalid input extension in '{2}'".format( ERROR, ext, filename ) )
			error = True

		else:
			# Initialize input object
			class_object = [ cl for cl in classes if cl[0] == valid_input_ext[ ext ] ][0][1]
			inputs.append( class_object( force_data, filename, None ) )

	# Determine each type of output
	valid_output_ext = {
		'.fcv'  : 'FCVForceData',
		'.csv'  : 'CSVForceData',
		'.xls'  : 'XLSForceData',
		'.xlsx' : 'XLSForceData',
		'.png'  : 'PNGForceData',
		'.svg'  : 'SVGForceData',
	}
	outputs = []
	for filename in output_files:

		# Make sure extension is valid
		if filename != 'plotly':
			# Make sure there is an extension
			if '.' not in filename:
				print( "{0} '{1}' is an invalid output".format( ERROR, filename ) )
				error = True
				continue

			# Extract basename and extension
			name = os.path.splitext( filename )[0]
			ext = os.path.splitext( filename )[1]

			if ext not in valid_output_ext.keys():
				print( "{0} '{1}' is an invalid output extension in '{2}'".format( ERROR, ext, filename ) )
				error = True

			# Rename any duplicate files
			counter = 1
			while os.path.isfile( filename ):
				filename = "{0}-{1}.raw".format( name, counter )
				counter += 1

			# Initialize output object
			class_object = [ cl for cl in classes if cl[0] == valid_output_ext[ ext ] ][0][1]
			outputs.append( class_object( force_data, None, filename ) )

		# Plotly
		else:
			outputs.append( PlotlyForceData( force_data, filename ) )


	# Exit if there were any errors
	if error:
		sys.exit( 1 )

	return inputs, outputs



### Main ###

if __name__ == '__main__':
	force_data = ForceData()

	# Process args
	input_objects, output_objects = processCommandLineArgs( force_data )

	# Process input objects
	print ("Input Files")
	switch = 0
	for obj in input_objects:
		print ( "-- {0} --".format( obj.input_file ) )
		force_data.set_switch( switch )
		obj.process_input()
		switch += 1

	# Calibration
	print ("Calibration Analysis")
	force_data.set_test( 0 )
	for index in range( force_data.switches ):
		force_data.set_switch( index )
		force_data.calibration_analysis()

	# Analysis
	print ("Curve Analysis")
	for index in range( force_data.switches ):
		force_data.set_switch( index )
		force_data.curve_analysis()

	# Process output objects
	print ("Output Files")
	for obj in output_objects:
		print ( "-- {0} --".format( obj.output_file ) )
		obj.process_output()

	# Successful Execution
	sys.exit( 0 )


