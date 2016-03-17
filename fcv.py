#!/usr/bin/env python3
'''
fcv format conversion and analysis script
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

import argparse
import inspect
import os
import sys



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
		self.test_data = dict()
		self.analysis_data = dict()
		self.force_adc_serial_factor = None
		self.usable_distance_range = None
		self.usable_force_range = None

	def set_test( self, num ):
		'''
		Sets the current test to store/access ForceDataPoints
		'''
		self.cur_test = num

		# Prepare list
		if num not in self.test_data.keys():
			self.test_data[ num ] = []
			self.analysis_data[ num ] = []

	def add( self, data_point ):
		'''
		Adds a ForceDataPoint to the list
		'''
		self.test_data[ self.cur_test ].append( data_point )


	def calibration_analysis( self ):
		'''
		Analysis collected from calibration data
		'''
		import math
		import numpy as np
		import statistics

		for index, datapoint in enumerate( self.test_data[ self.cur_test ] ):
			try:
				force_factor = datapoint.force_adc / datapoint.force_serial
			except ZeroDivisionError:
				force_factor = 0

			point = AnalysisDataPoint(
				force_factor
			)
			self.analysis_data[ self.cur_test ].append( point )

		# Use the grouped median as the conversion factor
		self.force_adc_serial_factor = statistics.median_grouped(
			[ elem.force_adc_serial_factor for elem in self.analysis_data[ self.cur_test ] ]
		)

		# Determine distance range, use raw adc values from press
		adc_diff = list( np.diff( [ elem.force_adc for elem in self.test_data[ self.cur_test ][ :self.mid_point() ] ] ) )
		peaks = peakdet( adc_diff, 400 )[0] # TODO configurable delta and forced range
		first = adc_diff.index( peaks[0][1] )
		last = adc_diff.index( peaks[-1][1] )
		dist_mm = self.distance_mm()
		# Round up/down to the nearst int
		#self.usable_distance_range = ( math.ceil( dist_mm[ first ] ), math.floor( dist_mm[ last ] ) ) # XXX Technically better, but doesn't look as good -HaaTa
		self.usable_distance_range = ( math.ceil( dist_mm[ first ] ), 0 )

		# Use the max force as 2x the median grouped force over the newly calculated distance range (press)
		# TODO configurable
		force_data = self.force_adc_converted()[ first:last ]
		self.usable_force_range = ( 0, statistics.median_grouped( force_data ) * 2 )


	def force_adc( self ):
		'''
		Returns a list of force adc values for the current test
		'''
		return [ data.force_adc for data in self.test_data[ self.cur_test ] ]

	def force_serial( self ):
		'''
		Returns a list of force serial values for the current test
		'''
		data = [ data.force_serial for data in self.test_data[ self.cur_test ] ]
		return data

	def force_adc_converted( self ):
		'''
		Returns a list of force serial values for the current test
		'''
		data = [ data.force_adc / self.force_adc_serial_factor for data in self.test_data[ self.cur_test ] ]
		return data


	def distance_raw( self ):
		'''
		Returns a list of distance values for the current test
		'''
		data = [ data.distance for data in self.test_data[ self.cur_test ] ]
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
		#min_value = min( self.test_data[ self.cur_test ], key = lambda t: t.distance ).distance
		#return [ (data.distance - min_value) * 0.009921875 for data in self.test_data[ self.cur_test ] ]
		# Determine 0 point and release offset (starting from the bottom out)
		mm_conv = 0.009921875
		zero_point = self.test_data[ self.cur_test ][ self.mid_point_peak()[0] ].distance
		offset = (
			self.test_data[ self.cur_test ][ self.mid_point_peak()[0] ].distance
			- self.test_data[ self.cur_test ][ self.mid_point_peak()[1] ].distance
		)
		offset *= mm_conv
		mid_point = self.mid_point()

		# Compute lists with offsets
		press = [ (data.distance - zero_point) * mm_conv - offset for data in self.test_data[ self.cur_test ][:mid_point] ]
		release = [ (data.distance - zero_point) * mm_conv + offset for data in self.test_data[ self.cur_test ][mid_point:] ]

		press.extend( release )

		return press

	def mid_point( self ):
		'''
		Determines the direction change point of a test sequence
		'''
		return self.test_data[ self.cur_test ].index( min( self.test_data[ self.cur_test ], key = lambda t: t.distance ) )

	def mid_point_dir( self ):
		'''
		Determines the direction change point of a test sequence using the direction field (more accurate than mid_point)
		Look for the transition from 2 to 1
		2 - Down
		1 - Up
		'''
		prev = ForceDataPoint( None, None, None, None, None, None )
		position = None
		for index, elem in enumerate( self.test_data[ self.cur_test ] ):
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

		adc_diff = list( np.diff( [ elem.force_adc for elem in self.test_data[ self.cur_test ] ] ) )
		return ( adc_diff.index( max( adc_diff ) ), adc_diff.index( min( adc_diff ) ) )



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
		for index, elem in enumerate( self.test_data[ self.cur_test ] ):
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



class RawForceData( GenericForceData ):
	'''
	Class used to import and prepare force gauge data
	'''
	def __init__( self, force_data, input_file ):
		super( RawForceData, self ).__init__( force_data, input_file, None )

	def start( self ):
		'''
		Start of test handler
		'''
		self.cur_test = 0
		self.calibration = False
		self.force_data.set_test( self.cur_test )
		self.data_point_cache = [ None ]

		print ( "Processing Test #{0}".format( self.cur_test ) )

	def next_test( self ):
		'''
		Next test handler
		'''
		self.cur_test += 1
		self.calibration = False
		self.force_data.set_test( self.cur_test )

		print ( "Processing Test #{0}".format( self.cur_test ) )

	def finish( self ):
		'''
		Test complete handler
		'''
		self.calibration = False

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

		# XXX Old, add all points to line
		#self.force_data.add( data_point )

	def process_input( self ):
		'''
		Import force data from file into force_data structure
		'''
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
		for line in open( self.input_file, 'r' ):
			if line is None:
				continue

			# Check for event, call the associated function
			event = [ event for event in events.keys() if event in line ]
			if any( event ):
				method = getattr( self, events[ event[0] ] )
				method()
				continue

			# Check for data point, call the associated function
			point = [ point for point in data_points.keys() if point in line ]
			if any( point ):
				method = getattr( self, data_points[ point[0] ] )
				method( line )
				continue


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

	def process_output( self ):
		'''
		Output force data from force_data structure to the output file
		'''
		import plotly.offline as py
		from plotly.graph_objs import Layout, Scatter, Scattergl

		# TODO Get info from file/command line args
		plot_data = {
			'test_name'  : 'Cherry 01APBSW Switch',
			'file_name'  : 'Cherry_01APBSW.html',
			'info_box'   :
				'Cherry 01APBSW</br>'
				'(NOS) Loose Switch</br>'
				'<a href="http://kiibohd.com">HaaTa</a></br>'
				'2016-03-13',
			'line_width' : 3,
		}

		# Graph infobox
		info_box = {
			'text' : plot_data['info_box'],
			'bgcolor': 'rgb(255, 255, 255)',
			'bordercolor': 'rgb(232, 174, 90)',
			'borderwidth': 4,
			'font': {
				'color': 'rgb(37, 37, 37)',
				'family': 'Open Sans, sans-serif',
				'size': 12,
			},
			'align' : 'left',
			'x': 1.06,
			'xanchor': 'right',
			'y': 0,
			'yanchor': 'bottom',
			'xref' : 'paper',
			'yref' : 'paper',
		}

		# Annotations
		annotations = [
			info_box,
		]

		# Acquire calibration data
		self.force_data.set_test( 0 )
		force_adc = self.force_data.force_adc_converted()
		force_serial = self.force_data.force_serial()
		distance = self.force_data.distance_mm()
		mid_point = self.force_data.mid_point()

		# Setup calibration graphs
		# TODO convert to Scattergl when less buggy -HaaTa
		graphs = [
			Scatter(
				x=distance[:mid_point],
				y=force_serial[:mid_point],
				name="RS232 (Cal) Press",
				legendgroup='rs232',
				line={
					'width' : plot_data['line_width'],
				},
				visible='legendonly',
			),
			Scatter(
				x=distance[mid_point:],
				y=force_serial[mid_point:],
				name="RS232 (Cal) Release",
				legendgroup='rs232',
				line={
					'width' : plot_data['line_width'],
				},
				visible='legendonly',
			),
			Scatter(
				x=distance[:mid_point],
				y=force_adc[:mid_point],
				name="Analog (Cal) Press",
				legendgroup='analog',
				line={
					'width' : plot_data['line_width'],
				},
				visible='legendonly',
			),
			Scatter(
				x=distance[mid_point:],
				y=force_adc[mid_point:],
				name="Analog (Cal) Release",
				legendgroup='analog',
				line={
					'width' : plot_data['line_width'],
				},
				visible='legendonly',
			),
		]

		# Setup normal test graphs
		# TODO handle more than one graph
		self.force_data.set_test( 1 )
		mid_point = self.force_data.mid_point_dir()
		force_adc = self.force_data.force_adc_converted()
		distance = self.force_data.distance_mm()
		actuation = self.force_data.actuation()

		# TODO Is this the best way to average points?
		# For distance positions with multiple readings, do an average for each reading
		# TODO

		graphs.extend([
			Scatter(
				x=distance[:mid_point],
				y=force_adc[:mid_point],
				name="Press",
				legendgroup='test1',
				line={
					'width' : plot_data['line_width'],
				},
			),
			Scatter(
				x=distance[mid_point:],
				y=force_adc[mid_point:],
				name="Release",
				legendgroup='test1',
				line={
					'width' : plot_data['line_width'],
				},
			),
		])

		# Press
		if len( actuation ) > 0:
			annotations.extend([{
				'text' : 'Press',
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
		if len( actuation ) > 1:
			annotations.extend([{
				'text' : 'Release',
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
				'ax' : 0,
				'ay' : 70,
			}])

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
				'bgcolor': 'rgb(255, 255, 255)',
				'bordercolor': 'rgb(88, 156, 189)',
				'borderwidth': 4,
				'font': {
					'color': 'rgb(37, 37, 37)',
					'family': 'Open Sans, sans-serif',
					'size': 12
				},
				'traceorder': 'normal',
				'x': 1.11,
				'xanchor': 'right',
				'y': 1,
				'yanchor': 'top'
			},
			'margin': {
				'pad': 0, 't': 110
			},
			'paper_bgcolor': 'rgb(37, 37, 37)',
			'plot_bgcolor': '#fff',
			'separators': '.,',
			'showlegend': True,
			'title': plot_data['test_name'],
			'titlefont': {
				'color': 'rgb(255, 255, 255)',
				'family': 'Open Sans, sans-serif',
				'size': 36
			},
			'xaxis': {
				'autorange': False,
				'domain': [0.01, 0.99],
				'dtick': 1,
				'exponentformat': 'B',
				'gridcolor': '#eee',
				'gridwidth': 2,
				'linecolor': 'rgb(88, 156, 189)',
				'linewidth': 5,
				'mirror': False,
				'range': self.force_data.usable_distance_range,
				'showexponent': 'all',
				'showgrid': True,
				'showline': True,
				'showticklabels': True,
				'tick0': 0,
				'tickangle': 'auto',
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
				'domain': [0.01, 1],
				'dtick': 10,
				'exponentformat': 'B',
				'gridcolor': '#eee',
				'gridwidth': 2,
				'linecolor': 'rgb(88, 156, 189)',
				'linewidth': 5,
				'mirror': 'ticks',
				'range': self.force_data.usable_force_range,
				'showexponent': 'all',
				'showgrid': True,
				'showline': True,
				'showticklabels': True,
				'side': 'left',
				'tick0': 0,
				'tickangle': 'auto',
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

		py.plot( force_curve, filename=plot_data['file_name'], )






### Argument Processing ###

def processCommandLineArgs( force_data ):
	'''
	Processes command line arguments and does basic error checking of inputs.
	'''
	# Setup argument processor
	pArgs = argparse.ArgumentParser(
		usage="%(prog)s [options] <input file> <output file1> [<output file2>..]",
		description="This script takes a given input file, imports it, then converts to the format\n"
		"specified by the output file(s). If not extension is given, .raw is assumed.\n"
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
		epilog="Example: {0} switch_test1.raw test1.fcv plotly test1.png".format( os.path.basename( sys.argv[0] ) ),
		formatter_class=argparse.RawTextHelpFormatter,
		add_help=False,
	)

	# Positional Arguments
	pArgs.add_argument( 'input_file', help=argparse.SUPPRESS ) # Suppressed help output
	pArgs.add_argument( 'output_files', nargs='+', help=argparse.SUPPRESS ) # Suppressed help output

	# Optional Arguments
	pArgs.add_argument( '-h', '--help', action="help",
		help="This message."
	)

	# Process Arguments
	args = pArgs.parse_args()

	error = False

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
	input_filename = args.input_file
	ext = os.path.splitext( input_filename )[1]

	# Raw is handled differently as it's only an input
	inputs = []
	if ext == '.raw':
		inputs.append( RawForceData( force_data, input_filename ) )

	elif ext not in valid_input_ext.keys():
		print( "{0} '{1}' is an invalid input extension in '{2}'".format( ERROR, ext, input_filename ) )
		error = True

	else:
		# Initialize input object
		class_object = [ cl for cl in classes if cl[0] == valid_input_ext[ ext ] ][0][1]
		inputs.append( class_object( force_data, input_filename, None ) )

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
	for filename in args.output_files:

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
	forcedata = ForceData()

	# Process args
	input_objects, output_objects = processCommandLineArgs( forcedata )

	# Process input objects
	print ("Input Files")
	for obj in input_objects:
		obj.process_input()

	# Analysis
	print ("Analysis")
	forcedata.set_test( 0 )
	forcedata.calibration_analysis()

	# Process output objects
	print ("Output Files")
	for obj in output_objects:
		obj.process_output()

	# Successful Execution
	sys.exit( 0 )


