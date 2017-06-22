#!/usr/bin/env python3
# Jacob Alexander 2017
# Uses Plotly and Google Sheets APIs to synchronize a google doc spreadsheet
# Data may entered into the google doc spreadsheet separately

import argparse
import concurrent.futures
import json
import os
import sys
import urllib.parse as urlparse

import gspread
import requests

from oauth2client.service_account import ServiceAccountCredentials
from requests.auth import HTTPBasicAuth



# Print Decorator Variables
ERROR = '\033[5;1;31mERROR\033[0m:'
WARNING = '\033[1;33mWARNING\033[0m:'


## Python Text Formatting Fixer...
##  Because the creators of Python are averse to proper capitalization.
textFormatter_lookup = {
	"usage: "              : "\033[1mUsage\033[0m: ",
	"optional arguments"   : "\033[1mOptional Arguments\033[0m",
	"positional arguments" : "\033[1mPositional Arguments\033[0m",
}

def textFormatter_gettext( s ):
	return textFormatter_lookup.get( s, s )

argparse._ = textFormatter_gettext


# Parse Arguments
parser = argparse.ArgumentParser(
	formatter_class=argparse.RawDescriptionHelpFormatter,
)
parser.add_argument("--config",
	default=os.path.expanduser("~/.config/plotly_gdoc_sync.json"),
	help="Config file containing API keys and google doc",
)
parser.add_argument("--list-plotly-switches",
	action="store_true",
	help="List switches stored in plotly",
)
parser.add_argument("--list-plotly-missing-metadata",
	action="store_true",
	help="List switches in plotly missing metadata",
)
parser.add_argument("--list-plotly-metadata",
	action="store_true",
	help="List switches in plotly with metadata",
)
parser.add_argument("--list-plotly-metadata-detailed",
	action="store_true",
	help="List switches in plotly with metadata, including the metadata",
)
parser.add_argument("--list-gdoc-switches",
	action="store_true",
	help="List switches stored in gdoc",
)
parser.add_argument("--list-gdoc-switches-no-id",
	action="store_true",
	help="List switches stored in gdoc without Plotly unique identifiers",
)
parser.add_argument("--sync-plotly-to-gdoc",
	action="store_true",
	help="Synchronizes Plotly data to Google Doc.",
)


args = parser.parse_args()
options = {}
if os.path.exists( args.config ):
	with open( args.config ) as fp:
		options = json.load( fp )
else:
	print( "{0} '{1}' doesn't exist. Must specify configuration file for API keys.".format( ERROR, args.config ) )



class PlotlyData:
	'''
	Class for reading switch data from Plotly
	'''
	def __init__( self ):
		# Authenticate with Plotly
		self.auth = HTTPBasicAuth( options['plotly_user'], options['plotly_api_key'] )
		self.headers = { 'Plotly-Client-Platform' : 'python' }

	def read_url( self, url ):
		r = requests.get(
			url,
			auth=self.auth,
			headers=self.headers
		)
		if r.status_code != 200:
			print( "{0} Could not retrieve Plotly folders: {1}".format( ERROR, r ) )
			print( url )
			sys.exit( 1 )
		return r

	def get_all_pages( self, url ):
		data = []

		# Get initial page
		user = options['plotly_query_user']
		r = self.read_url( url.format( user, 1 ) )
		data.append( json.loads( r.text )['children'] )

		# Determine total pages to read
		try:
			next_page = int( urlparse.parse_qs( urlparse.urlparse( data[-1]['next'] ).query )['page'][0] )
			last_page = int( urlparse.parse_qs( urlparse.urlparse( data[-1]['last'] ).query )['page'][0] )
		except:
			# There may not be any next/last pages
			return data

		urls = [ url.format( user, num ) for num in range( next_page, last_page + 1 ) ]

		# Concurrently read generated list of urls
		with concurrent.futures.ThreadPoolExecutor( max_workers=10 ) as executor:
			future_to_url = {
				executor.submit( self.read_url, url ) : url for url in urls
			}
			for future in concurrent.futures.as_completed( future_to_url ):
				url = future_to_url[ future ]
				try:
					data.append( json.loads( future.result().text )['children'] )
				except Exception as exc:
					print( "{0} generated an exception: {1}".format( ERROR, exc ) )
		return data

	def get_all_folders( self ):
		# Get all page data
		data = self.get_all_pages('https://api.plot.ly/v2/folders/home?user={}&page={}&world_readable=true')

		# Parse over results and filter out folders
		folders = {}
		for elem in data:
			for result in elem['results']:
				if result['filetype'] == 'fold':
					folders[ result['api_urls']['folders'] ] = result['filename']

		return folders

	def get_plots_in_folders( self, url, folder ):
		# Get all page data
		data = self.get_all_pages( "{0}?user={{}}&page={{}}".format( url ) )

		# Gather results, only grid and plots
		results = []
		for elem in data:
			for result in elem['results']:
				if result['filetype'] == 'plot' or result['filetype'] == 'grid':
					results.append( result )

		# Return a tuple so we know where the data came from
		return (results, url)

	def get_switches( self, verbose=False ):
		#print( json.dumps( data[-1], sort_keys=True, indent=4 ) )
		#print( json.dumps( data['children'], sort_keys=True, indent=4 ) )
		# Get list of folders (world readable)
		folders = self.get_all_folders()

		# Get list of switches
		switches = {}

		# Parallelize API calls to retrieve list of switches
		with concurrent.futures.ThreadPoolExecutor( max_workers=100 ) as executor:
			future_to_url = { executor.submit( self.get_plots_in_folders, *item ) : item for item in folders.items() }
			for future in concurrent.futures.as_completed( future_to_url ):
				url = future_to_url[ future ]
				try:
					result = future.result()
					# Returns a tuple, use the url as the dictionary key
					switches[ result[1] ] = result[0]
				except Exception as exc:
					print( "{0} generated an exception: {1}".format( ERROR, exc ) )

		# Sort by folder name
		total_switches = 0
		for url, name in sorted( folders.items(), key=lambda items: items[1] ):
			if verbose:
				print( name, url )
			switch_list = []
			for elem in switches[ url ]:
				if elem['filetype'] == 'plot':
					switch_list.append( elem )
					total_switches += 1
			for switch in sorted( switch_list, key=lambda items: items['filename'] ):
				if verbose:
					print( "\t{0}".format( switch['filename'] ) )

		# Print total number of switch plots
		if verbose:
			print( "Total Switches:", total_switches )

		return switches, folders

	def check_switch_metadata( self, switch, has, show ):
		new_url = switch['api_urls']['grids']
		json_data = json.loads( self.read_url( new_url ).text )
		#print( json.dumps( json_data, sort_keys=True, indent=4 ) )
		metadata_len = len( json_data['metadata'].keys() )
		if has and metadata_len > 0 or not has and metadata_len == 0:
			if show:
				print( "{0} - {1}".format( json_data['filename'], json_data['fid'] ) )
		return ( new_url, json_data['metadata'], json_data['web_url'] )


	def check_all_switch_metadata( self, has=False, show=False, detailed_show=False ):
		switches, folders = p_data.get_switches()
		switch_list = []

		# Sort by Vendor
		for url, name in sorted( folders.items(), key=lambda items: items[1] ):
			# Find grid files to build lists
			for elem in switches[ url ]:
				if elem['filetype'] == 'grid':
					switch_list.append( elem )

		metadata_list = []

		# Parallelize API calls to retrieve list of switches
		with concurrent.futures.ThreadPoolExecutor( max_workers=100 ) as executor:
			future_to_url = { executor.submit( self.check_switch_metadata, switch, has, show ) : switch for switch in switch_list }
			for future in concurrent.futures.as_completed( future_to_url ):
				url = future_to_url[ future ]
				try:
					# Returns a tuple, use the url as the dictionary key
					result = future.result()
					if detailed_show and len( result[1].keys() ) > 0:
						print( result[2], result[0] )
						print( json.dumps( result[1], sort_keys=True, indent=4 ) )

					# Append tuple to returning list
					if len( result[1].keys() ) > 0:
						metadata_list.append( result )
				except Exception as exc:
					print( "{0} generated an exception: {1}".format( ERROR, exc ) )

		return metadata_list


class GDocData:
	'''
	Class for read/writing to google doc spreadsheets
	'''
	def __init__( self ):
		self.credentials = ServiceAccountCredentials.from_json_keyfile_name(
			options['gdoc_oauth_json'],
			options['gdoc_scope'],
		)
		self.gc = gspread.authorize( self.credentials )
		self.sheet = self.gc.open_by_url( options['gdoc_url'] )
		self.worksheet = self.sheet.get_worksheet(0)

	def get_switches( self, verbose=False ):
		# Get list of switches
		switches = {}

		# Iterate over each switch entry
		for switch in self.worksheet.get_all_records():
			name = switch['Popular Name']
			if switch['Part Number'] != "":
				name = "{0} {1}".format( name, switch['Part Number'] )

			# Insert switch into dictionary
			if switch['Brand'] not in switches.keys():
				switches[ switch['Brand'] ] = [ ( name, switch ) ]
			else:
				switches[ switch['Brand'] ].append( ( name, switch ) )

		# Sort by brand
		for brand, elem in sorted( switches.items(), key=lambda items: items[0] ):
			if verbose:
				print( "{0} Switches".format( brand ) )
			for switch in sorted( elem, key=lambda items: items[0] ):
				if verbose:
					print( "\t{0} {1}".format( brand, switch[0] ) )

		return switches


### Main ###

# Initialize APIs
p_data = PlotlyData()
g_data = GDocData()


# List plotly switches
if args.list_plotly_switches:
	p_data.get_switches( verbose=True )
	sys.exit(0)

# List gdoc switches
if args.list_gdoc_switches:
	g_data.get_switches( verbose=True )
	sys.exit(0)

# List plotly switches missing metadata
if args.list_plotly_missing_metadata:
	p_data.check_all_switch_metadata( False, True )
	sys.exit(0)

# List plotly switches with metadata
if args.list_plotly_metadata:
	p_data.check_all_switch_metadata( True, True )
	sys.exit(0)

# List plotly switches with detailed metadata, i.e. show all metadata for each switch
if args.list_plotly_metadata_detailed:
	# - List all switches (like list_plotly_metadata), but also show the metadata dictionary information
	p_data.check_all_switch_metadata( True, True, True )
	sys.exit(0)

# List gdoc switches without a plotly unique identifier
if args.list_gdoc_switches_no_id:
	# TODO
	# - List gdoc switches without a plotly unique identifier
	sys.exit(0)

# Synchronize Plotly to Google Doc
if args.sync_plotly_to_gdoc:
	# TODO
	# Query each plotly switch
	# - Use Plotly Link as unique identifier (i.e. https://plot.ly/~haata/268)
	# - If no matching Plotly Link, use the Popular Name + Part Number fields to identify
	#   If more than one switch matches the Popular Name + Part Number, do not update, and display a warning
	# - If no
	# If any field (metadata) is not present or empty (""), do not update in gdoc
	# When in doubt, warn and do not update
	sys.exit(0)

