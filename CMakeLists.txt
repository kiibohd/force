###| CMAKE Kiibohd Controller |###
#
# Jacob Alexander 2011-2015
# Due to this file's usefulness:
#
# Released into the Public Domain
#
###



###
# Chip Selection
#

#| You _MUST_ set this to match the microcontroller you are trying to compile for
#| You _MUST_ clean the build directory if you change this value
#|
set( CHIP
#       "at90usb162"       # Teensy   1.0 (avr)
#       "atmega32u4"       # Teensy   2.0 (avr)
#       "at90usb646"       # Teensy++ 1.0 (avr)
#       "at90usb1286"      # Teensy++ 2.0 (avr)
#       "mk20dx128"        # Teensy   3.0 (arm)
#	"mk20dx128vlf5"    # McHCK       mk20dx128vlf5
        "mk20dx256"        # Teensy   3.1 (arm)
#       "mk20dx256vlh7"    # Kiibohd-dfu mk20dx256vlh7
	CACHE STRING "Microcontroller Chip" )



###
# Compiler Selection
#

#| *** EXPERIMENTAL ***
#| Stick with gcc unless you know what you're doing
#| Currently only arm is supported with clang
set( COMPILER
	"gcc"   # arm-none-eabi-gcc / avr-gcc - Default
#       "clang" # arm-none-eabi
	CACHE STRING "Compiler Type" )



###
# Compiler Intialization
#
set ( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/Lib/CMake )
include( initialize )



###
# Project Modules
#

#| Note: This is the only section you probably want to modify
#| Each module is defined by it's own folder (e.g. Scan/Matrix represents the "Matrix" module)
#| All of the modules must be specified, as they generate the sources list of files to compile
#| Any modifications to this file will cause a complete rebuild of the project

#| Please look at the {Output,Debug} for information on the modules and how to create new ones

##| Sends the current list of usb key codes through USB HID
set( OutputModule "pjrcUSB"
	CACHE STRING "Output Module" )

##| Debugging source to use, each module has it's own set of defines that it sets
set(  DebugModule "full"
	CACHE STRING "Debug Module" )



###
# Source Defines (in addition to the selected Modules)
#
set( MAIN_SRCS
	main.c
)



###
# Project Description
#

#| Project
project( kiibohd_controller )

#| Target Name (output name)
set( TARGET kiibohd )

#| General Settings
cmake_minimum_required( VERSION 2.8 )



###
# Module Initialization / Compilation / Targets
#
include( modules )
include( build )

