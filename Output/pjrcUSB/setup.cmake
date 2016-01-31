###| CMake Kiibohd Controller USB Module |###
#
# Written by Jacob Alexander in 2011-2016 for the Kiibohd Controller
#
# Released into the Public Domain
#
###


###
# Module C files
#

#| ARM Compiler
if ( ${COMPILER_FAMILY} MATCHES "arm" )

	set ( Module_SRCS
		output_com.c
		arm/usb_desc.c
		arm/usb_dev.c
		arm/usb_mem.c
		arm/usb_rawio.c
		arm/usb_serial.c
	)

endif ()


###
# Compiler Family Compatibility
#
set( ModuleCompatibility
	arm
)

