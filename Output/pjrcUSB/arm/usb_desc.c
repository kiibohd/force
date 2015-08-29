/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 * Modified by Jacob Alexander (2013-2015)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// ----- Includes -----

// Local Includes
#include "usb_desc.h"



// ----- Macros -----

#define LSB(n) ((n) & 255)
#define MSB(n) (((n) >> 8) & 255)



// ----- USB Device Descriptor -----

// USB Device Descriptor.  The USB host reads this first, to learn
// what type of device is connected.
static uint8_t device_descriptor[] = {
	18,                                     // bLength
	1,                                      // bDescriptorType
	0x00, 0x02,                             // bcdUSB
	DEVICE_CLASS,                           // bDeviceClass
	DEVICE_SUBCLASS,                        // bDeviceSubClass
	DEVICE_PROTOCOL,                        // bDeviceProtocol
	EP0_SIZE,                               // bMaxPacketSize0
	LSB(VENDOR_ID), MSB(VENDOR_ID),         // idVendor
	LSB(PRODUCT_ID), MSB(PRODUCT_ID),       // idProduct
	0x00, 0x01,                             // bcdDevice
	1,                                      // iManufacturer
	2,                                      // iProduct
	3,                                      // iSerialNumber
	1                                       // bNumConfigurations
};

// USB Device Qualifier Descriptor
static uint8_t device_qualifier_descriptor[] = {
	0                                       // Indicate only single speed
	/* Device qualifier example (used for specifying multiple USB speeds)
	10,                                     // bLength
	6,                                      // bDescriptorType
	0x00, 0x02,                             // bcdUSB
	DEVICE_CLASS,                           // bDeviceClass
	DEVICE_SUBCLASS,                        // bDeviceSubClass
	DEVICE_PROTOCOL,                        // bDeviceProtocol
	EP0_SIZE,                               // bMaxPacketSize0
	0,                                      // bNumOtherSpeedConfigurations
	0                                       // bReserved
	*/
};

// USB Debug Descriptor
// XXX Not sure of exact use, lsusb requests it
static uint8_t usb_debug_descriptor[] = {
	0
};

// XXX
// These descriptors must NOT be "const", because the USB DMA
// has trouble accessing flash memory with enough bandwidth
// while the processor is executing from flash.
// XXX



// ----- USB Configuration -----

// USB Configuration Descriptor.  This huge descriptor tells all
// of the devices capbilities.
static uint8_t config_descriptor[CONFIG_DESC_SIZE] = {
// --- Configuration ---
// - 9 bytes -
	// configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
	9,                                      // bLength;
	2,                                      // bDescriptorType;
	LSB(CONFIG_DESC_SIZE),                  // wTotalLength
	MSB(CONFIG_DESC_SIZE),
	NUM_INTERFACE,                          // bNumInterfaces
	1,                                      // bConfigurationValue
	0,                                      // iConfiguration
	0xA0,                                   // bmAttributes
	250,                                    // bMaxPower

// --- Serial CDC --- CDC IAD Descriptor
// - 8 bytes -
	// interface association descriptor, USB ECN, Table 9-Z
	8,                                      // bLength
	11,                                     // bDescriptorType
	CDC_STATUS_INTERFACE,                   // bFirstInterface
	2,                                      // bInterfaceCount
	0x02,                                   // bFunctionClass
	0x02,                                   // bFunctionSubClass
	0x01,                                   // bFunctionProtocol
	CDC_STATUS_INTERFACE + 4,               // iFunction

// --- Serial CDC --- CDC Data Interface
// - 9 bytes -
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,                                      // bLength
	4,                                      // bDescriptorType
	CDC_STATUS_INTERFACE,                   // bInterfaceNumber
	0,                                      // bAlternateSetting
	1,                                      // bNumEndpoints
	0x02,                                   // bInterfaceClass
	0x02,                                   // bInterfaceSubClass
	0x01,                                   // bInterfaceProtocol
	CDC_STATUS_INTERFACE + 4,               // iInterface
// - 5 bytes -
	// CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
	5,                                      // bFunctionLength
	0x24,                                   // bDescriptorType
	0x00,                                   // bDescriptorSubtype
	0x10, 0x01,                             // bcdCDC
// - 5 bytes -
	// Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
	5,                                      // bFunctionLength
	0x24,                                   // bDescriptorType
	0x01,                                   // bDescriptorSubtype
	0x01,                                   // bmCapabilities
	CDC_DATA_INTERFACE,                     // bDataInterface
// - 4 bytes -
	// Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
	4,                                      // bFunctionLength
	0x24,                                   // bDescriptorType
	0x02,                                   // bDescriptorSubtype
	0x06,                                   // bmCapabilities
// - 5 bytes -
	// Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
	5,                                      // bFunctionLength
	0x24,                                   // bDescriptorType
	0x06,                                   // bDescriptorSubtype
	CDC_STATUS_INTERFACE,                   // bMasterInterface
	CDC_DATA_INTERFACE,                     // bSlaveInterface0
// - 7 bytes -
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                      // bLength
	5,                                      // bDescriptorType
	CDC_ACM_ENDPOINT | 0x80,                // bEndpointAddress
	0x03,                                   // bmAttributes (0x03=intr)
	CDC_ACM_SIZE, 0,                        // wMaxPacketSize
	64,                                     // bInterval
// - 9 bytes -
	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
	9,                                      // bLength
	4,                                      // bDescriptorType
	CDC_DATA_INTERFACE,                     // bInterfaceNumber
	0,                                      // bAlternateSetting
	2,                                      // bNumEndpoints
	0x0A,                                   // bInterfaceClass
	0x00,                                   // bInterfaceSubClass
	0x00,                                   // bInterfaceProtocol
	CDC_DATA_INTERFACE + 4,                 // iInterface
// - 7 bytes -
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                      // bLength
	5,                                      // bDescriptorType
	CDC_RX_ENDPOINT,                        // bEndpointAddress
	0x02,                                   // bmAttributes (0x02=bulk)
	CDC_RX_SIZE, 0,                         // wMaxPacketSize
	0,                                      // bInterval
// - 7 bytes -
	// endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
	7,                                      // bLength
	5,                                      // bDescriptorType
	CDC_TX_ENDPOINT | 0x80,                 // bEndpointAddress
	0x02,                                   // bmAttributes (0x02=bulk)
	CDC_TX_SIZE, 0,                         // wMaxPacketSize
	0,                                      // bInterval
};



// ----- String Descriptors -----

// The descriptors above can provide human readable strings,
// referenced by index numbers.  These descriptors are the
// actual string data

struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wString[];
};

extern struct usb_string_descriptor_struct usb_string_manufacturer_name
	__attribute__ ((weak, alias("usb_string_manufacturer_name_default")));
extern struct usb_string_descriptor_struct usb_string_product_name
	__attribute__ ((weak, alias("usb_string_product_name_default")));
extern struct usb_string_descriptor_struct usb_string_serial_number
	__attribute__ ((weak, alias("usb_string_serial_number_default")));

struct usb_string_descriptor_struct string0 = {
	4,
	3,
	{0x0409}
};

#define usb_string_descriptor(name, str) \
	struct usb_string_descriptor_struct name = { \
		sizeof(str), \
		3, \
		{str} \
	}

usb_string_descriptor( usb_string_manufacturer_name_default, STR_MANUFACTURER );
usb_string_descriptor( usb_string_product_name_default, STR_PRODUCT );
usb_string_descriptor( usb_string_serial_number_default, STR_SERIAL );
usb_string_descriptor( usb_string_cdc_status_name, CDC_STATUS_NAME );
usb_string_descriptor( usb_string_cdc_data_name, CDC_DATA_NAME );



// ----- Descriptors List -----

// This table provides access to all the descriptor data above.

const usb_descriptor_list_t usb_descriptor_list[] = {
	//wValue, wIndex, address,          length
	{0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
	{0x0200, 0x0000, config_descriptor, sizeof(config_descriptor)},
	{0x0600, 0x0000, device_qualifier_descriptor, sizeof(device_qualifier_descriptor)},
	{0x0A00, 0x0000, usb_debug_descriptor, sizeof(usb_debug_descriptor)},

#define iInterfaceString(num, var) \
	{0x0300 + 4 + num, 0x409, (const uint8_t *)&var, 0 }

	{0x0300, 0x0000, (const uint8_t *)&string0, 0},
	{0x0301, 0x0409, (const uint8_t *)&usb_string_manufacturer_name, 0},
	{0x0302, 0x0409, (const uint8_t *)&usb_string_product_name, 0},
	{0x0303, 0x0409, (const uint8_t *)&usb_string_serial_number, 0},
	iInterfaceString( CDC_STATUS_INTERFACE, usb_string_cdc_status_name ),
	iInterfaceString( CDC_DATA_INTERFACE, usb_string_cdc_data_name ),
	{0, 0, NULL, 0}
};



// ----- Endpoint Configuration -----

// See usb_desc.h for Endpoint configuration
// 0x00 = not used
// 0x19 = Recieve only
// 0x15 = Transmit only
// 0x1D = Transmit & Recieve
//
const uint8_t usb_endpoint_config_table[NUM_ENDPOINTS] =
{
#if (defined(ENDPOINT1_CONFIG) && NUM_ENDPOINTS >= 1)
	ENDPOINT1_CONFIG,
#elif (NUM_ENDPOINTS >= 1)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT2_CONFIG) && NUM_ENDPOINTS >= 2)
	ENDPOINT2_CONFIG,
#elif (NUM_ENDPOINTS >= 2)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT3_CONFIG) && NUM_ENDPOINTS >= 3)
	ENDPOINT3_CONFIG,
#elif (NUM_ENDPOINTS >= 3)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT4_CONFIG) && NUM_ENDPOINTS >= 4)
	ENDPOINT4_CONFIG,
#elif (NUM_ENDPOINTS >= 4)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT5_CONFIG) && NUM_ENDPOINTS >= 5)
	ENDPOINT5_CONFIG,
#elif (NUM_ENDPOINTS >= 5)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT6_CONFIG) && NUM_ENDPOINTS >= 6)
	ENDPOINT6_CONFIG,
#elif (NUM_ENDPOINTS >= 6)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT7_CONFIG) && NUM_ENDPOINTS >= 7)
	ENDPOINT7_CONFIG,
#elif (NUM_ENDPOINTS >= 7)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT8_CONFIG) && NUM_ENDPOINTS >= 8)
	ENDPOINT8_CONFIG,
#elif (NUM_ENDPOINTS >= 8)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT9_CONFIG) && NUM_ENDPOINTS >= 9)
	ENDPOINT9_CONFIG,
#elif (NUM_ENDPOINTS >= 9)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT10_CONFIG) && NUM_ENDPOINTS >= 10)
	ENDPOINT10_CONFIG,
#elif (NUM_ENDPOINTS >= 10)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT11_CONFIG) && NUM_ENDPOINTS >= 11)
	ENDPOINT11_CONFIG,
#elif (NUM_ENDPOINTS >= 11)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT12_CONFIG) && NUM_ENDPOINTS >= 12)
	ENDPOINT12_CONFIG,
#elif (NUM_ENDPOINTS >= 12)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT13_CONFIG) && NUM_ENDPOINTS >= 13)
	ENDPOINT13_CONFIG,
#elif (NUM_ENDPOINTS >= 13)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT14_CONFIG) && NUM_ENDPOINTS >= 14)
	ENDPOINT14_CONFIG,
#elif (NUM_ENDPOINTS >= 14)
	ENDPOINT_UNUSED,
#endif
#if (defined(ENDPOINT15_CONFIG) && NUM_ENDPOINTS >= 15)
	ENDPOINT15_CONFIG,
#elif (NUM_ENDPOINTS >= 15)
	ENDPOINT_UNUSED,
#endif
};


