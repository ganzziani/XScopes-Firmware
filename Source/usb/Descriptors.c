/*
             LUFA Library
     Copyright (C) Dean Camera, 2011.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2010  OBinou (obconseil [at] gmail [dot] com)
  Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions. */

#include "Descriptors.h"

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins. */
const USB_Descriptor_Device_t PROGMEM XmegaScope_DeviceDescriptor = {
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},
	.USBSpecification       = 0x0200,	// USB 2.0 (full speed): Windows asks for the MS OS descriptors only from 2.0 devices
	.Class                  = USB_CSCP_VendorSpecificClass,
	.SubClass               = USB_CSCP_NoDeviceSubclass,
	.Protocol               = USB_CSCP_NoDeviceProtocol,
	.Endpoint0Size          = USB_EP0SIZE,
	.VendorID               = 0x16D0,
	.ProductID              = 0x06F9,
	.ReleaseNumber          = 0x0101,	// 01.01, bumped so Windows re-reads the MS OS descriptors. Not VERSION_BCD(01.01):
	                                        // with 32-bit doubles its hundredths digit rounds down to 0x0100
	.ManufacturerStrIndex   = 0x01,
	.ProductStrIndex        = 0x02,
	.SerialNumStrIndex      = USE_INTERNAL_SERIAL,
	.NumberOfConfigurations = 1
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device. */
const USB_Descriptor_Configuration_t PROGMEM XmegaScope_ConfigurationDescriptor = {
	.Config = {
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},
			.TotalConfigurationSize = sizeof(USB_Descriptor_Configuration_t),
			.TotalInterfaces        = 1,
			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,
			.ConfigAttributes       = USB_CONFIG_ATTR_BUSPOWERED,
			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(500)
		},

	.XmegaScopeInterface = {
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
			.InterfaceNumber        = 0,
			.AlternateSetting       = 0,
			.TotalEndpoints         = 2,
			.Class                  = USB_CSCP_VendorSpecificClass,
			.SubClass               = 0x00,
			.Protocol               = 0x00,
			.InterfaceStrIndex      = NO_DESCRIPTOR
		},
	.DataInEndpoint = {
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			.EndpointAddress        = (ENDPOINT_DESCRIPTOR_DIR_IN | 1),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = 64,
			.PollingIntervalMS      = 0x00
		},
	.DataOutEndpoint = {
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			.EndpointAddress        = (ENDPOINT_DESCRIPTOR_DIR_OUT | 1),
			.Attributes             = (EP_TYPE_BULK | ENDPOINT_ATTR_NO_SYNC | ENDPOINT_USAGE_DATA),
			.EndpointSize           = 64,
			.PollingIntervalMS      = 0x00
		},
};

/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors. */
const USB_Descriptor_String_t PROGMEM XmegaScope_LanguageString = {
	.Header                 = {.Size = USB_STRING_LEN(1), .Type = DTYPE_String},
	.UnicodeString          = {LANGUAGE_ID_ENG}
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor. */
const USB_Descriptor_String_t PROGMEM XmegaScope_ManufacturerString = {
	.Header                 = {.Size = USB_STRING_LEN(11), .Type = DTYPE_String},
	.UnicodeString          = L"Gabotronics"
};

/** Product descriptor string. This is a Unicode string containing the product's details in human readable form,
 *  and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor. */
const USB_Descriptor_String_t PROGMEM XmegaScope_ProductString = {
	.Header                 = {.Size = USB_STRING_LEN(6), .Type = DTYPE_String},
	.UnicodeString          = L"XScope"
};


/** Microsoft OS 1.0 string descriptor (index 0xEE): "MSFT100" plus the vendor request code that
 *  Windows uses to fetch the Extended Compat ID below. Windows 8+ then binds WinUSB, no .inf needed. */
const uint8_t PROGMEM XmegaScope_MSOSString[18] = {
	18, DTYPE_String, 'M',0, 'S',0, 'F',0, 'T',0, '1',0, '0',0, '0',0, MS_VENDOR_CODE, 0
};

/** Microsoft OS 1.0 Extended Compat ID descriptor: interface 0 is compatible with "WINUSB". */
const uint8_t PROGMEM XmegaScope_MSCompatID[40] = {
	40, 0, 0, 0,		// dwLength
	0x00, 0x01,			// bcdVersion 1.00
	0x04, 0x00,			// wIndex: Extended Compat ID
	1,					// bCount: one function section
	0, 0, 0, 0, 0, 0, 0,
	0,					// bFirstInterfaceNumber
	1,					// Reserved, must be 1
	'W','I','N','U','S','B',0,0,	// compatibleID
	0, 0, 0, 0, 0, 0, 0, 0,			// subCompatibleID
	0, 0, 0, 0, 0, 0
};

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given to the
 *  USB library. When the device receives a Get Descriptor request on the control endpoint, this function is called
 *  so that the descriptor details can be passed back and the appropriate descriptor sent back to the USB host. */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress) {
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType) {
		case DTYPE_Device:
			Address = &XmegaScope_DeviceDescriptor;
			Size    = sizeof(USB_Descriptor_Device_t);
			break;
		case DTYPE_Configuration:
			Address = &XmegaScope_ConfigurationDescriptor;
			Size    = sizeof(USB_Descriptor_Configuration_t);
			break;
		case DTYPE_String:
			switch (DescriptorNumber) {
				case 0x00:
					Address = &XmegaScope_LanguageString;
					Size    = pgm_read_byte(&XmegaScope_LanguageString.Header.Size);
					break;
				case 0x01:
					Address = &XmegaScope_ManufacturerString;
					Size    = pgm_read_byte(&XmegaScope_ManufacturerString.Header.Size);
					break;
				case 0x02:
					Address = &XmegaScope_ProductString;
					Size    = pgm_read_byte(&XmegaScope_ProductString.Header.Size);
					break;
				case 0xEE:
					Address = &XmegaScope_MSOSString;
					Size    = sizeof(XmegaScope_MSOSString);
					break;
			}
			break;
	}
	*DescriptorAddress = Address;
	return Size;
}
