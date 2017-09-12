/*
 *
 *  Copyright (c) 2014 Warren J. Jasper <wjasper@ncsu.edu>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include "pmd.h"

#define EP_INTR (1 | LIBUSB_ENDPOINT_IN)
#define EP_DATA (2 | LIBUSB_ENDPOINT_IN)
    
int usb_get_max_packet_size(libusb_device_handle* udev, int endpointNum) 
{
  struct libusb_device *device;
  struct libusb_device_descriptor desc;
  struct libusb_config_descriptor *config;
  const struct libusb_interface *interface;
  const struct libusb_interface_descriptor *altsetting;
  const struct libusb_endpoint_descriptor *endpoint;
  int packet_size;
  int ret;

  device = libusb_get_device(udev);
  ret = libusb_get_active_config_descriptor(device, &config);
  if (ret < 0) {
    perror("usb_get_max_packet_size: error in libusb_get_active_config_descriptor");
    return ret;
  }
  interface = &config->interface[0];
  altsetting = &interface->altsetting[0];
  endpoint = &altsetting->endpoint[endpointNum];
  if (endpoint == NULL) {
   ret = libusb_get_device_descriptor(device, &desc);
   if (ret < 0) {
     perror("usb_get_max_packet_size: error in libusb_get_device_descriptor");
     return ret;
   }
   packet_size = desc.bMaxPacketSize0;
  } else {
    packet_size = endpoint->wMaxPacketSize;
  }
  libusb_free_config_descriptor(config);
  if (ret < 0) {
    perror("usb_get_max_packet_size: error in libusb_free_condfig_descriptor");
    return ret;
  }
  
  return packet_size;
}

libusb_device_handle* usb_device_find_USB_MCC( int productId, char *serialID )
{
  int vendorId = MCC_VID;

  struct libusb_device_handle *udev = NULL;
  struct libusb_device_descriptor desc;
  struct libusb_device **list;
  struct libusb_device *found = NULL;
  struct libusb_device *device;
  char serial[9];

  ssize_t cnt = 0;
  ssize_t i = 0;
  int err = 0;
  int cfg;
  int config;

  // discover devices
  cnt = libusb_get_device_list(NULL, &list);
  if (cnt < 0) {
    perror("usb_device_find_USB_MCC: No USB devices found on bus.");
    return (void *) cnt;
  }

  for (i = 0; i < cnt; i++) {
    device = list[i];
    err = libusb_get_device_descriptor(device, &desc);
    if (err < 0) {
      perror("usb_device_find_USB_MCC: Can not get USB device descriptor");
      goto out;
    }
    if (desc.idVendor == vendorId && desc.idProduct == productId) {
      found = device;
      err = libusb_open(found, &udev);
      if (err < 0) {
	perror("usb_device_find_USB_MCC: libusb_open failed.");
	udev = NULL;
	continue;
      }
      err = libusb_kernel_driver_active(udev, 0);
      if (err == 1) {
	/* 
	   device active by another driver. (like HID).  This can be dangerous,
           as we don't know if the kenel has claimed the interface or another
           process.  No easy way to tell at this moment, but HID devices won't
           work otherwise.
	 */
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000103)
	libusb_set_auto_detach_kernel_driver(udev, 1);
#else
	libusb_detach_kernel_driver(udev, 0);
#endif
      }
      err = libusb_claim_interface(udev, 0);
      if (err < 0) {
        // perror("error claiming interface 0");
	libusb_close(udev);
	udev = NULL;
	continue;
      }
      /* Check to see if serial ID match */
      if (serialID != NULL) {
	err = libusb_get_string_descriptor_ascii(udev, desc.iSerialNumber, (unsigned char *) serial, sizeof(serial));
	if (err < 0) {
	  perror("usb_device_find_USB_MCC: Error reading serial number for device.");
	  libusb_release_interface(udev, 0);
	  libusb_close(udev);
	  udev = NULL;
	  continue;
	}
        if (strcmp(serialID, serial) == 0) {
	  break;
	} else {
	  libusb_release_interface(udev, 0);
	  libusb_close(udev);
	  udev = NULL;
	  continue;
	}
      } else {
        
        /* If we got to here, we found a match and were able to claim the interface.  At
  	  this point we should stop looking and break out;
        */
	break;
      }
    }
  }
  libusb_free_device_list(list,1);

  if (udev) {
    cfg = libusb_get_configuration(udev, &config);
    if (cfg != 0) {
      err = libusb_set_configuration(udev, 1);
      if (err < 0) {
	perror("usb_device_find_USB_MCC: error in setting configuration.");
      }
    }
  }
  return udev;

out:
  libusb_free_device_list(list,1);
  libusb_close(udev);
  libusb_exit(NULL);
  return (void *) -1;
}

int getUsbSerialNumber(libusb_device_handle *udev, unsigned char serial[])
{
  struct libusb_device_descriptor desc;
  struct libusb_device *device;
  int ret;

  device = libusb_get_device(udev);
  ret = libusb_get_device_descriptor(device, &desc);
  if (ret < 0) return ret;
  ret = libusb_get_string_descriptor_ascii(udev, desc.iSerialNumber, serial, 8);
  if (ret < 0) return ret;
  serial[8] = '\0';
  return 0;
}

#define HS_DELAY 20

int sendStringRequest(libusb_device_handle *udev, char *message)
{
  uint8_t requesttype = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
  unsigned char string[MAX_MESSAGE_LENGTH];
  int ret;
  
  strncpy((char *)string, message, MAX_MESSAGE_LENGTH);
  string[MAX_MESSAGE_LENGTH - 1] = '\0';
  //  printf("SendStringRequest: string = %s\n", string);

  ret = libusb_control_transfer(udev, requesttype, STRING_MESSAGE, 0, 0, (unsigned char *) string, MAX_MESSAGE_LENGTH, HS_DELAY);
  return ret;
}

int  getStringReturn(libusb_device_handle *udev, char *message)
{
  /* Return 64 byte message */
  uint8_t requesttype = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
  int ret;

  ret = libusb_control_transfer(udev, requesttype, STRING_MESSAGE, 0, 0, (unsigned char *)message, MAX_MESSAGE_LENGTH, HS_DELAY);
  //  printf("getStringReturn: string = %s\n", message);
  return ret;
}

/********************** HID wrapper functions ******************/
int PMD_SendOutputReport(hid_device* hid, uint8_t* values, size_t length)
{
  int ret;
  
  ret = hid_write(hid, values, length);
  if (ret < 0) {
    printf("PMD_SendOutputReport.  Unable to write data %ls \n", hid_error(hid));
  }
  return ret;
}

int PMD_GetInputReport(hid_device* hid, uint8_t *values, size_t length, int delay)
{
  int res;
  
  //  err = hid_read_timeout(hid, values, length, delay);
  res = hid_read_timeout(hid, values, length, delay);
  if (res < 0) {
    printf("PMD_GetInputReport.  Unable to read data %ls \n", hid_error(hid));
    return res;
  }
  return res;
}

int PMD_GetFeatureReport(hid_device* hid, uint8_t *data, int length)
{
  hid_get_feature_report(hid, data, length);
  return length;
}
