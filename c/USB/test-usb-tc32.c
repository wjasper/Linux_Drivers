/*
 *  Copyright (c) 2016 Warren J. Jasper <wjasper@ncsu.edu>
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
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "pmd.h"
#include "usb-tc-32.h"

/* Test Program */
int toContinue()
{
  int answer;
  answer = 0; //answer = getchar();
  printf("Continue [yY]? ");
  while((answer = getchar()) == '\0' ||
    answer == '\n');
  return ( answer == 'y' || answer == 'Y');
}

int main (int argc, char **argv)
{
  libusb_device_handle *udev = NULL;
  tc32_channel cdev;
  struct version_t version;
  struct networkDeviceInfo_t device;
  struct in_addr ip_addr;
  float temperature, CJC_temp;
  uint8_t channel;
  uint8_t tc_type;
  int ret;
  int ch, i;
  int temp;
  uint8_t dioIn[2];
  uint32_t dioOut;
  struct tm date_base, date_exp;
 
 start:
  udev = NULL;
  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB_TC32_PID, NULL))) {
      printf("Success, found a USB-TC-32!\n");
  } else {
    printf("Failure, did not find a USB-TC-32!\n");
    return 0;
  }

  cdev.units = CELSIUS;
  cdev.wait = 0x0;
  for (i = 0; i < 64; i++) {
    cdev.config_values[i] = 0x0;  // disable all channels
  }

  //print out the wMaxPacketSize.  Should be 512
  printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

  while(1) {
    printf("\nUSB TC-32 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink.\n");
    printf("Hit 'C' for A/D system calibration.\n");
    printf("Hit 'd' for DIO.\n");
    printf("Hit 'e' to exit.\n");
    printf("Hit 'j' for CJC compensation\n");
    printf("Hit 'n' for network configuration.\n");    
    printf("Hit 'r' to reset.\n");
    printf("Hit 's' for status.\n");
    printf("Hit 't' for temperature.\n");
    printf("Hit 'T' for multiple temperature readings.\n");
    printf("Hit 'v' for version and calibration date.\n");

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
	scanf("%d", &temp);
        usbBlink_TC32(udev, temp);
        break;
      case 'd': /* test DIO */
	printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO input [0-7] <--> DIO Output[0-7]\n");
	do {
	  printf("Enter a  number [0-0xff] : " );
	  scanf("%x", &dioOut);
	  usbDOut_TC32(udev, BASE, dioOut);
	  usbDIn_TC32(udev, dioIn);
	  printf("The number you entered = %#x   Value read = %#x\n\n", dioOut, dioIn[0]);
	  for (i = 0; i < 8; i++) {
	    printf("Bit %d = %d\n", i, (dioIn[0]>>i) & 0x1);
	  }
        } while (toContinue());
        break;
      case 'r': 
	usbReset_TC32(udev);
	sleep(2);
	goto start;
	break;
      case 's':
        usbStatus_TC32(udev, &cdev);
	if (cdev.status == 0) {
	  printf("No EXP detected.\n");
	} else {
	  printf("EXP detected.\n");
	}
	usbTinStatus_TC32(udev, &cdev);
	printf("TinStatus = %#x\n", cdev.Tin_status[0]);
	usbOTDStatus_TC32(udev, &cdev);
	printf("OTDStatus = %#x\n", cdev.OTD_status[0]);
	break;
      case 'j':
        for (channel = 0; channel < 32; channel++) {
	  CJC_temp = usbCJC_TC32(udev, channel);
	  printf("Channel = %d   Temperature = %.2f C  %.2f F\n", channel, CJC_temp, CJC_temp*9./5. + 32.);
	}
        break;
      case 't':
        printf("Enter channel number [0-31]: ");
	scanf("%hhd", &channel);
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': 
  	  case 'j':
	    tc_type = TC_TYPE_J; break;
  	  case 'K': 
  	  case 'k': 
	    tc_type = TC_TYPE_K; break;
	  case 'R':
	  case 'r':
	    tc_type = TC_TYPE_R; break;
  	  case 'S':
	  case 's':
	    tc_type = TC_TYPE_S; break;
   	  case 'T':
	  case 't':
	    tc_type = TC_TYPE_T; break;
     	  case 'N':
	  case 'n':
	    tc_type = TC_TYPE_N; break;
       	  case 'E':
  	  case 'e':
	    tc_type = TC_TYPE_E; break;
	  case 'B':
 	  case 'b':
	    tc_type = TC_TYPE_B; break;
	  default: tc_type = TC_TYPE_J; break;
	}
	for (i = 0; i < 64; i++) {
	  cdev.config_values[channel] = CHAN_DISABLE;
	}
	cdev.config_values[channel] = tc_type;
	usbTinConfig_TC32(udev, &cdev);
	for (i = 0; i < 20; i++) {
	  temperature = usbTin_TC32(udev, channel, CELSIUS, 1);
	  printf("Channel = %d   Temperature = %.2f C  %.2F\n", channel, temperature, temperature*9./5. + 32.);
	  sleep(1);
	}
        break;
      case 'T':
	printf("Read Multiple Thermocouple channels\n");
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': 
  	  case 'j':
	    tc_type = TC_TYPE_J; break;
  	  case 'K': 
  	  case 'k': 
	    tc_type = TC_TYPE_K; break;
	  case 'R':
	  case 'r':
	    tc_type = TC_TYPE_R; break;
  	  case 'S':
	  case 's':
	    tc_type = TC_TYPE_S; break;
   	  case 'T':
	  case 't':
	    tc_type = TC_TYPE_T; break;
     	  case 'N':
	  case 'n':
	    tc_type = TC_TYPE_N; break;
       	  case 'E':
  	  case 'e':
	    tc_type = TC_TYPE_E; break;
	  case 'B':
 	  case 'b':
	    tc_type = TC_TYPE_B; break;
	  default: tc_type = TC_TYPE_J; break;
	}
	for (i = 0; i < 31; i++) {
	  cdev.config_values[i] = tc_type;
	}
	usbTinConfig_TC32(udev, &cdev);
	// Just read the even channels
	cdev.all_channels = 0;  // use the channel mask
	cdev.channel_mask[0] = 0x55555555;
	cdev.wait = 1;
	cdev.units = CELSIUS;
	usbTinMultiple_TC32(udev, &cdev);
	for (i = 0; i < 16; i++) {
	  printf("Channel[%d] Temperature = %f\n", 2*i, cdev.Tin_values[i]);
	}
        break;
      case 'v':
	usbVersion_TC32(udev, &version);
	printf("Communications micro firmware version = %#x\n", version.version_comms);
	printf("Communcations mico bootloader firmware version = %#x\n", version.boot_version_comms);
	printf("Base measurement micro firmware version = %#x\n", version.version_base);
	printf("Base measurement micro bootloader firmware version = %#x\n", version.boot_version_base);
	printf("EXP measurement micro firmware version = %#x\n", version.version_exp);
	printf("EXP measurement micro bootloader firmware version = %#x\n\n", version.boot_version_exp);
	usbCalDate_TC32(udev, &date_base, &date_exp);
	printf("Calibration date = %s\n", asctime(&date_base));
	break;
      case 'C':
	printf("Performing A/D system offset calibration.\n");
	usbADCal_TC32(udev);
	break;
      case 'n':
	usbNetworkConfig_TC32(udev, &device);
	ip_addr.s_addr = device.ip_address;
	printf("Device IP address:  %s\n", inet_ntoa(ip_addr));
	ip_addr.s_addr = device.subnet_mask;
	printf("Device subnet mask: %s\n", inet_ntoa(ip_addr));
	ip_addr.s_addr =  device.gateway_address;
	printf("Device gateway address: %s\n", inet_ntoa(ip_addr));
	break;
      case 'e':
	exit(0);
	break;
      default:
        break;
    }
  }
}
