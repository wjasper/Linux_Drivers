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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "E-TC32.h"

#include <fcntl.h>
#include <math.h>
#include <ctype.h>

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

int main(int argc, char**argv)
{
  struct version_t version;
  struct tm date_base, date_exp;
  struct networkDeviceInfo_t network;
  struct in_addr ip_addr;

  DeviceInfo_TC32 device_info;
  uint8_t dioIn[2];
  uint32_t dioOut;
  int i;
  float temperature, CJC_temp;
  uint8_t channel;
  uint8_t tc_type;
  int ch;
  uint8_t options;
  uint8_t buf[32];
  in_addr_t address;

 start:
  device_info.device.connectCode = 0x0;   // default connect code
  device_info.device.frameID = 0;         // zero out the frameID

  if (argc == 2) {
    printf("E-TC32 IP address = %s\n", argv[1]);
    device_info.device.Address.sin_family = AF_INET;
    device_info.device.Address.sin_port = htons(COMMAND_PORT);
    device_info.device.Address.sin_addr.s_addr = INADDR_ANY;
    if (inet_aton(argv[1], &device_info.device.Address.sin_addr) == 0) {
      printf("Improper destination address.\n");
      return -1;
    }
  } else if (discoverDevice(&device_info.device, ETC32_PID) <= 0) {
    printf("No device found.\n");
    return -1;
  }

  if ((device_info.device.sock = openDevice(inet_addr(inet_ntoa(device_info.device.Address.sin_addr)),
					    device_info.device.connectCode)) < 0) {
    printf("Error opening socket\n");
    return -1;
  }

  device_info.units = UNITS_CELSIUS;
  device_info.wait = 0x0;
  for (i = 0; i< 64; i++) {
    device_info.config_values[i] = 0x0;   // disable all channels
  }

  while(1) {
    printf("\nE-TC32 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'C' for A/D system calibration.\n");
    printf("Hit 'd' to test digitial IO.\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'j' for CJC compensation\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 'n' to get networking information\n");
    printf("Hit 's' to get Status\n");
    printf("hit 'R' to read System Memory Map\n");
    printf("hit 'W' to write System Memory Map\n");
    printf("Hit 't' for temperature.\n");
    printf("Hit 'T' for multiple temperature readings.\n");
    printf("Hit 'v' for version and calibration date.\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
	BlinkLED_E_TC32(&device_info, options);
        break;
      case 'C':
	printf("Performing A/D system offset calibration.\n");
	ADCal_E_TC32(&device_info);
	break;
      case 'd': /* test DIO */
	printf("\nTesting Digital I/O...\n");
	printf("connect pins DIO input [0-7] <--> DIO Output[0-7]\n");
	do {
	  printf("Enter a  number [0-0xff] : " );
	  scanf("%x", &dioOut);
	  DOutW_E_TC32(&device_info, BASE, dioOut);
	  DIn_E_TC32(&device_info, dioIn);
	  printf("The number you entered = %#x   Value read = %#x\n\n", dioOut, dioIn[0]);
	  for (i = 0; i < 8; i++) {
	    printf("Bit %d = %d\n", i, (dioIn[0]>>i) & 0x1);
	  }
        } while (toContinue());
        break;
      case 'j':
        for (channel = 0; channel < 32; channel++) {
	  CJC_E_TC32(&device_info, channel, &CJC_temp);
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
	  device_info.config_values[i] = CHAN_DISABLE;
	}
	device_info.config_values[channel] = tc_type;
	TinConfigW_E_TC32(&device_info);
	for (i = 0; i < 20; i++) {
	  Tin_E_TC32(&device_info, channel, UNITS_CELSIUS, 1, &temperature);
	  printf("Channel = %d   Temperature = %.2f C  %.2F\n", channel, temperature, temperature*9./5. + 32.);
	  sleep(1);
	}
        break;
      case 'r': 
	Reset_E_TC32(&device_info);
	sleep(2);
	goto start;
	break;
      case 's':
        Status_E_TC32(&device_info);
	if (device_info.status == 0) {
	  printf("No EXP detected.\n");
	} else {
	  printf("EXP detected.\n");
	}
	TinStatus_E_TC32(&device_info);
	printf("TinStatus = %#x\n", device_info.Tin_status[0]);
	OTDStatus_E_TC32(&device_info);
	printf("OTDStatus = %#x\n", device_info.OTD_status[0]);
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
	  device_info.config_values[i] = tc_type;
	}
	TinConfigW_E_TC32(&device_info);
	// Just read the even channels
	device_info.channel_mask[0] = 0x55555555;
	device_info.wait = 1;
	device_info.units = UNITS_CELSIUS;
	TinMultiple_E_TC32(&device_info);
	for (i = 0; i < 16; i++) {
	  printf("Channel[%d] Temperature = %f\n", 2*i, device_info.Tin_values[i]);
	}
	// Turn off the LED on the front pannel.
	device_info.config_measure[0] = 1;  //disable the OTD which turns off the LED.
	MeasureConfigW_E_TC32(&device_info);
	device_info.config_measure[0] = 0;  //enable the OTD;
	MeasureConfigW_E_TC32(&device_info);
        break;
      case 'v':
	Version_E_TC32(&device_info, &version);
	printf("Communications micro firmware version = %#x\n", version.version_comms);
	printf("Communcations mico bootloader firmware version = %#x\n", version.boot_version_comms);
	printf("Base measurement micro firmware version = %#x\n", version.version_base);
	printf("Base measurement micro bootloader firmware version = %#x\n", version.boot_version_base);
	printf("EXP measurement micro firmware version = %#x\n", version.version_exp);
	printf("EXP measurement micro bootloader firmware version = %#x\n\n", version.boot_version_exp);
	CalDateR_E_TC32(&device_info, &date_base, &date_exp);
	printf("Calibration date = %s\n", asctime(&date_base));
	break;
      case 'e':
	close(device_info.device.sock);
	printf("Success!\n");
        return(0);
        break;
      case 'n':
	NetworkConfig_E_TC32(&device_info, &network);
	ip_addr.s_addr = network.ip_address;
	printf("Device IP address:  %s\n", inet_ntoa(ip_addr));
	ip_addr.s_addr = network.subnet_mask;
	printf("Device subnet mask: %s\n", inet_ntoa(ip_addr));
	ip_addr.s_addr =  network.gateway_address;
	printf("Device gateway address: %s\n", inet_ntoa(ip_addr));
	break;
      case 'R':
	printf("Reading Settings Memory.\n");
	for (i = 0x0; i < 0x1f; i++) {
	  SettingsMemoryR_E_TC32(&device_info, i, 1, buf);
	  printf("address: %#x      value: %d\n", i, buf[0]);
	}
	break;
      case 'W':
	printf("Writing Settings Memory.\n");
	printf("Note: The could be dangerous if you add invalid data!!!\n");

	printf("Enter new default IP address (eg. 192.168.0.101): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_E_TC32(&device_info, 0x2, 4, (uint8_t*) &address);

	printf("Enter new default netmask (eg. 255.255.255.0): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_E_TC32(&device_info, 0x6, 4, (uint8_t*) &address);

	printf("Enter new default gateway (eg. 192.168.0.1): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_E_TC32(&device_info, 0xa, 4, (uint8_t*) &address);

	printf("Enter new Network options (0-3): ");
	scanf("%hhd", buf);
	SettingsMemoryW_E_TC32(&device_info, 0x0, 1, buf);

	printf("Reset or powercycle for changes to take effect.\n");
	
        break;

    }
  }
}
