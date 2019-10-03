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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "E-TC.h"

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
  struct tm date;
  struct networkDeviceInfo_t network;
  struct in_addr ip_addr;

  DeviceInfo_TC device_info;
  uint8_t dioIn;
  uint8_t dioOut;
  int i;
  int ch;
  int flag;
  float temperature[8];
  int nChannels;       // number of channels
  uint8_t channel;
  uint8_t tc_type;
  uint8_t options;
  uint8_t buf[32];
  in_addr_t address;

  #define MAX_DEVICES 100
  EthernetDeviceInfo **devices;
  int nDevices = 0;

 start:
  device_info.device.connectCode = 0x0;   // default connect code
  device_info.device.frameID = 0;         // zero out the frameID

  if (argc == 2) {
    printf("E-TC IP address = %s\n", argv[1]);
    device_info.device.Address.sin_family = AF_INET;
    device_info.device.Address.sin_port = htons(COMMAND_PORT);
    device_info.device.Address.sin_addr.s_addr = INADDR_ANY;
    if (inet_aton(argv[1], &device_info.device.Address.sin_addr) == 0) {
      printf("Improper destination address.\n");
      return -1;
    }
  } else if (discoverDevice(&device_info.device, ETC_PID) <= 0) {
    printf("No device found.\n");
    return -1;
  }

  /* if you have more than one device, this is one way to manage them */
  // Build up a structure of devices
  devices = malloc(MAX_DEVICES*sizeof(devices));
  for (i = 0; i < MAX_DEVICES; i++) {
    devices[i] = malloc(sizeof(EthernetDeviceInfo));
  }
  nDevices = discoverDevices(devices, ETC_PID, MAX_DEVICES);

  if (nDevices <= 0) {
    printf("No device found.\n");
    return -1;
  } else {
    printf("%d E-TC devices found!\n", nDevices);
  }

  /* Do check here, manage MAC address, etc. */
  for (i = 0; i < nDevices; i++) {
    printDeviceInfo(devices[i]);
  }

  // cleanup
  for (i = 0; i < MAX_DEVICES; i++) {
    free(devices[i]);
  }
  free(devices);

  device_info.device.connectCode = 0x0;   // default connect code
  device_info.device.frameID = 0;         // zero out the frameID

  if ((device_info.device.sock = openDevice(inet_addr(inet_ntoa(device_info.device.Address.sin_addr)),
					    device_info.device.connectCode)) < 0) {
    printf("Error opening socket\n");
    return -1;
  }

  device_info.units = UNITS_CELSIUS;
  device_info.wait = 0x0;
  for (i = 0; i< 8; i++) {
    device_info.config_values[i] = 0x0;   // disable all channels
  }

  while(1) {
    printf("\nE-TC Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink.\n");
    printf("Hit 'c' to test counter.\n");
    printf("Hit 'C' for A/D system calibration.\n");
    printf("Hit 'd' to test digitial IO.\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'j' for CJC compensation.\n");
    printf("Hit 'r' to reset the device.\n");
    printf("Hit 'n' to get networking information.\n");
    printf("Hit 's' for thermocouple status\n");
    printf("Hit 'R' to read System Memory Map\n");
    printf("Hit 'W' to write System Memory Map\n");
    printf("Hit 't' for temperature.\n");
    printf("Hit 'v' for version and calibration date.\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
	BlinkLED_E_TC(&device_info, options);
        break;
      case 'c':
	printf("connect DIO0 to CTR\n");
	DConfigW_E_TC(&device_info, 0xf0);   // set pins 0-3 output
	ResetCounter_E_TC(&device_info);    // reset the counter
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  DOutW_E_TC(&device_info, 0x1);
	  usleep(50000);
	  DOutW_E_TC(&device_info, 0x0);
	  usleep(50000);
	  CounterR_E_TC(&device_info);
	  printf("Counter = %d\n", device_info.counter);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'C':
	printf("Performing A/D system offset calibration.\n");
	ADCal_E_TC(&device_info);
	break;
      case 'd': /* test DIO */
	printf("\nTesting Digital I/O...\n");
	printf("connect DIO pins  [0-3] <--> [4-7]\n");
	do {
	  printf("Enter a  number [0-0xf] : " );
	  scanf("%hhx", &dioOut);
	  DConfigW_E_TC(&device_info, 0xf0); 
	  DOutW_E_TC(&device_info, dioOut);
	  DIn_E_TC(&device_info, &dioIn);
	  dioIn >>= 0x4;
	  printf("The number you entered = %#x   Value read = %#x\n\n", dioOut, dioIn);
	  for (i = 0; i < 8; i++) {
	    printf("Bit %d = %d\n", i, (dioIn >> i) & 0x1);
	  }
        } while (toContinue());
        break;
      case 'j':
      	 CJC_E_TC(&device_info);
	 printf("Sensor 1: CJC Temperature = %.2f C  %.2f F\n", device_info.CJC_value[0], device_info.CJC_value[0]*9./5.+32.);
 	 printf("Sensor 2: CJC Temperature = %.2f C  %.2f F\n", device_info.CJC_value[1], device_info.CJC_value[1]*9./5.+32.);
        break;
      case 's':
	TinStatus_E_TC(&device_info);
	printf("Tin status byte = %#x\n", device_info.Tin_status);
	OTDStatus_E_TC(&device_info);
	printf("Open Thermocouple Detection status byte = %#x\n", device_info.OTD_status);
	AlarmStatusR_E_TC(&device_info);
	printf("Alarm Status byte = %#x.\n", device_info.alarm_status);
	MeasureConfigR_E_TC(&device_info);
	printf("Measurement Configuration byte = %#x\n", device_info.config_measure);
	MeasureModeR_E_TC(&device_info);
	printf("Measurement Mode byte = %#x\n", device_info.mode_measure);
	CJCOffsetR_E_TC(&device_info);
	for (i = 0; i < 8; i++) {
	  printf("CJC offsets: channel[%d] = %f\n", i, device_info.CJC_offsets[i]);
        }
	AlarmConfigR_E_TC(&device_info);
	printf("\n");
	for (i = 0; i < 8; i++) {
	  printf("Channel %d:  alarm config = %d   threshold1 = %f  threshold2 = %f\n",
		 i, device_info.alarm_config[i], device_info.alarm_threshold1[i], device_info.alarm_threshold2[i]);
	}
	break;
      case 't':
        printf("Enter number of channels  [1-8]: ");
	scanf("%d", &nChannels);
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
	for (i = 0; i < 8; i++) {
	  device_info.config_values[i] = CHAN_DISABLE;
	}
	channel = 0x0;
	for (i = 0; i < nChannels; i++) {
  	  device_info.config_values[i] = tc_type;
	  channel |= 0x1 << i; // set bit to corresponding channel
	}
	TinConfigW_E_TC(&device_info);
        Tin_E_TC(&device_info, channel, UNITS_CELSIUS, 1, temperature);
	for (i = 0; i < nChannels; i++) {
	  printf("Channel = %d   Temperature = %.2f C  %.2F\n", i, temperature[i], temperature[i]*9./5. + 32.);
	}
        break;
      case 'r': 
	Reset_E_TC(&device_info);
	sleep(2);
	goto start;
	break;
      case 'e':
	close(device_info.device.sock);
	printf("Success!\n");
        return(0);
        break;
      case 'n':
	NetworkConfig_E_TC(&device_info, &network);
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
	  SettingsMemoryR_E_TC(&device_info, i, 1, buf);
	  printf("address: %#x      value: %d\n", i, buf[0]);
	}
	break;
      case 'W':
	printf("Writing Settings Memory.\n");
	printf("Note: The could be dangerous if you add invalid data!!!\n");

	printf("Enter new default IP address (eg. 192.168.0.101): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_E_TC(&device_info, 0x2, 4, (uint8_t*) &address);

	printf("Enter new default netmask (eg. 255.255.255.0): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_E_TC(&device_info, 0x6, 4, (uint8_t*) &address);

	printf("Enter new default gateway (eg. 192.168.0.1): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_E_TC(&device_info, 0xa, 4, (uint8_t*) &address);

	printf("Enter new Network options (0-3): ");
	scanf("%hhd", buf);
	SettingsMemoryW_E_TC(&device_info, 0x0, 1, buf);
	printf("Reset or powercycle for changes to take effect.\n");

        break;
      case 'v':
	FactoryCalDateR_E_TC(&device_info, &date);
	printf("Factory Calibration date = %s", asctime(&date));
	FieldCalDateR_E_TC(&device_info, &date);
	printf("Field Calibration date = %s\n", asctime(&date));
	break;
    }
  }
}
