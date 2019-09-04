/*
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@ncsu.edu>
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
#include "E-DIO24.h"

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
  struct in_addr network[3];
  EthernetDeviceInfo device_info;
  int i;
  uint16_t status;
  uint8_t options;
  uint32_t input;
  uint32_t value, mask;
  uint32_t counter;
  int ch;
  int flag;
  uint8_t buf[32];
  in_addr_t address;

  device_info.connectCode = 0x0;   // default connect code
  device_info.frameID = 0;         // zero out the frameID

  if (argc == 2) {
    printf("E-DIO24 IP address = %s\n", argv[1]);
    device_info.Address.sin_family = AF_INET;
    device_info.Address.sin_port = htons(COMMAND_PORT);
    device_info.Address.sin_addr.s_addr = INADDR_ANY;
    if (inet_aton(argv[1], &device_info.Address.sin_addr) == 0) {
      printf("Improper destination address.\n");
      return -1;
    }
  } else if (discoverDevice(&device_info, EDIO24_PID) <= 0) {
    printf("No device found.\n");
    return -1;
  }

  if ((device_info.sock = openDevice(inet_addr(inet_ntoa(device_info.Address.sin_addr)),
					    device_info.connectCode)) < 0) {
    printf("Error opening socket\n");
    return -1;
  }

  while(1) {
    printf("\nE-DIO24 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter.\n");
    printf("Hit 'd' to test digitial IO.\n");
    printf("Hit 'C' to configure the DIO for input or output\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 'n' to get networking information\n");
    printf("hit 'R' to read System Memory Map\n");
    printf("hit 'W' to write System Memory Map\n");
    printf("Hit 's' to get Status\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
	BlinkLED_DIO24(&device_info, options);
        break;
      case 'c':
	printf("Testing Counter on E-DIO24\n");
	printf("connect P2D6 to P2D7\n");
	mask = 0xc00000;
	DConfigW_DIO24(&device_info, mask, 0x800000);  // set pins P2D6 output, counter pin P2D7 input
	CounterW_DIO24(&device_info);                  // reset the counter
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
	  DOut_DIO24(&device_info, mask, 0x400000); // set P2D6 HI
	  usleep(5000);
	  DOut_DIO24(&device_info, mask, 0x000000); // set P2D6 LOW
	  usleep(500000);
	  CounterR_DIO24(&device_info, &counter);
	  printf("Counter = %d\n", counter);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'C':
	printf("Enter mask (which bits to configure): ");
	scanf("%x", &mask);
	printf("Enter the configuration value: ");
	scanf("%x", &value);
	if (DConfigW_DIO24(&device_info, mask, value) == false) {
	  printf("error in DConfigW_DIO24\n");
	}
	break;
      case 'd':
	printf("\nTesting Digital I/O...\n");
	printf("connect pins P0[0-3] <--> P0[4-7] and P1[0-3] <--> P1[4-7\n");
        mask = 0xffff;
        value = 0x00f0f0;
	if (DConfigW_DIO24(&device_info, mask, value) == false) {
	  printf("error in DConfigW_DIO24\n");
	}
	if (DConfigR_DIO24(&device_info, &input) == false) {
	  printf("error in DConfigR_DIO24\n");
	} else {
	  printf("Digital Port Tristate Register = %#x\n", input);
	}
	do {
	  printf("Enter a number [0-0xff] : ");
	  scanf("%x", &value);
	  value = (value & 0xf) | ((value << 4) & 0xf00);
	  DOut_DIO24(&device_info, 0xf0f,  value);
	  DOutR_DIO24(&device_info, (uint32_t *) &value);
    	  value = (value & 0xf) | ((value >> 4) & 0xf0);
	  DIn_DIO24(&device_info, &input);
	  input = (input & 0xf) | ((input >> 4) & 0xf0);
          printf("The number you entered = %#x   Latched value = %#x\n\n",input, value);
	  for (i = 0; i < 8; i++) {
	    printf("Bit %d = %d\n", i, (value>>i)&0x1);
	  }
        } while (toContinue());
        break;
      case 'e':
	close(device_info.sock);
	printf("Success!\n");
	return 0;
	break;
      case 'r':
	Reset_DIO24(&device_info);
	break;
      case 's':
	// print out the status
	if(Status_DIO24(&device_info, &status)) {
	  printf("status = %#x\n", status);
	} else {
	  printf("Error in reading status\n");
	}
	break;
      case 'n':
        // get network configuration parameters
	NetworkConfig_DIO24(&device_info, network);
	printf("Network configuration values: \n");
	printf("  IP address = %s\n", inet_ntoa(network[0]));
	printf("  subnet mask = %s\n", inet_ntoa(network[1]));
	printf("  gateway = %s\n", inet_ntoa(network[2]));
	break;
      case 'R':
	printf("Reading Settings Memory.\n");
	for (i = 0x0; i < 0x1f; i++) {
	  SettingsMemoryR_DIO24(&device_info, i, 1, buf);
	  printf("address: %#x      value: %d\n", i, buf[0]);
	}
	break;
      case 'W':
	printf("Writing Settings Memory.\n");
	printf("Note: The could be dangerous if you add invalid data!!!\n");

	printf("Enter new default IP address (eg. 192.168.0.101): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_DIO24(&device_info, 0x2, 4, (uint8_t*) &address);

	printf("Enter new default netmask (eg. 255.255.255.0): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_DIO24(&device_info, 0x6, 4, (uint8_t*) &address);

	printf("Enter new default gateway (eg. 192.168.0.1): ");
	scanf("%s", buf);
	address = inet_addr((char *) buf);
	SettingsMemoryW_DIO24(&device_info, 0xa, 4, (uint8_t*) &address);

	printf("Enter new Network options (0-3): ");
	scanf("%hhd", buf);
	SettingsMemoryW_DIO24(&device_info, 0x0, 1, buf);

	printf("See E-DIO24.h for other values in Systems Memory Map\n");
	printf("Reset or powercycle for changes to take effect.\n");
	
        break;
    }
  }
}

