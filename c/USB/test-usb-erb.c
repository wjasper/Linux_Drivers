/*
 *
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-erb.h"

#define MAX_STR 255

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

int main (int argc, char **argv) {
  
  int ch;
  int temp;
  int device = 0;  // either USBERB24 or USBERB08
  uint16_t status;
  uint8_t input; 
  uint8_t port;
  uint8_t pin = 0;
  uint8_t bit_value;

  hid_device*  hid = 0x0;
  int ret;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USBERB24_PID, NULL)) > 0) {
    printf("USB ERB24 Device is found!\n");
    device = USBERB24_PID;
  } else if ((hid = hid_open(MCC_VID, USBERB08_PID, NULL)) > 0) {
    printf("USB ERB08 Device is found!\n");
    device = USBERB08_PID;
    printf("device = %d\n", device);
  } else {
    fprintf(stderr, "USB ERB24 or ERB08 not found.\n");
    exit(-1);
  }

  while(1) {
    printf("\nUSB ERB24 and ERB08 Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'd' to test digital I/O \n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'j' for information\n");
    printf("Hit 's' to get status\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 't' to test digital bit I/O\n");
    printf("Hit 'l' to get temperature\n");
    
    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
    case 'b': /* test to see if led blinks */
      usbBlink_USBERB(hid);
      break;
    case 'd':
      printf("\nTesting Digital I/O \n");
      do {
	if (device == USBERB24_PID) {
	  printf("Enter a port number [0-3]: ");
	  scanf("%hhd", &port);
	  if (port > 3) break;
	  printf("Enter a byte number [0-0xff] : " );
          scanf("%x", &temp);
          usbDOut_USBERB(hid, port, (uint8_t)temp);
	} else {
	  printf("Enter a port number [2-3]: ");  // only CL and CH on ERB08
	  scanf("%hhd", &port);
	  printf("port = %d\n", port);
	  if (port != 2 &&  port != 3) continue;
          printf("Enter a nibble number [0-0xf] : " );
          scanf("%x", &temp);
	  usbDOut_USBERB(hid, port, (uint8_t)temp);
	}
        status = usbGetStatus_USBERB(hid);
	input = usbDIn_USBERB(hid, port);
        switch (port) {
          case 0:   /* Port A */
	    printf("    Port A = %#x\n", input);
            if (status & 0x1<<0) {
              printf("    Port A polarity = normal\n");
            } else {
              printf("    Port A polarity = inverted\n");
            }
            if (status & 0x1<<4) {
              printf("    Port A  = pull up\n");
            } else {
              printf("    Port A  = pull down\n");
            }
            break;
          case 1:   /* Port B */
	    printf("    Port B = %#x\n", input);
            if (status & 0x1<<1) {
              printf("    Port B polarity = normal\n");
            } else {
              printf("    Port B polarity = inverted\n");
            }
            if (status & 0x1<<5) {
              printf("    Port B  = pull up\n");
            } else {
              printf("    Port B  = pull down\n");
            }
            break;
          case 2:   /* Port C Low */
	    printf("    Port C Low = %#x\n", input);
            if (status & 0x1<<2) {
              printf("    Port C Low polarity = normal\n");
            } else {
              printf("    Port C Low polarity = inverted\n");
            }
            if (status & 0x1<<6) {
              printf("    Port C Low  = pull up\n");
            } else {
              printf("    Port C Low  = pull down\n");
            }
            break;
          case 3:   /* Port C High */
	    printf("    Port C High = %#x\n", input);
            if (status & 0x1<<3) {
              printf("    Port C High polarity = normal\n");
            } else {
              printf("    Port C High polarity = inverted\n");
            }
            if (status & 0x1<<7) {
              printf("    Port C High  = pull up\n");
            } else {
              printf("    Port C High  = pull down\n");
            }
            break;
        }
      } while (toContinue());
      break;
    case 'g':
      hid_get_serial_number_string(hid, serial, 64);
      printf("Serial Number = %ls\n", serial);
      break;
    case 'j':
      // Read the Manufacuter String
      ret = hid_get_manufacturer_string(hid, wstr, MAX_STR);
      printf("Manufacturer String: %ls\n", wstr);
      // Read the Product String
      ret = hid_get_product_string(hid, wstr, MAX_STR);
      printf("Product String: %ls\n", wstr);
      // Read the Serial Number String
      ret = hid_get_serial_number_string(hid, wstr, MAX_STR);
      printf("Serial Number String: %ls\n", wstr);
      break;            
    case 't':
      printf("\nTesting Digital Bit I/O....\n");
      do {
	if (device == USBERB24_PID) {
	  printf("Enter a port number [0-3]: ");
	  scanf("%hhd", &port);
	  if (port > 3) break;
	} else {
	  printf("Enter a port number [2-3]: ");  // only CL and CH on ERB08
	  scanf("%hhd", &port);
	  printf("port = %d\n", port);
	  if (port != 2 &&  port != 3) continue;
	}
        printf("Select the Pin in port  %d  [0-7] :", port);
        scanf("%hhd", &pin);
	printf("Enter a bit value for output (0 | 1) : ");
	scanf("%hhd", &bit_value);
	usbDBitOut_USBERB(hid, port, pin, bit_value);
        status = usbGetStatus_USBERB(hid);
	input = usbDBitIn_USBERB(hid, port, pin);
	printf("    Port %d  Pin %d = %#x\n", port, pin, input);
        switch (port) {
          case 0:   /* Port A */
            if (status & 0x1<<0) {
              printf("    Port A polarity = normal\n");
            } else {
              printf("    Port A polarity = inverted\n");
            }
            if (status & 0x1<<4) {
              printf("    Port A  = pull up\n");
            } else {
              printf("    Port A  = pull down\n");
            }
            break;
          case 1:   /* Port B */
            if (status & 0x1<<1) {
              printf("    Port B polarity = normal\n");
            } else {
              printf("    Port B polarity = inverted\n");
            }
            if (status & 0x1<<1) {
              printf("    Port B  = pull up\n");
            } else {
              printf("    Port B  = pull down\n");
            }
            break;
          case 2:   /* Port C Low */
            if (status & 0x1<<2) {
              printf("    Port C Low polarity = normal\n");
            } else {
              printf("    Port C Low polarity = inverted\n");
            }
            if (status & 0x1<<6) {
              printf("    Port C Low  = pull up\n");
            } else {
              printf("    Port C Low  = pull down\n");
            }
            break;
          case 3:   /* Port C High */
            if (status & 0x1<<3) {
              printf("    Port C High polarity = normal\n");
            } else {
              printf("    Port C High polarity = inverted\n");
            }
            if (status & 0x1<<7) {
              printf("    Port C High  = pull up\n");
            } else {
              printf("    Port C High  = pull down\n");
            }
            break;
        }
      } while (toContinue());
      break;
    case 's':
      status = usbGetStatus_USBERB(hid);      
      printf("Status = %#x\n", status);
      if (device == USBERB24_PID) {
	if (status & 0x1<<1) {
	  printf("    Port A polarity = normal\n");
	} else {
	  printf("    Port A polarity = inverted\n");
	}
	if (status & 0x1<<4) {
	  printf("    Port A  = pull up\n");
	} else {
	  printf("    Port A  = pull down\n");
	}
	/* Port B */
	if (status & 0x1<<2) {
	  printf("    Port B polarity = normal\n");
	} else {
	  printf("    Port B polarity = inverted\n");
	}
	if (status & 0x1<<4) {
	  printf("    Port B  = pull up\n");
	} else {
	  printf("    Port B  = pull down\n");
	}
      }
      /* Port C Low */
      if (status & 0x1<<3) {
	printf("    Port C Low polarity = normal\n");
      } else {
	printf("    Port C Low polarity = inverted\n");
      }
      if (status & 0x1<<6) {
	printf("    Port C Low  = pull up\n");
      } else {
	printf("    Port C Low  = pull down\n");
      }
      /* Port C High */
      if (status & 0x1<<4) {
	printf("    Port C High polarity = normal\n");
      } else {
	printf("    Port C High polarity = inverted\n");
      }
      if (status & 0x1<<7) {
	printf("    Port C High  = pull up\n");
      } else {
	printf("    Port C High  = pull down\n");
      }
      break;
    case 'l':
      if (device == USBERB24_PID) {
	printf("Temperature = %f C\n", usbGetTemp_USBERB(hid));
      } else {
        printf("N/A on ERB08\n");
      }
      break;
    case 'r':
      usbReset_USBERB(hid);
      return 0;
      break;
    case 'e':
      hid_close(hid);
      hid_exit();
      exit(0);
    default:
      break;
    }
  }
}
