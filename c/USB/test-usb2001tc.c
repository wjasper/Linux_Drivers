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
#include <math.h>
#include <time.h>
#include "pmd.h"
#include "usb-2001-tc.h"

#define MAX_COUNT     (0xffff)
#define FALSE 0
#define TRUE 1

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
  uint32_t value;
  double CJC_Temp, temp;
  int ret, i, status;
  char ch;
  char serial[9];
  char version[64];
  uint8_t options, tc_type;
  double slope, offset;
  float slope_f, offset_f;
  struct tm date;
  char filename[80];
  FILE *fp;

  ret = libusb_init(NULL);
  if (ret < 0) {
    perror("usb_device_find_USB_MCC: Failed to initialize libusb");
    exit(1);
  }

  if ((udev = usb_device_find_USB_MCC(USB_2001_TC_PID, NULL))) {
    printf("Success, found a USB-2001-TC!\n");
  } else {
    printf("Failure, did not find a USB-2100-TC!\n");
    return 0;
  }

  printf("bMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));
  while(1) {
    printf("\nUSB USB 2001-TC Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to get calibration slope and offset\n");
    printf("Hit 'C' to get calibration date\n");
    printf("Hit 'd' to set device\n");
    printf("Hit 'i' to get CJC and Analog Input readings\n");
    printf("Hit 'F' to get the CJC reading in degree F\n");
    printf("Hit 'K' to get the CJC reading in degree Kelvin\n");
    printf("Hit 'r' to get reset device\n");
    //    printf("Hit 'R' to restore calibration coefficients\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get status\n");
    printf("Hit 't' to get the temperature\n");
    printf("Hit 'T' to write temperature to file\n");
    printf("Hit 'v' to get firmware version\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
        usbBlink_USB2001TC(udev, options);
	break;
      case 'c':
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': 
  	  case 'j':
	    tc_type = TYPE_J; break;
  	  case 'K': 
  	  case 'k': 
	    tc_type = TYPE_K; break;
	  case 'R':
	  case 'r':
	    tc_type = TYPE_R; break;
  	  case 'S':
	  case 's':
	    tc_type = TYPE_S; break;
   	  case 'T':
	  case 't':
	    tc_type = TYPE_T; break;
     	  case 'N':
	  case 'n':
	    tc_type = TYPE_N; break;
       	  case 'E':
  	  case 'e':
	    tc_type = TYPE_E; break;
	  case 'B':
 	  case 'b':
	    tc_type = TYPE_B; break;
	  default: tc_type = TYPE_J; break;
	}
	//sendSensorType_USB2001TC(udev, ch);
        getSlope_USB2001TC(udev, &slope);
        getOffset_USB2001TC(udev, &offset);
        printf("Calibration data: Type = %c  Slope = %f   Offset = %f\n", ch, slope, offset);
        break;
      case 'C':
	getMFGCAL_USB2001TC(udev, &date);
        printf("\nLast Calibration date: %s\n\n", asctime(&date));
        break;
      case 'd':
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	sendSensorType_USB2001TC(udev, ch);
	getSensorType_USB2001TC(udev, &ch);
	// read it back
	printf("Sensor Type = %c\n", ch);
	break;
      case 'i':
	CJC_Temp = -2000;
	getCJCDegC_USB2001TC(udev, &CJC_Temp);
	getCJCDegC_USB2001TC(udev, &CJC_Temp);
	getCJCDegC_USB2001TC(udev, &CJC_Temp);
	printf("CJC = %f degree C. \n", CJC_Temp);

	//	CJC_Temp = -2000;
	//	getCJC_USB2001TC(udev, &CJC_Temp);
	//	printf("CJC = %f \n", CJC_Temp);

	getValue_USB2001TC(udev, &value);
        printf("ADC value = %#x\n", value);
        break;
      case 'F':
	CJC_Temp = -2000;
	getCJCDegF_USB2001TC(udev, &CJC_Temp);
	getCJCDegF_USB2001TC(udev, &CJC_Temp);
	getCJCDegF_USB2001TC(udev, &CJC_Temp);
	printf("CJC = %f degree F. \n", CJC_Temp);
	break;
      case 'K':
	CJC_Temp = -2000;
	getCJCDegKelvin_USB2001TC(udev, &CJC_Temp);
	getCJCDegKelvin_USB2001TC(udev, &CJC_Temp);
	getCJCDegKelvin_USB2001TC(udev, &CJC_Temp);
	printf("CJC = %f degree K. \n", CJC_Temp);
	break;
      case 'g':
        break;
      case 'r':
        reset_USB2001TC(udev);
        break;
      case 'R':
        printf("Enter slope: ");
        scanf("%f", &slope_f);
        printf("Enter offset :");
        scanf("%f", &offset_f);
        usbMemoryUnlock_USB2001TC(udev);
        usbSetMemoryAddress_USB2001TC(udev, 0x200);
        usbMemoryW_USB2001TC(udev, 4, (uint8_t *) &slope_f);
        usbSetMemoryAddress_USB2001TC(udev, 0x200);
        usbMemoryW_USB2001TC(udev, 4, (uint8_t *) &offset_f);
        usbMemoryLock_USB2001TC(udev);
        break;
      case 's':
        usbGetSerialNumber_USB2001TC(udev, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'S':
        status = getStatus_USB2001TC(udev);
        printf("Status = %d\n", status);
        break;
      case 't':
        // put the board in the correct voltage range +/- 73.125mV
        setVoltageRange_USB2001TC(udev, 4);
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': 
  	  case 'j':
	    tc_type = TYPE_J; break;
  	  case 'K': 
  	  case 'k': 
	    tc_type = TYPE_K; break;
	  case 'R':
	  case 'r':
	    tc_type = TYPE_R; break;
  	  case 'S':
	  case 's':
	    tc_type = TYPE_S; break;
   	  case 'T':
	  case 't':
	    tc_type = TYPE_T; break;
     	  case 'N':
	  case 'n':
	    tc_type = TYPE_N; break;
       	  case 'E':
  	  case 'e':
	    tc_type = TYPE_E; break;
	  case 'B':
 	  case 'b':
	    tc_type = TYPE_B; break;
	  default: tc_type = TYPE_J; break;
	}

	getCJCDegC_USB2001TC(udev, &CJC_Temp);
	printf("CJC value = %f C\n", CJC_Temp);

        for (i = 0; i < 10; i++) {
          if (tc_temperature_USB2001TC(udev, tc_type, &temp) >= 0) {
             printf("Thermocouple of Type %c:  Temperature = %.3f C   %.3f F\n", ch, temp, temp*9./5 + 32.);
	  } else {
	    printf("Error.  Check Thermocouple\n");
	  }
	  sleep(1);
	}
        break;
      case 'T':
	printf("Enter filename: ");
	scanf("%s", filename);
	fp = fopen(filename, "w");
	
        // put the board in the correct voltage range +/- 73.125mV
        setVoltageRange_USB2001TC(udev, 4);
	printf("Input Thermocouple type [J,K,R,S,T,N,E,B]: ");
	while((ch = getchar()) == '\0' || ch == '\n');
	switch(ch) {
	  case 'J': 
  	  case 'j':
	    tc_type = TYPE_J; break;
  	  case 'K': 
  	  case 'k': 
	    tc_type = TYPE_K; break;
	  case 'R':
	  case 'r':
	    tc_type = TYPE_R; break;
  	  case 'S':
	  case 's':
	    tc_type = TYPE_S; break;
   	  case 'T':
	  case 't':
	    tc_type = TYPE_T; break;
     	  case 'N':
	  case 'n':
	    tc_type = TYPE_N; break;
       	  case 'E':
  	  case 'e':
	    tc_type = TYPE_E; break;
	  case 'B':
 	  case 'b':
	    tc_type = TYPE_B; break;
	  default: tc_type = TYPE_J; break;
	}

	getCJCDegC_USB2001TC(udev, &CJC_Temp);
	printf("CJC value = %f C\n", CJC_Temp);

        for (i = 0; i < 1000; i++) {
          if (tc_temperature_USB2001TC(udev, tc_type, &temp) >= 0) {
             printf("Thermocouple of Type %c:  Temperature = %.3f C   %.3f F\n", ch, temp, temp*9./5 + 32.);
	     fprintf(fp, "%5d,%.3f,%.3f\n", i+1, temp, temp*9./5. + 32.);
	  } else {
	    printf("Error.  Check Thermocouple\n");
	  }
	  sleep(1);
	}
	fclose(fp);
        break;
      case 'v':
        getFirmwareVersion_USB2001TC(udev, version);
        printf("Firmware version = %s\n", version);
        break;
      case 'e':
        cleanup_USB2001TC(udev);
        return 0;
      default:
        break;
    }
  }
}
