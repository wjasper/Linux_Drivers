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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>

#include "pmd.h"
#include "usb-tc-ai.h"

#define MAX_STR 255

float volts_FS(const int gain, const signed short num);

/* Test Program */
float celsius2fahr( float celsius)
{
  return (celsius*9.0/5.0 + 32.);
}

float fahr2celsius( float fahr)
{
  return (fahr - 32.)*5.0/9.0;
}

int toContinue()
{
  int answer;
  answer = 0; //answer = getchar();
  printf("Continue [yY]? ");
  while((answer = getchar()) == '\0' ||
    answer == '\n');
  return ( answer == 'y' || answer == 'Y');
}

int main(int argc, char **argv) {

  int ch;
  int i;
  int nchan;
  uint8_t bIReg, bOReg;
  uint8_t in_options, out_options;
  float temperature;
  float volts;
  float temperature_array[8];
  float value_1, value_2;
  int flag;
  char type[10];

  hid_device *hid = 0x0;  // Composite device with 1 interface.
  int ret;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];
  
  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB_TC_AI_PID, NULL)) > 0) {
      printf("USB TC-AI is found.\n");
  } else {
    fprintf(stderr, "USB-TC-AI is not found.\n");
    exit(0);
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USBTC_AI(hid, DIO_DIR_OUT);
  usbDOut_USBTC_AI(hid, 0x0);

  while(1) {
    printf("\nUSB TC-AI Testing\n");
    printf("----------------\n");
    printf("Hit 'a' to get alarm status\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to calibrate\n");
    printf("Hit 'd' to test DIO\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'f' for burnout status\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'h' to run the counter\n");
    printf("Hit 'i' for information\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 'p' read the CJC\n");
    printf("Hit 's' to get status\n");
    printf("Hit 't' to measure temperature\n");
    printf("Hit 'x' to measure temperature (Thermocouple) multiple channels\n");
    printf("Hit 'v' to measure voltage (channels 4-7)\n");

    while((ch = getchar()) == '\0' ||
      ch == '\n');

    switch(tolower(ch)) {
      case 'a':
        for ( i = 0; i < 8; i++ ) {
	  value_1 = 1.0 + i;
	  value_2 = 0.0;
	  in_options = 0x40 | i;
	  out_options = 0x0;  // disable alarm
          usleep(10000);
          usbConfigAlarm_USBTC_AI(hid, i, in_options, out_options, value_1, value_2);
          usleep(10000);
    	  usbGetAlarmConfig_USBTC_AI(hid, i, &in_options, &out_options, &value_1, &value_2);
	  printf("Alarm %d: input options = %#x  output options = %#x  value_1 = %f  value_2 = %f\n",
		 i, in_options, out_options, value_1, value_2);
	}
        printf("\n");
	break;
      case 'b': /* test to see if led blinks */
        usbBlink_USBTC_AI(hid);
        break;
      case 'c': /* calibration */
        usbCalibrate_USBTC_AI(hid, 0);
        usbCalibrate_USBTC_AI(hid, 1);
        break;
      case 'f':  /* Get status of thermocouple burnout detection */
	 printf("Burnout status = %#x\n", usbGetBurnoutStatus_USBTC_AI(hid, 0xf));
	 break;
      case 'g':
	hid_get_serial_number_string(hid, serial, 64);
        printf("Serial Number = %ls\n", serial);
        break;
      case 'p':  /* read the CJC */
        usbAin_USBTC_AI(hid, CJC0, 0, &temperature);
	printf("CJC 0 = %.2f degress Celsius or %.2f degrees Fahrenheit.\n", temperature,
	        celsius2fahr(temperature));
        break;
      case 'd': /* test to see if led blinks */
        printf("conect DIO0 - DIO4\n");
	printf("conect DIO1 - DIO5\n");
	printf("conect DIO2 - DIO6\n");
	printf("conect DIO3 - DIO7\n");
	usbDConfigBit_USBTC_AI(hid, 0,  DIO_DIR_OUT);
	usbDConfigBit_USBTC_AI(hid, 1,  DIO_DIR_OUT);
	usbDConfigBit_USBTC_AI(hid, 2,  DIO_DIR_OUT);
	usbDConfigBit_USBTC_AI(hid, 3,  DIO_DIR_OUT);
	usbDConfigBit_USBTC_AI(hid, 4,  DIO_DIR_IN);
	usbDConfigBit_USBTC_AI(hid, 5,  DIO_DIR_IN);
	usbDConfigBit_USBTC_AI(hid, 6,  DIO_DIR_IN);
	usbDConfigBit_USBTC_AI(hid, 7,  DIO_DIR_IN);
	do {
  	  printf("Enter value [0-f]: ");
	  scanf("%hhx", &bIReg);
	  bIReg &= 0xf;
  	  usbDOut_USBTC_AI(hid, bIReg);
	  usbDIn_USBTC_AI(hid, &bOReg);
	  printf("value = %#hhx\n", bOReg);
	} while (toContinue());
	break;
      case 'h':
	printf("connect CTR to DIO0.\n");
	usbDConfigBit_USBTC_AI(hid, 0,  DIO_DIR_OUT);
	usbInitCounter_USBTC_AI(hid);
        sleep(1);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
        do {
          usbDOutBit_USBTC_AI(hid, 0, 1);
	  usleep(200000);
          usbDOutBit_USBTC_AI(hid, 0, 0);
	  printf("Counter = %d\n",usbReadCounter_USBTC_AI(hid));
        } while (!isalpha(getchar()));
        fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'i':
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
      case 's':
        printf("Status = %#x\n", usbGetStatus_USBTC_AI(hid));
	break;
      case 'r':
        usbReset_USBTC_AI(hid);
        return 0;
	break;
      case 'e':
	hid_close(hid);
        hid_exit();
        return 0;
	break;
      case 't':
        printf("Select Channel [0-3]: ");
	scanf("%d", &i);
        if ( i > 3 ) {
          printf("Channel must be between 0-3.\n");
          i = 0;
	}
        printf("Connect thermocouple to channel %d\n", i);
	printf(" Select Thermocouple Type [JKSRBETN]: ");
	scanf("%s", type);
	switch(type[0]) {
	case 'J':
	  bIReg = TYPE_J;
	  printf("Type J Thermocouple Selected: \n");
	  break;
	case 'K':
	  bIReg = TYPE_K;
  	  printf("Type K Thermocouple Selected: \n");
	  break;
	case 'T':
	  bIReg = TYPE_T;
    	  printf("Type T Thermocouple Selected: \n");
	  break;
	case 'E':
	  bIReg = TYPE_E;
    	  printf("Type E Thermocouple Selected: \n");
	  break;
	case 'R':
	  bIReg = TYPE_R;
      	  printf("Type R Thermocouple Selected: \n");
	  break;
	case 'S':
	  bIReg = TYPE_S;
       	  printf("Type S Thermocouple Selected: \n");
	  break;
	case 'B':
	  bIReg = TYPE_B;
       	  printf("Type B Thermocouple Selected: \n");
	  break;
	case 'N':
	  bIReg = TYPE_N;
       	  printf("Type N Thermocouple Selected: \n");
	  break;
        default:
	  printf("Unknown or unsupported thermocopule type.\n");
	  break;
	}
        usbSetItem_USBTC_AI(hid, i, i%2+CH_0_TC, bIReg);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbAin_USBTC_AI(hid, i, 0, &temperature);
    	  printf("Channel: %d  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		 i, temperature, celsius2fahr(temperature));
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'x':
	printf("Enter number of Channels (1-4): ");
	scanf("%d", &nchan);
        if (nchan > 4) nchan = 4;
	for ( i = 0; i < nchan; i++ ) {
          printf("Connect thermocouple to channel %d\n", i);
	  printf(" Select Thermocouple Type [JKSRBETN]: ");
	  scanf("%s", type);
	  switch(type[0]) {
	  case 'J':
	    bIReg = TYPE_J;
	    printf("Type J Thermocouple Selected: \n");
	    break;
	  case 'K':
	    bIReg = TYPE_K;
  	    printf("Type K Thermocouple Selected: \n");
	    break;
	  case 'T':
	    bIReg = TYPE_T;
    	    printf("Type T Thermocouple Selected: \n");
	    break;
	  case 'E':
	    bIReg = TYPE_E;
    	    printf("Type E Thermocouple Selected: \n");
	    break;
	  case 'R':
	    bIReg = TYPE_R;
      	    printf("Type R Thermocouple Selected: \n");
	    break;
	  case 'S':
	    bIReg = TYPE_S;
       	    printf("Type S Thermocouple Selected: \n");
	    break;
	  case 'B':
	    bIReg = TYPE_B;
       	    printf("Type B Thermocouple Selected: \n");
	    break;
	  case 'N':
	    bIReg = TYPE_N;
       	    printf("Type N Thermocouple Selected: \n");
	    break;
          default:
	    printf("Unknown or unsupported thermocopule type.\n");
	    break;
	  }
          usbSetItem_USBTC_AI(hid, i/2, i%2+CH_0_TC, bIReg);
	}
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbAinScan_USBTC_AI(hid, CH0, nchan-1, 0, temperature_array);
	  for ( i = 0; i < nchan; i++ ) {
  	    printf("Channel %d:  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		   i, temperature_array[i], celsius2fahr(temperature_array[i]));
	  }
	  printf("\n");
	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'v':
      	printf("Select Channel [4-7]: ");
        scanf("%d", &i);
        if ( i < 4 || i > 7 ) {
	  printf("Channel must be between 4-7 inclusive.  Setting to 4\n");
	i = 4;
    	}
	//usbSetItem_USBTC_AI(hid, i/2, SENSOR_TYPE, VOLTAGE);
	usbSetItem_USBTC_AI(hid, i/2, i%2+CH_0_VOLT_CONN, DIFFERENTIAL);
	usbSetItem_USBTC_AI(hid, i/2, i%2+CH_0_GAIN, GAIN_4X);  // +/- 10V.
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbAin_USBTC_AI(hid, i, 0, &volts);
    	  printf("Channel: %d  %.2f Volts.\n", i, volts);
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
        break;
      default:
        break;
    }
  }
}
