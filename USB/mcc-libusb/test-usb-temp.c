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
#include "usb-temp.h"
#include "usb-1024LS.h"

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

int main(int argc, char **argv)
{
  int ch;
  uint8_t bIReg, bOReg;
  float temperature;
  float temperature_array[8];
  int nchan;
  int flag;
  char type;
  int i;
  float R0, A, B, C;      // RTD: Callendar-Van Dusen coefficients
  float A0, A1, A2;       // Thermistor: Steinhart-Hart coefficients
  float Offset, Scale;    // Semiconductor
  float value;
  
  int ret;
  hid_device *hid = 0x0;  // Composite device with 1 interface.
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];
  
  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB_TEMP_PID, NULL)) > 0) {
    printf("USB-TEMP Device is found!\n");
  } else {
    fprintf(stderr, "USB-TEMP not found.\n");
    exit(1);
  }

  /* config mask 0x01 means all inputs */
  usbDConfigPort_USBTEMP(hid, DIO_DIR_OUT);
  usbDOut_USBTEMP(hid, 0x0);

  while(1) {
    printf("\nUSB-TEMP Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink LED\n");
    printf("Hit 'c' to calibrate\n");
    printf("Hit 'd' to test DIO\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'f' for burnout status\n");
    printf("Hit 'g' to get serial number\n");
    printf("Hit 'i' for information\n");
    printf("Hit 'R' to reset\n");
    printf("Hit 'r' to measure temperature (RTD)\n");
    printf("Hit 'p' read the CJC\n");
    printf("Hit 's' to get status\n");
    printf("Hit 'S' to measure temperature (Semiconductor)\n");
    printf("Hit 't' to measure temperature (Thermocouple)\n");
    printf("Hit 'T' to measure temperature (Thermistor)\n");    
    printf("Hit 'x' to measure temperature (Thermocouple) multiple channels\n");

    while((ch = getchar()) == '\0' ||  ch == '\n');

    switch(ch) {
      case 'b': /* test to see if led blinks */
        usbBlink_USBTEMP(hid);
        break;
      case 'c': /* calibration */
        usbCalibrate_USBTEMP(hid);
        break;
      case 'f':  /* Get status of thermocouple burnout detection */
	 printf("Burnout status = %#x\n", usbGetBurnoutStatus_USBTEMP(hid, 0xf));
	 break;
      case 'g':
	hid_get_serial_number_string(hid, serial, 64);
        printf("Serial Number = %ls\n", serial);
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
      case 'p':  /* read the CJC */
        usbTin_USBTEMP(hid, CJC0, 0, &temperature);
	printf("CJC 0 = %.2f degress Celsius or %.2f degrees Fahrenheit.\n", temperature,
	        celsius2fahr(temperature));
        usbTin_USBTEMP(hid, CJC1, 0, &temperature);
	printf("CJC 1 = %.2f degress Celsius or %.2f degrees Fahrenheit.\n", temperature,
	        celsius2fahr(temperature));
        break;
      case 'd': /* test to see if led blinks */
        printf("conect DIO0 - DIO4\n");
	printf("conect DIO1 - DIO5\n");
	printf("conect DIO2 - DIO6\n");
	printf("conect DIO3 - DIO7\n");
	usbDConfigBit_USBTEMP(hid, 0,  DIO_DIR_OUT);
	usbDConfigBit_USBTEMP(hid, 1,  DIO_DIR_OUT);
	usbDConfigBit_USBTEMP(hid, 2,  DIO_DIR_OUT);
	usbDConfigBit_USBTEMP(hid, 3,  DIO_DIR_OUT);
	usbDConfigBit_USBTEMP(hid, 4,  DIO_DIR_IN);
	usbDConfigBit_USBTEMP(hid, 5,  DIO_DIR_IN);
	usbDConfigBit_USBTEMP(hid, 6,  DIO_DIR_IN);
	usbDConfigBit_USBTEMP(hid, 7,  DIO_DIR_IN);
	do {
  	  printf("Enter value [0-f]: ");
	  scanf("%hhx", &bIReg);
	  bIReg &= 0xf;
  	  usbDOut_USBTEMP(hid, bIReg);
	  usbDIn_USBTEMP(hid, &bOReg);
	  printf("value = %#x\n", bOReg);
	} while (toContinue());
	break;
      case 's':
        printf("Status = %#x\n", usbGetStatus_USBTEMP(hid));
	break;
      case 'S':
        printf("Sampling Semiconductor TMP36\n");
	printf("Enter channel number [0-7]: ");
	scanf("%d", &ch);
        usbSetItem_USBTEMP(hid, ch/2, SENSOR_TYPE, SEMICONDUCTOR);
	printf("        1.  Single ended.\n");
	printf("        2.  Differential.\n");
	printf("Enter connection type [1-2]: \n");
	scanf("%d", &i);
	switch (i) {
	  case 1: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, SINGLE_ENDED);break;
	  case 2: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, DIFFERENTIAL); break;
	  default: printf("Unknown type\n"); break;
	}
	printf("Enter Offset: ");
	scanf("%f", &Offset);
	printf("Enter Scale: ");
	scanf("%f", &Scale);
	usbSetItem_USBTEMP(hid, ch/2, EXCITATION, EXCITATION_OFF);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_GAIN + ch%2, 0x1);       // Set for Semiconductor
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_0 + ch%2, Offset);  // Offset
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_0 + ch%2, &value);
	printf("Offset = %f     ", value);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_1 + ch%2, Scale);   // Scale
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_1 + ch%2, &value);
	printf("Scale = %f     ", value);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(fileno(stdin), F_SETFL, flag | O_NONBLOCK);
	do {
          usbTin_USBTEMP(hid, ch, 0, &temperature);
  	  printf("Channel: %d  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		 ch, temperature, celsius2fahr(temperature));
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'r':
        printf("Sampling RTD\n");
	printf("Enter channel number [0-7]: ");
	scanf("%d", &ch);
        usbSetItem_USBTEMP(hid, ch/2, SENSOR_TYPE, RTD);
	printf("        1.  2-wire with 1 sensor.\n");
	printf("        2.  2-wire with 2 sensors.\n");
	printf("        3.  3-wire. \n");
	printf("        4.  4-wire.\n");
	printf("Enter connector type [1-4]: \n");
	scanf("%d", &i);
	switch (i) {
	  case 1: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, TWO_WIRE_ONE_SENSOR);break;
	  case 2: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, TWO_WIRE_TWO_SENSOR); break;
	  case 3: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, THREE_WIRE);
	    printf("Connection-type = 3 wire.\n");
	    break;
	  case 4: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, FOUR_WIRE);
	    printf("Connection-type = 4 wire.\n");
	    break;
	}
	R0 = 100.;
	A = .003908;
	B = -5.8019E-7;
	C = -4.2735E-12;
	usbSetItem_USBTEMP(hid, ch/2, EXCITATION, MU_A_210);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_GAIN + ch%2, 0x2);          // Set 0 - 0.5V for RTD
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_0 + ch%2, R0);         // R0 value
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_0 + ch%2, &value);
	printf("R0 = %f     ", value);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_1 + ch%2, A);          // Callendar-Van Dusen Coefficient A
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_1 + ch%2, &value);
	printf("A = %e     ", value);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_2 + ch%2, B);          // Callendar-Van Dusen Coefficient B
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_2 + ch%2, &value);
	printf("B = %e     ", value);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_3 + ch%2, C);          // Callendar-Van Dusen Coefficient C
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_3 + ch%2, &value);
	printf("C = %e\n", value);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(fileno(stdin), F_SETFL, flag | O_NONBLOCK);
	do {
          usbTin_USBTEMP(hid, ch, 0, &temperature);
  	  printf("Channel: %d  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		 ch, temperature, celsius2fahr(temperature));
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'R':
        usbReset_USBTEMP(hid);
        return 0;
	break;
      case 'e':
	hid_close(hid);
        hid_exit();
        return 0;
	break;
      case 't':
        printf("Select Channel [0-7]: ");
        scanf("%d", &i);
	usbSetItem_USBTEMP(hid, i/2, SENSOR_TYPE, THERMOCOUPLE);
        usbSetItem_USBTEMP(hid, i/2, EXCITATION, EXCITATION_OFF);
        printf("Connect thermocouple to channel %d\n", i);
	printf(" Select Thermocouple Type [JKSRBETN]: ");
	scanf("%s", &type);
	switch(type) {
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
        usbSetItem_USBTEMP(hid, i/2, i%2+CH_0_TC, bIReg);
        usbCalibrate_USBTEMP(hid);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbTin_USBTEMP(hid, i, 0, &temperature);
	  printf("Channel: %d  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		 i, temperature, celsius2fahr(temperature));
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'T':
        printf("Sampling Thermistor\n");
	printf("Enter channel number [0-7]: ");
	scanf("%d", &ch);
        usbSetItem_USBTEMP(hid, ch/2, SENSOR_TYPE, THERMISTOR);
	printf("        1.  2-wire with 1 sensor.\n");
	printf("        2.  2-wire with 2 sensors.\n");
	printf("        3.  3-wire.\n");
	printf("        4.  4-wire.\n");
	printf("Enter connector type [1-4]: \n");
	scanf("%d", &i);
	switch (i) {
	  case 1: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, TWO_WIRE_ONE_SENSOR);break;
	  case 2: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, TWO_WIRE_TWO_SENSOR); break;
	  case 3: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, THREE_WIRE);
	    printf("Connection-type = 3 wire.\n");
	    break;
	  case 4: usbSetItem_USBTEMP(hid, ch/2, CONNECTION_TYPE, FOUR_WIRE);
	    printf("Connection-type = 4 wire.\n");
	    break;
	}
	printf("Enter Steinhart-Hart coefficient A0: ");
	scanf("%f", &A0);
	printf("Enter Steinhart-Hart coefficient A1: ");
	scanf("%f", &A1);
	printf("Enter Steinhart-Hart coefficient A2: ");
	scanf("%f", &A2);
	usbSetItem_USBTEMP(hid, ch/2, EXCITATION, MU_A_10);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_GAIN + ch%2, 0x0);      // Set for Thermnistor
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_0 + ch%2, A0);     // Steinhart-Hart coefficient A0
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_0 + ch%2, &value);
	printf("A0 = %f     ", value);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_1 + ch%2, A1);     // Steinhart-Hart coefficient A1
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_1 + ch%2, &value);
	printf("A1 = %e     ", value);
	usbSetItem_USBTEMP(hid, ch/2, CH_0_COEF_2 + ch%2, A2);     // Steinhart-Hart coefficient A2
	usbGetItem_USBTEMP(hid, ch/2, CH_0_COEF_2 + ch%2, &value);
	printf("A2 = %e     ", value);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(fileno(stdin), F_SETFL, flag | O_NONBLOCK);
	do {
          usbTin_USBTEMP(hid, ch, 0, &temperature);
  	  printf("Channel: %d  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		 ch, temperature, celsius2fahr(temperature));
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'x':
        printf("Enter number of Channels (1-8): ");
	scanf("%d", &nchan);
	for ( i = 0; i < nchan; i++ ) {
	  usbSetItem_USBTEMP(hid, i/2, SENSOR_TYPE, THERMOCOUPLE);
	  usbSetItem_USBTEMP(hid, i/2, EXCITATION, EXCITATION_OFF);
          printf("Connect thermocouple to channel %d\n", i);
	  printf("Select Thermocouple Type [JKSRBETN]: ");
	  scanf("%s", &type);
	  switch(type) {
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
          usbSetItem_USBTEMP(hid, i/2, i%2+CH_0_TC, bIReg);
	}
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbTinScan_USBTEMP(hid, CH0, nchan-1, 0, temperature_array);
	  for ( i = 0; i < nchan; i++ ) {
  	    printf("Channel %d:  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		   i, temperature_array[i], celsius2fahr(temperature_array[i]));
	  }
	  printf("\n");
	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      default:
        break;
    }
  }
}
