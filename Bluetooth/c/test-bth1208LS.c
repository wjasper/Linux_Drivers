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
#include <fcntl.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "bth-1208LS.h"

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
  struct tm calDate;
  DeviceInfo_BTH1208LS device_info;
  uint8_t options;
  uint8_t channel, channels;
  int flag;
  int ch;
  int i, j, k, m;
  char serial[9];  // serial number
  uint16_t status, version, radioVersion;
  uint8_t range;
  uint8_t ranges[4] = {0, 0, 0, 0};
  uint16_t value;
  uint16_t values[2];
  uint32_t count;
  double frequency, voltage;
  uint16_t dataAIn[4*12192];  // holds 16 bit unsigned analog input data
  uint16_t data;
  int nChan, repeats, nScan;

  memset(&device_info, 0x0, sizeof(device_info));
  device_info.device.frameID = 0;         // zero out the frameID

  if (argc == 2) {
    strncpy(device_info.device.baddr, argv[1], 18);
    printf("BTH-1208LS BADDR address = %s\n", argv[1]);
  } else if (discoverDevice(&device_info.device, "BTH-1208LS-6833") < 0) {
    printf("No device found.\n");
    return -1;
  }

  if (openDevice(&device_info.device) < 0) {
    printf("Error openeing device\n");
    exit(-1);
  } else {
    printf("Found a Bluetooth BTH-1208LS.\n");
  }

  // some initialization
  BuildGainTable_DE_BTH1208LS(&device_info);
  BuildGainTable_SE_BTH1208LS(&device_info);
  for (i = 0; i < NCHAN_DE; i++) {   // channel
    for (j = 0; j < NGAINS; j++ ) {      // range 
      printf("Calibration Table (Differential): Channel = %d  Gain = %d  Slope = %f   Offset = %f\n", 
	     i, j, device_info.table_AInDE[i][j].slope, device_info.table_AInDE[i][j].intercept);
    }
  }
  printf("\n");
  for (i = 0; i < NCHAN_SE; i++ ) {  // channel  (SE only has 1 gain range)
    printf("Calibration Table (Single Ended): Channel = %d  Slope = %f   Offset = %f\n", 
	   i, device_info.table_AInSE[i].slope, device_info.table_AInSE[i].intercept);
  }

  CalDate_BTH1208LS(&device_info, &calDate);
  printf("\n");
  printf("MFG Calibration date = %s\n", asctime(&calDate));

  while(1) {
    printf("\n\nBTH-1208LS Testing\n");
    printf("----------------\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to test counter\n");
    printf("Hit 'C' to for continuous sampling\n");
    printf("Hit 'd' to test digitial IO\n");
    printf("Hit 'i' to test Analog Input\n");
    printf("Hit 'I' to test Analog Input Scan\n");
    printf("Hit 'o' to test Analog Output\n");
    printf("Hit 'x' to test Analog Input Scan (Multi-channel)\n");
    printf("Hit 'r' to reset the device\n");
    printf("Hit 's' to get serial number\n");
    printf("Hit 'S' to get Status\n");
    printf("Hit 'v' to get the battery voltage in mV\n");
    printf("Hit 'e' to exit\n");

    while((ch = getchar()) == '\0' || ch == '\n');
    switch(ch) {
      case 'b': /* test to see if LED blinks */
        printf("Enter number or times to blink: ");
        scanf("%hhd", &options);
	BlinkLED_BTH1208LS(&device_info, options);
        break;
      case 'c':
        ResetCounter_BTH1208LS(&device_info);
        printf("Connect AO 0 to CTR.\n");
        toContinue();
        for (i = 0; i < 100; i++) {                   // toggle
    	  AOut_BTH1208LS(&device_info, 0, 4095);
	  AOut_BTH1208LS(&device_info, 0, 0);
        }
	Counter_BTH1208LS(&device_info, &count);
        printf("Count = %d.  Should read 100.\n", count);
        break;
      case 'd':
	printf("Test of DIO Out\n");
	printf("Input value [0-ff]: ");
	scanf("%hhx", &options);
	DOut_BTH1208LS(&device_info, options);
	DOutR_BTH1208LS(&device_info, &options);
	printf("The value you wrote = %#x", options);
	break;
      case 'e':
        close(device_info.device.sock);
	printf("Success!\n");
	return 0;
	break;
      case 'i':
	printf("Input channel DE [0-3]: ");
	scanf("%hhd", &channel);
        printf("Input range [0-7]: ");
	scanf("%hhd", &range);
	for (i = 0; i < 20; i++) {
	  AIn_BTH1208LS(&device_info, channel, DIFFERENTIAL, range, &value);
	  value = rint(value*device_info.table_AInDE[channel][range].slope + device_info.table_AInDE[channel][range].intercept);
  	  printf("Range %d  Channel %d   Sample[%d] = %#x Volts = %lf\n",
		   range, channel,  i, value, volts_BTH1208LS(value, range));
	  usleep(50000);	  
	}
        break;
      case 'I':
	printf("Testing BTH-1208lS Analog Input Scan.\n");
        printf("Enter number of scans: ");
        scanf("%d", &count);
	printf("Input channel 0-3: ");
        scanf("%hhd", &channel);
        printf("Enter sampling frequency [Hz]: ");
	scanf("%lf", &frequency);
        printf("Enter Range [0-7]: ");
        scanf("%hhd", &range);
        ranges[channel] = range;
        options = DIFFERENTIAL_MODE;
	AInScanStop_BTH1208LS(&device_info);
	AInScanClearFIFO_BTH1208LS(&device_info);
        AInConfigW_BTH1208LS(&device_info, ranges);
	memset(dataAIn, 0x0, sizeof(dataAIn));
	AInConfigR_BTH1208LS(&device_info, ranges);
	AInScanStart_BTH1208LS(&device_info, count, 0x0, frequency, (0x1<<channel), options);
	AInScanRead_BTH1208LS(&device_info, count, dataAIn);
	for (i = 0; i < count; i++) {
	  dataAIn[i] = rint(dataAIn[i]*device_info.table_AInDE[channel][range].slope + device_info.table_AInDE[channel][range].intercept);
          printf("Range %d Channel %d  Sample[%d] = %#x Volts = %lf\n", range, channel,
		 i, dataAIn[i], volts_BTH1208LS(dataAIn[i], range));
	}
        break;
      case 'x':
        printf("Testing BTH-1208LS Multi-Channel Analog Input Scan.\n");
        printf("Enter number of channels (1-4): ");
        scanf("%d", &nChan);
        printf("Enter number of scans: ");
        scanf("%d", &nScan);
        printf("Enter number of repeats: ");
        scanf("%d", &repeats);
	printf("Enter sampling frequency: ");
        scanf("%lf", &frequency);
        // Build bitmap for the first nChan in channels.
        channels = 0;
        for (i = 0; i < nChan; i++) {
	  channels |= (1 << i);
	}
        // Always use BP_20V to make it easy (BP_20V is 0...)
        range = BP_20V;
        memset(ranges, BP_20V, sizeof(ranges));
	AInScanStop_BTH1208LS(&device_info);
	AInScanClearFIFO_BTH1208LS(&device_info);
        AInConfigW_BTH1208LS(&device_info, ranges);
        options = DIFFERENTIAL_MODE;
        // Run a loop for the specified number of repeats and
        // show the results...
        for (m = 0; m < repeats; m++) {
	  printf("\n\n---------------------------------------");
	  printf("\nrepeat: %d\n", m);
	  AInScanStop_BTH1208LS(&device_info);
	  AInScanClearFIFO_BTH1208LS(&device_info);
	  AInScanStart_BTH1208LS(&device_info, nScan*nChan, 0x0, frequency, channels, options);
	  AInScanRead_BTH1208LS(&device_info, nScan, dataAIn);
	  for (i = 0; i < nScan; i++) {
	    printf("%6d", i);
	    for (j = 0; j < nChan; j++)	{
	      k = i*nChan + j;
	      data = rint(dataAIn[k]*device_info.table_AInDE[j][range].slope + device_info.table_AInDE[j][range].intercept);
	      printf(", %8.4f", volts_BTH1208LS(data, range));
	    } /* for (j - 0; j < 8, j++) */
	    printf("\n");
	  } /* for (i = 0; i < nScan; i++) */
	} /* for (m = 0; m < repeats; m++) */
	printf("\n\n---------------------------------------");
	break;
      case 'C':
        printf("Testing BTH-1208LS Continuous Analog Input Scan.\n");
	printf("Hit any key to exit\n");
	printf("Enter sampling frequency: ");
        scanf("%lf", &frequency);
	channels = 0x2;
	range = BP_20V;
	memset(ranges, BP_20V, sizeof(ranges));
	AInScanStop_BTH1208LS(&device_info);
	AInScanClearFIFO_BTH1208LS(&device_info);
        AInConfigW_BTH1208LS(&device_info, ranges);
        options = DIFFERENTIAL_MODE;
	nScan = 0;    // Continuous scan
	flag = fcntl(fileno(stdin), F_GETFL);
	fcntl(0, F_SETFL, flag | O_NONBLOCK);
	AInScanStart_BTH1208LS(&device_info, nScan, 0x0, frequency, channels, options);
	i = 0;
	do {
	  device_info.nDelay = (127.*1000.)/frequency;     // delay in ms
	  usleep(device_info.nDelay*900);                  // sleep in us
	  j = AInScanSendData_BTH1208LS(&device_info, 127, dataAIn, device_info.nDelay);
	  printf("Scan = %d, samples returned = %d\n", i, j);
	  i++;
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	AInScanStop_BTH1208LS(&device_info);
	break;
      case 'o':
	printf("Test Analog Output\n");
        printf("Enter Channel [0-1] ");
        scanf("%hhd", &channel);
        printf("Enter voltage [0-2.5V]: ");
	scanf("%lf", &voltage);
        value = voltage * 4095 / 2.5;
	AOut_BTH1208LS(&device_info, channel, value);
	AOutR_BTH1208LS(&device_info, values);
	printf("Analog Output Voltage = %f V\n", volts_BTH1208LS(values[channel], UP_2_5V));
        break;
      case 'S':
	Status_BTH1208LS(&device_info, &status);
	FirmwareVersion_BTH1208LS(&device_info, &version);
	RadioFirmwareVersion_BTH1208LS(&device_info, &radioVersion);
	printf("Status = %#x   Firmware Version = %x.%x   Radio Firmware Version = %x.%x\n",
	       status, (version >> 8) & 0xff, version & 0xff, (radioVersion >> 8) & 0xff, radioVersion & 0xff);
	break;
      case 's':
        GetSerialNumber_BTH1208LS(&device_info, serial);
        printf("Serial number = %s\n", serial);
        break;
      case 'r':
        Reset_BTH1208LS(&device_info);
        break;
      case 'v':
        BatteryVoltage_BTH1208LS(&device_info);
        printf("Battery Voltage = %d mV\n", device_info.voltage);
	break;
    }
  }
}
