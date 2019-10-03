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
#include <time.h>

#include "pmd.h"
#include "usb-5200.h"

#define MAX_STR 255

float volts_FS(const int gain, const signed short num);

/* Test Program */
float celsius2fahr(float celsius)
{
  return (celsius*9.0/5.0 + 32.);
}

float fahr2celsius(float fahr)
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

void readHeader(int fd, file_header *header)
{
  deviceTime date;
  int second;
  int minute;
  int hour;
  int day;
  int month;
  int year;

  read(fd, header, sizeof(file_header));
  memcpy(&second, header->seconds, 4);
  printf("Header information: \n");
  printf("MCC file identifier = %#x\tData file version = %d\tLogging options = %#x\n",
	 header->identifier, header->version, header->options);
  printf("Channels logged = %#x\t\tData units = %d\t\tNumber of seconds between entries = %hd\n",
	 header->channels, header->units, second);
  memcpy(&date, header->start_time, 6);  // note time_zone not logged into header.
  second = (date.seconds >> 4)*10 + (date.seconds & 0xf);
  minute = (date.minutes >> 4)*10 + (date.minutes & 0xf);
  hour =   (date.hours >> 4)*10 + (date.hours & 0xf);
  day =    (date.day >> 4)*10 + (date.day & 0xf);
  month =  (date.month >> 4)*10 + (date.month & 0xf);
  year =   (date.year >> 4)*10 + (date.year & 0xf) + 2000;
  if (hour >= 12) {
    if (hour >= 13) hour -= 12;
    printf("Logging Time started:  %02d/%02d/%4d %02d:%02d:%02d p.m.\n",
	   month, day, year, hour, minute, second);
  } else {
    if (hour == 0) hour = 12;
    printf("Logging Time started:  %02d/%02d/%4d %02d:%02d:%02d a.m.\n",
	   month, day, year, hour, minute, second);
  }
  printf("\n\n");
}

void readFile(int fd, file_header header)
{
  int nbytes = 0;      // number of bytes per record (depends on header options).
  int i;
  int channels[8];
  int nchan = 0;       // number of channels
  int log_CJC = 0;     // 1 = true, 0 = false;
  int timestamp = 0;   // 1 = ture, 0 = false;
  int second;
  int minute;
  int hour;
  int day;
  int month;
  int year;
  float temp[8];
  float CJC[2];
  deviceTime date;
  char *record;        //pointer to 1 data record

  if (header.options & LOG_TIME) {
    nbytes += 6;  // Time readings were performed.
    timestamp = 1;
    printf("       Time Stamp       ");
  }

  nbytes++;         // Value of the DIO pins at the logging time.
  printf("  DIO ");
  
  for (i = 0; i < 8; i++) {
    if ((header.channels >> i) & 0x1) {
      channels[nchan] = i;
      printf("  Channel %d  ", channels[nchan]);
      nchan++;
    }
  }
  nbytes += nchan*4;  // Readings from each channel logged in floating point format

  if (header.options & LOG_CJC) {
    log_CJC = 1;
    nbytes += 8;
    printf("   CJC 0        CJC 1  ");
  }
  printf("\n");
  record = malloc(nbytes);

  while( read(fd, record, nbytes) == nbytes ) {
    if (timestamp) {
      memcpy(&date, record, 6);           // note time_zone not logged into header.
      memcpy(temp, &record[7], 4*nchan);  // copy the temperature readings

      second = (date.seconds >> 4)*10 + (date.seconds & 0xf);
      minute = (date.minutes >> 4)*10 + (date.minutes & 0xf);
      hour =   (date.hours >> 4)*10 + (date.hours & 0xf);
      day =    (date.day >> 4)*10 + (date.day & 0xf);
      month =  (date.month >> 4)*10 + (date.month & 0xf);
      year =   (date.year >> 4)*10 + (date.year & 0xf) + 2000;
      if (hour >= 12) {
	if (hour >= 13) hour -= 12;
	printf("%02d/%02d/%4d %02d:%02d:%02d p.m. ",
	       month, day, year, hour, minute, second);
      } else {
	if (hour == 0) hour = 12;
	printf("%02d/%02d/%4d %02d:%02d:%02d a.m. ",
	       month, day, year, hour, minute, second);
      }
      printf("%#hhx", record[6]);
      for (i = 0; i < nchan; i++) {
	printf("  %8.2f  ", celsius2fahr(temp[i]));
      }
      if (log_CJC) {
        memcpy(CJC, &record[4*nchan+7], 8);
	printf("  %8.2f     %8.2f", celsius2fahr(CJC[0]), celsius2fahr(CJC[1]));
      }
      printf("\n");
    } else {
      memcpy(temp, &record[1], 4*nchan);
      printf(" %#hhx", record[0]);
      for (i = 0; i < nchan; i++) {
	printf("    %.2f  ", temp[i]);
      }
      if (log_CJC) {
        memcpy(CJC, &record[4*nchan+1], 8);
	printf("  %.2f     %.2f", CJC[0], CJC[1]);
      }
      printf("\n");
    }
  }
  free(record);
}

int main(int argc, char **argv) {
  float value_1;
  float value_2;
  float temperature;
  float temperature_array[8];

  int nchan;
  int fd;
  hid_device *hid = 0x0;  // Composite device with 1 interface.
  int ret;
  int ch;
  int flag;
  int i;
  int second;
  int minute;
  int hour;
  int day;
  int month;
  int year;

  time_t now;
  uint32_t seconds;

  uint16_t filenumber;

  char type;
  char ans[80];

  uint8_t out_options;
  uint8_t number;
  uint8_t in_options;
  uint8_t units;
  uint8_t channels;
  uint8_t status;
  uint8_t bIReg, bOReg;

  struct tm *tp;
  deviceTime date;
  file_header header;
  wchar_t serial[64];
  wchar_t wstr[MAX_STR];

  ret = hid_init();
  if (ret < 0) {
    fprintf(stderr, "hid_init failed with return code %d\n", ret);
    return -1;
  }

  if ((hid = hid_open(MCC_VID, USB5201_PID, NULL)) > 0) {
    printf("USB 5201 Device is found!\n");
  } else if ((hid = hid_open(MCC_VID, USB5201_OLD_PID, NULL)) > 0) {
    printf("USB 5201 Device is found!\n");
  } else {
    fprintf(stderr, "USB 5201 not found.\n");
    exit(1);
  }
  
  /* config mask 0x01 means all inputs */
  usbDConfigPort_USB5200(hid, DIO_DIR_OUT);
  usbDOut_USB5200(hid, 0);

  while(1) {
    printf("\nUSB-5201 Testing\n");
    printf("----------------\n");
    printf("Hit 'a' to get alarm options\n");
    printf("Hit 'b' to blink\n");
    printf("Hit 'c' to calibrate\n");
    printf("Hit 'd' to test DIO\n");
    printf("Hit 'e' to exit\n");
    printf("Hit 'f' to format memory card\n");
    printf("Hit 'g' to set the device clock\n");
    printf("Hit 'h' to read the time from the device clock\n");
    printf("Hit 'i' to read data logging configuration.\n");
    printf("Hit 'I' for information\n");
    printf("Hit 'j' to set logging configuration.\n");
    printf("Hit 'k' to read file.\n");
    printf("Hit 'p' read the CJC\n");
    printf("Hit 'r' to reset\n");
    printf("Hit 's' to get status\n");
    printf("Hit 't' to measure temperature\n");
    printf("Hit 'u' for burnout status\n");
    printf("Hit 'v' to get serial number\n");        
    printf("Hit 'x' to measure temperature (Thermocouple) multiple channels\n");    

    while((ch = getchar()) == '\0' || ch == '\n');

    switch(ch) {
      case 'a':
        printf("Enter alarm number (0-7): ");
        scanf("%hhd", &number);
        usbGetAlarmConfig_USB5200(hid, number, &in_options, &out_options, &value_1, &value_2);
        printf("Alarm Configuration: alarm number = %d, in_options = %#x, out_options = %#x, value_1 = %f,  value2 = %f\n",
             number, in_options, out_options, value_1, value_2);
        break;
      case 'b': /* test to see if led blinks */
        usbBlink_USB5200(hid);
        break;
      case 'c': /* calibration */
        usbCalibrate_USB5200(hid);
        break;
      case 'd': /* test to see if led blinks */
        printf("conect DIO0 - DIO4\n");
	printf("conect DIO1 - DIO5\n");
	printf("conect DIO2 - DIO6\n");
	printf("conect DIO3 - DIO7\n");
	usbDConfigBit_USB5200(hid, 0,  DIO_DIR_OUT);
	usbDConfigBit_USB5200(hid, 1,  DIO_DIR_OUT);
	usbDConfigBit_USB5200(hid, 2,  DIO_DIR_OUT);
	usbDConfigBit_USB5200(hid, 3,  DIO_DIR_OUT);
	usbDConfigBit_USB5200(hid, 4,  DIO_DIR_IN);
	usbDConfigBit_USB5200(hid, 5,  DIO_DIR_IN);
	usbDConfigBit_USB5200(hid, 6,  DIO_DIR_IN);
	usbDConfigBit_USB5200(hid, 7,  DIO_DIR_IN);
	do {
  	  printf("Enter value [0-f]: ");
	  scanf("%hhx", &bIReg);
	  bIReg &= 0xf;
  	  usbDOut_USB5200(hid, bIReg);
	  usbDIn_USB5200(hid, &bOReg);
	  printf("value = %#x\n", bOReg);
	} while (toContinue());
	break;
      case 'e':
	hid_close(hid);
        hid_exit();
        return 0;
	break;
      case 'f':  
        printf("Formatting Memory card.\n");
	usbFormatCard_USB5200(hid);
	sleep(2);
	break;
      case 'g':
        time(&now);
	tp = localtime(&now);
	tp->tm_mon++;          // MCC uses months from 1-12;
	tp->tm_year -= 100; // MCC year starts at 2000 not 1900.
	date.seconds = ((tp->tm_sec/10<<4) + (tp->tm_sec%10));
	date.minutes = ((tp->tm_min/10<<4) + (tp->tm_min%10));
	date.hours = ((tp->tm_hour/10<<4) + (tp->tm_hour%10));
	date.day = ((tp->tm_mday/10<<4) + (tp->tm_mday%10));
	date.month = ((tp->tm_mon/10<<4) + (tp->tm_mon%10));
	date.year = ((tp->tm_year/10<<4) + (tp->tm_year%10));
	date.time_zone = 0;
	printf("Setting Device time to: %s\n", ctime(&now));
	usbSetDeviceTime_USB5200(hid, &date);
	break;
      case 'h':
        usbGetDeviceTime_USB5200(hid, &date);
	second = (date.seconds >> 4)*10 + (date.seconds & 0xf);
	minute = (date.minutes >> 4)*10 + (date.minutes & 0xf);
	hour =   (date.hours >> 4)*10 + (date.hours & 0xf);
	day =    (date.day >> 4)*10 + (date.day & 0xf);
	month =  (date.month >> 4)*10 + (date.month & 0xf);
	year =   (date.year >> 4)*10 + (date.year & 0xf) + 2000;
        if (hour >= 12) {
	  if (hour >= 13) hour -= 12;
	  printf("Device Time:  %02d/%02d/%4d %02d:%02d:%02d p.m.\n",
		 month, day, year, hour, minute, second);
        } else {
	  if (hour == 0) hour = 12;
	  printf("Device Time:  %02d/%02d/%4d %02d:%02d:%02d a.m.\n",
		 month, day, year, hour, minute, second);
        }
        break;
      case 'i':
        usbGetLoggingConfig_USB5200(hid, &in_options, &channels, &units, &seconds, &filenumber, &date);
	printf("Logging Configuration:\n");
	printf("options = %#x  ", in_options);
	if ((in_options & 0x3) == DISABLE)      printf("    Disable Logging.");
	if ((in_options & 0x3) == POWER_UP)     printf("    Start logging on powerup.");
	if ((in_options & 0x3) == START_BUTTON) printf("    Start logging on button press.");
	if (in_options  & LOG_CJC)              printf("    Log CJC temperatures.");
	if (in_options  & LOG_TIME)             printf("    Log timestamp on each entry.");
	printf("\n");
        printf("channels = %#x, units = %d, seconds = %d, filenumber = %d\n",
	       channels, units, second, filenumber);
        break;
      case 'I':
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
      case 'j':
        in_options = START_BUTTON; // start on button press
	printf("Log CJC temperatures? ");
	scanf("%s", ans);
	if (ans[0] == 'y') {
  	  in_options |= LOG_CJC;
	}
	printf("Log timestamp? ");
	scanf("%s", ans);
	if (ans[0] == 'y') {
  	  in_options |= LOG_TIME;
	}
	channels = 0x1;
	units = 0;
	second = 1;
	filenumber = 1;
        usbConfigureLogging_USB5200(hid, in_options, channels, units, second, filenumber, date);
	printf("Set Logging Configuration: options = %#x, channels = %#x, units = %d, seconds = %d, filenumber = %d\n",
	       in_options, channels, units, second, filenumber);
        break;
      case 'k':
        printf("Opening file log00001.bin\n");
        fd = open("log00001.bin",  O_RDONLY);
	readHeader(fd, &header);
	readFile(fd, header);
	close(fd);
	break;
      case 'p':  /* read the CJC */
        usbTin_USB5200(hid, CJC0, 0, &temperature);
	printf("CJC 0 = %.2f degress Celsius or %.2f degrees Fahrenheit.\n", temperature,
	        celsius2fahr(temperature));
        usbTin_USB5200(hid, CJC1, 0, &temperature);
	printf("CJC 1 = %.2f degress Celsius or %.2f degrees Fahrenheit.\n", temperature,
	        celsius2fahr(temperature));
        break;
      case 'r':
        usbReset_USB5200(hid);
        return 0;
	break;
      case 's':
        status = usbGetStatus_USB5200(hid);
        printf("Status = %#x  ", status);
	if (status & PERFORMING_CALIBRATION) printf("  Performing Channel Calibration.");
	if (status & DAUGHTERBOARD_PRESENT)  printf("  Data logging daughter board present.");
	if (status & MEMORYCARD_PRESENT)     printf("  Memory card present.");
	if (status & READFILE_IN_PROGRESS)   printf("  ReadFile in progress.");
        printf("\n");
	break;
      case 't':
	printf("Select Channel [0-7]: ");
        scanf("%d", &i);
        printf("Connect thermocouple to channel 0\n");
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
        usbSetItem_USB5201(hid, i/2, i%2+CH_0_TC, bIReg);
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbTin_USB5200(hid, i, 0, &temperature);
    	  printf("Channel: %d  %.2f degress Celsius or %.2f degrees Fahrenheit.\n",
		 i, temperature, celsius2fahr(temperature));
  	  sleep(1);
	} while (!isalpha(getchar()));
	fcntl(fileno(stdin), F_SETFL, flag);
	break;
      case 'u':  /* Get status of thermocouple burnout detection */
	 printf("Burnout status = %#x\n", usbGetBurnoutStatus_USB5200(hid, 0x0));
	 break;
      case 'x':
	printf("Enter number of Channels (1-8): ");
	scanf("%d", &nchan);
	for ( i = 0; i < nchan; i++ ) {
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
          usbSetItem_USB5201(hid, i/2, i%2+CH_0_TC, bIReg);
	}
        flag = fcntl(fileno(stdin), F_GETFL);
        fcntl(0, F_SETFL, flag | O_NONBLOCK);
	do {
          usbTinScan_USB5200(hid, CH0, nchan-1, 0, temperature_array);
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
	hid_get_serial_number_string(hid, serial, 64);
        printf("Serial Number = %ls\n", serial);
        break;
      default:
        break;
    }
  }
}

      
