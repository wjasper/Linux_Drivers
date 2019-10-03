/*
 * Copyright (C) 2007  Warren Jasper
 * All rights reserved.
 *
 */


/***************************************************************************
 *
 *  test-das08.c
 *
 *  This program is used to test the PCI-DAS08 Analog to Digital
 *  Linux loadable module.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include "pci-das08.h"

/* Programmable Range and Gain Settings */
#define BP_10_00V  (0x00 << 8)    /* +/- 10V      */
#define BP_5_00V   (0x01 << 8)    /* +/-  5V      */
#define BP_2_50V   (0x02 << 8)    /* +/-  2.5V    */
#define BP_1_25V   (0x03 << 8)    /* +/-  1.25V   */
#define UP_10_00V  (0x08 << 8)    /* 0 - 10V      */
#define UP_5_00V   (0x09 << 8)    /* 0 - 5V       */
#define UP_2_50V   (0x0a << 8)    /* 0 - 2.5V     */
#define UP_1_25V   (0x0b << 8)    /* 0 - 1.25V    */


/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/das08/ad0_0";
int  Mode     = ADC_SOFT_TRIGGER;
int  Count    = 1;
int  NoStop   = 0;
int  Print    = 1;
int  Status;

int fdADC;                   /* A/D file descriptors */

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: adcread \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "   [-dev /dev/das08/ad0_#]  - Specify device file.\n");
  fprintf(stderr, "   [-ct ##]                 - Number of samples to read\n");
  fprintf(stderr, "   [-noprint]               - Don't print samples\n");
  fprintf(stderr, "   [-nostop]                - Sample forever\n");
  fprintf(stderr, "\n");
  exit(1);
}

void DoCommandLine(int argc, char **argv)
{
  int i = 1;

  while (i < argc) {
    if (strcmp(argv[i], "-dev") == 0) {
      i++;
      if (i == argc) {
        Usage();
      } else {
        DevName = argv[i];
      }
    } else if (strcmp(argv[i], "-ct") == 0) {
      i++;
      if (i == argc) {
        Usage();
      } else {
        Count = atoi(argv[i]);
      }
    } else if (strcmp(argv[i], "-noprint") == 0) {
      Print = 0;
    } else if (strcmp(argv[i], "-nostop") == 0) {
      NoStop = 1;
    } else {
      Usage();
    }
    i++;
  }
}

void DoOpenDevices()
{
  char str[80];

  if (( fdADC = open(DevName, ADC_SOFT_TRIGGER)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", DevName);
    exit(2);
  }
}

float volts( int gain, unsigned short value )
{
  float volt;
  
  switch( gain ) {
    case BP_10_00V:
      volt = (20.0/4096.)*(value - 2048);
      break;
    case BP_5_00V:
      volt = (10.0/4096.)*(value - 2048);
      break;
    case UP_10_00V:
      volt = (10.0/4096.)*value;
      break;
    case UP_5_00V:
      volt = (5.0/4096.)*value;
      break;
  }
  return volt;
}

void testDIO()
{
  unsigned short value, bReg;
  char answer[82];
  
  printf("Enter a number in hex (0-f): ");
  scanf("%hx", &value);
  ioctl(fdADC, ADC_SET_DIO, value);
  ioctl(fdADC, ADC_GET_DIO, &bReg);
  printf("Value of DIO is %#hx\n", bReg);
  printf("Continue?");
  scanf("%s", answer);
}

void testADC()
{
  unsigned short value[1024];
  char str[80];
  double mean, sd;
  unsigned short max, min;
  int bytesRead;
  int i;

  while (1) {
    bytesRead = read(fdADC, value, Count);

    for (i = 0; i < Count; i++) {
      printf("The value of %s is %#x or %f V\n ", DevName, value[i], volts(BP_5_00V, value[i]));
    }
    printf("%d samples read\n", bytesRead);
    printf("\n*******  Statistics ******\n");
    mean = 0.0;
    max = 0x0;
    min = 0xffff;
    for ( i = 0; i < Count; i++ ) {
      mean += value[i];
      if ( value[i] > max ) max = value[i];
      if ( value[i] < min ) min = value[i];
    }
    mean /= Count;
    sd = 0.0;
    for ( i = 0; i < Count; i++ ) {
      sd += (value[i] - mean)*(value[i] - mean);
    }
    sd = sqrt( sd / (Count - 1));
    printf("Mean = %f,  Standard deviation = %f.\n", mean, sd);
    printf("Max = %#x,  Min = %#x,  Avg = %#x\n", max, min, (unsigned short) (mean+0.5));

    printf("\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

void Domenu()
{
  int choice;

  while(1) {
    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("    1. Test Digital I/O.\n");
    printf("    2. Test Analog to Digital Converter.\n");
    printf("    3. Test interrupts.\n");
    printf("    4. Exit.\n");
    printf("\nOption: ");
   
    scanf("%d", &choice);
    switch(choice){
      case 1:  
        testDIO();
        break;
      case 2:  
        testADC();
        break;
      case 3:
        ioctl(fdADC, INT_ENABLE, INTERRUPT_ENABLE);
        ioctl(fdADC, SW_INTERRUPT);
        ioctl(fdADC, INT_ENABLE, INTERRUPT_DISABLE);
	break;
      case 4:  
        return;
        break;

      default:
        break;
    }
  }    
}

int main(int argc, char **argv)
{
  DoCommandLine(argc, argv);
  DoOpenDevices();
  Domenu();
  close(fdADC);
  return(1);
}


