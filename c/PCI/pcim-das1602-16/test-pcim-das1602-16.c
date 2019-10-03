/*
 * Copyright (C) 2011  Warren Jasper
 * All rights reserved.
 */


/***************************************************************************
 *
 *  test-pcim-das1602-16.c
 *
 *  This program is used to test the PCIM-DAS1602/16 Analog to Digital
 *  Linux loadable module.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include "pcim-das1602-16.h"

float value2Volts( int value );


/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/pcim-das1602-16/ad0_0";
int  Gain     = UP_10_00V;
int  ADC_Mode = ADC_SOFT_CONVERT;
int  ADC_Type = ADC_ACQ_NORMAL;
int  Count    = 1;
int  NoStop   = 0;
int  Print    = 1;
int  freq_A2D = 1000;
int  Status;

int fdDAC0, fdDAC1;          /* D/A file descriptors */
int fdADC;                   /* A/D file descriptors */
int fdDIOA, fdDIOB, fdDIOC;  /* DIO file descriptors */

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: adcread \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr,"   [-dev /dev/pcim-das1602-16/adc0_#]  - Specify device file.\n");
  fprintf(stderr,"   [-range (0-8)]                      - Voltage input range (see manual)\n");
  fprintf(stderr,"   [-ct ##]                            - Number of samples to read\n");
  fprintf(stderr,"   [-soft]                             - Use software triggers\n");
  fprintf(stderr,"   [-extern]                           - Use extnernal clock triggers\n");
  fprintf(stderr,"   [-pacer]                            - Use pacer clock triggers\n");
  fprintf(stderr,"   [-ringbuf]                          - Use continuous acquisition\n");
  fprintf(stderr,"   [-noprint]                          - Don't print samples\n");
  fprintf(stderr,"   [-nostop]                           - Sample forever\n");
  fprintf(stderr,"   [-A2Dfreq ##]                       - Sample A/D Frequency used with pacer triggers\n");
  fprintf(stderr,"\n");
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
    } else if (strcmp(argv[i], "-range") == 0) {
      i++;
      if (i == argc) {
        Usage();
      } else {
        Gain = atoi(argv[i]);
      }
    } else if (strcmp(argv[i], "-ct") == 0) {
      i++;
      if (i == argc) {
        Usage();
      } else {
        Count = atoi(argv[i]);
      }
    } else if (strcmp(argv[i], "-soft") == 0) {
      ADC_Mode = ADC_SOFT_CONVERT;
    } else if (strcmp(argv[i], "-pacer") == 0) {
      ADC_Mode = ADC_PACER_CLOCK;
    } else if (strcmp(argv[i], "-ringbuf") == 0) {
      ADC_Type = ADC_ACQ_CONTINUOUS;
    } else if (strcmp(argv[i], "-extern") == 0) {
      ADC_Mode = ADC_EXTERNAL_PACER_FALLING;
    } else if (strcmp(argv[i], "-noprint") == 0) {
      Print = 0;
    } else if (strcmp(argv[i], "-nostop") == 0) {
      NoStop = 1;
    } else if (strcmp(argv[i], "-A2Dfreq") == 0) {
      i++;
      if (i == argc) {
        Usage();
      } else {
        freq_A2D = atoi(argv[i]);
      }
    } else {
      Usage();
    }
    i++;
  }
}

void DoOpenDevices()
{
  char str[80];
  long arg;

  if ((fdADC = open(DevName, O_RDWR)) < 0) {
    perror(DevName);
    printf("error opening device %s\n", DevName);
    exit(2);
  }
  ioctl(fdADC, ADC_SET_MODE, ADC_Mode);
  ioctl(fdADC, ADC_SET_ACQ_TYPE, ADC_Type);
  
  strcpy(str, "/dev/pcim-das1602-16/da0_0");
  if ((fdDAC0 = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/pcim-das1602-16/da0_1");
  if ((fdDAC1 = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/pcim-das1602-16/dio0_A");
  if ((fdDIOA = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/pcim-das1602-16/dio0_B");
  if ((fdDIOB = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/pcim-das1602-16/dio0_C");
  if ((fdDIOC = open(str, O_RDWR)) < 0) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  ioctl(fdADC, ADC_GET_POLARITY, &arg);
  if (arg == 'U') {
    printf("The Analog Input Polarity Switch is set to Unipolar.\n");
  } else {
    printf("The Analog Input Polarity Switch is set to Bipolar.\n");
  }

  ioctl(fdADC, ADC_GET_FRONT_END, &arg);
  if (arg == 16) {
    printf("The Analog Input Mode Switch is set to 16 single-ended channels.\n");
  } else {
    printf("The Analog Input Mode Switch is set to 8 differential channels.\n");
  }

  ioctl(fdADC, ADC_GET_CLK_FREQ, &arg);
  printf("The Pacer Clock jumper is set to %ld Hz.\n", arg);
}

void testDIO()
{
  unsigned short value;
  uint8_t bReg;
  char str[80];

  printf("***********************************************************\n");
  printf("* The program writes values on port A and reads them on   *\n");
  printf("* port B (8 bits), and writes on the low nibble of port C *\n");
  printf("* (4 bits) and reads them on the high nibble of port C.   *\n");
  printf("***********************************************************\n\n");

  ioctl(fdDIOA, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fdDIOB, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fdDIOC, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fdDIOC, DIO_SET_DIRECTION, HIGH_PORT_INPUT);

  while (1) {
      printf("Enter value for port A in hex: ");
      scanf("%hx", &value);
      write(fdDIOA, &value, 1);
      read(fdDIOB, &bReg, 1);
      printf("Port B value = %#hx\n", (short int) bReg);
      printf("Enter value for port C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fdDIOC, &value, 1);
      read(fdDIOC, &bReg, 1);
      bReg >>= 4;
      printf("Port C value = %#hx\n", (short int) bReg);

      printf("\n Continue? ");
      scanf("%s", str);
      if (str[0] == 'n' || str[0] == 'N') return;
  }
}

#define MAX_COUNT 16384
void testDAC()
{
  uint16_t value;
  char str[80];

  while (1) {
    printf("Enter value for DAC0 [0-0xfff]: ");
    scanf("%hx", &value);
    write(fdDAC0, &value, 1);
    printf("Enter value for DAC1[0-0xfff]: ");
    scanf("%hx", &value);
    write(fdDAC1, &value, 1);
    printf("\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

void testADC()
{
  uint16_t value[16384];
  char str[80];
  int i;
  int bytesRead;
  double mean, sd;
  uint16_t max, min;
  uint8_t lowChan, hiChan;
  uint64_t diff;
  double accum;
  struct timespec start, end;
  int timerok = 1;

  printf("Enter low channel: ");
  scanf("%hhd", &lowChan);
  printf("Enter hight channel: ");
  scanf("%hhd", &hiChan);
  if (Count == 1) {
    printf("Enter number of samples [1-16834]: ");
    scanf("%d", &Count);
  }
  
  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);
  ioctl(fdADC, ADC_SET_MUX_LOW, lowChan);
  ioctl(fdADC, ADC_SET_MUX_HIGH, hiChan);
  ioctl(fdADC, ADC_SET_ACQ_TYPE, ADC_ACQ_CONTINUOUS);
  
  while (1) {
    for (i = 0; i < 16384; i++) {
      value[i] = 0xbeef;
    }

    if (clock_gettime(CLOCK_REALTIME, &start) == -1) timerok = 0;
    bytesRead = read(fdADC, value, Count);
    if (clock_gettime(CLOCK_REALTIME, &end) == -1) timerok = 0;
    diff = 1000000000L * (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec);
    accum = end.tv_nsec - start.tv_nsec;
    accum = accum / 1E9;
    accum += (end.tv_sec - start.tv_sec);
    
    if (bytesRead != Count) {
      printf("testADC: Error on read() \n");
    }

    for (i = 0; i < Count; i++) {
      printf("Value number %d is %#6x volts = %f\n", i+1, value[i], value2Volts((int) value[i]));
    }

    printf("elasped time = %llu nsec\n", (long long unsigned int) diff);
    printf("elapsed time = %lf sec -- ok:%d\n", accum, timerok);
    printf("%d samples read\n", bytesRead);
    printf("\n*******  Statistics ******\n");
    mean = 0.0;
    max = 0x0;
    min = 0xffff;

    for ( i = 0; i < Count; i++ ) {
      mean += value[i];
      if (value[i] > max) max = value[i];
      if (value[i] < min) min = value[i];
    }
    mean /= Count;
    sd = 0.0;
    for ( i = 0; i < Count; i++ ) {
      sd += (value[i] - mean)*(value[i] - mean);
    }
    sd = sqrt(sd / (Count - 1));
    printf("Mean = %f,  Standard deviation = %f.\n", mean, sd);
    printf("Max = %#x,  Min = %#x,  Avg = %#x\n", max, min, (uint16_t) (mean+0.5));

    printf("\n\n Continue? ");
    scanf("%s", str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

void ChangeADCGains()
{
    int choice;

    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("  1.   Bipolar +/- 10.00V.\n");
    printf("  2.   Bipolar +/-  5.00V.\n");
    printf("  3.   Bipolar +/-  2.50V.\n");
    printf("  4.   Bipolar +/-  1.25V.\n");
    printf("  5.   Unipolar     0-10V.\n");
    printf("  6.   Unipolar     0-5V.\n");
    printf("  7.   Unipolar     0-2.5V.\n");
    printf("  8.   Unipolar     0-1.5V.\n");
    printf("\nOption: ");
    scanf("%d", &choice);
    switch(choice){
    case 1:
      ioctl(fdADC, ADC_SET_GAINS, BP_10_00V);
      break;
    case 2:
      ioctl(fdADC, ADC_SET_GAINS, BP_5_00V);
      break;
    case 3:
      ioctl(fdADC, ADC_SET_GAINS, BP_2_50V);
      break;
    case 4:
      ioctl(fdADC, ADC_SET_GAINS, BP_1_25V);
      break;
    case 5:
      ioctl(fdADC, ADC_SET_GAINS, UP_10_00V);
      break;
    case 6:
      ioctl(fdADC, ADC_SET_GAINS, UP_5_00V);
      break;
    case 7:
      ioctl(fdADC, ADC_SET_GAINS, UP_2_50V);
      break;
    case 8:
      ioctl(fdADC, ADC_SET_GAINS, UP_1_25V);
      break;
    default:
      printf("Illegal option.\n");
    }
}

void SetADPacer()
{
  printf("***********************************************************\n");
  printf("* This program sets the A/D Pacer Frequency which can be  *\n");
  printf("* seen with a scope on Pin 25.                            *\n");
  printf("***********************************************************\n\n");
  
  printf("Input the desired AD Pacer Frequency (Hz): ");
  scanf("%d", &freq_A2D);
  ioctl(fdADC, ADC_STOP_PACER, 0);
  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);
  ioctl(fdADC, ADC_GET_PACER_FREQ, &freq_A2D);
  ioctl(fdADC, ADC_START_PACER, 0);
  printf("\nThe AD Pacer Frequency is set to %d\n", freq_A2D);
  sleep(3);
}

float value2Volts( int value )
{
  long  gain;

  ioctl(fdADC, ADC_GET_GAINS, &gain);
  switch (gain) {
  case BP_10_00V:
    return (value - 0x7fff) * 10.0 / 0x7fff;
    break;
  case BP_5_00V:
    return (value - 0x7fff) * 5.0 / 0x7fff;
    break;
  case BP_2_50V:
    return (value - 0x7fff) * 2.5 / 0x7fff;
    break;
  case BP_1_25V:
    return (value - 0x7fff) * 1.25 / 0x7fff;
    break;
  case UP_10_00V:
    return (value * 10.0 / 0xffff);
    break;
  case UP_5_00V:
    return (value * 5.0 / 0xffff);
    break;
  case UP_2_50V:
    return (value * 2.5 / 0xffff);
    break;
  case UP_1_25V:
    return (value * 1.25 / 0xffff);
    break;
  }
  return -1.0;
}

void Domenu()
{
  int choice;

  while(1) {
    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("    1. Test Digital I/O.\n");
    printf("    2. Test Digital to Analog Converter.\n");
    printf("    3. Test Analog to Digital Converter.\n");
    printf("    4. Change A/D Gains.\n");
    printf("    5. Set A/D Internal Pacer.\n");
    printf("    6. Exit.\n");
    printf("\nOption: ");
   
    scanf("%d", &choice);
    switch(choice){
      case 1:  
        testDIO();
        break;
      case 2:  
        testDAC();
        break;
      case 3:  
        testADC();
        break;
      case 4:
        ChangeADCGains();
        break;
      case 5:
        SetADPacer();
        break;
      case 6:  
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
  close(fdDIOA);
  close(fdDIOB);
  close(fdDIOC);
  close(fdDAC0);
  close(fdDAC1);
  close(fdADC);
  return(1);
}

