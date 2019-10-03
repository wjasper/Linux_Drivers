/*
 * Copyright (C) 1997  Warren Jasper
 * All rights reserved.
 */


/***************************************************************************
 *
 *  test-das1602.c
 *
 *  This program is used to test the PCI-DAS1602/16 Analog to Digital
 *  Linux loadable module.
 *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "pci-das1602-16.h"


/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/das1602-16/ad0_0";
int  Gain     = UP_10_00V;
int  ADC_Mode     = ADC_SOFT_CONVERT;
int  DAC_Mode     = DAC_SOFT_CONVERT;
int  Count    = 1;
int  NoStop   = 0;
int  Print    = 1;
int  freq_A2D = 5000;
int  freq_D2A = 1000;
int  Status;

int fdDAC0, fdDAC1;          /* D/A file descriptors */
int fdADC;                   /* A/D file descriptors */
int fdDIOA, fdDIOB, fdDIOC;  /* DIO file descriptors */

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: adcread \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr,"   [-dev /dev/das1602-16/adc0_#]  - Specify device file.\n");
  fprintf(stderr,"   [-range (0-8)]                 - Voltage input range (see manual)\n");
  fprintf(stderr,"   [-ct ##]                       - Number of samples to read\n");
  fprintf(stderr,"   [-soft]                        - Use software triggers\n");
  fprintf(stderr,"   [-extern]                      - Use extnernal clock triggers\n");
  fprintf(stderr,"   [-pacer]                       - Use pacer clock triggers\n");
  fprintf(stderr,"   [-noprint]                     - Don't print samples\n");
  fprintf(stderr,"   [-nostop]                      - Sample forever\n");
  fprintf(stderr,"   [-A2Dfreq ##]                  - Sample A/D Frequency used with pacer triggers\n");
  fprintf(stderr,"   [-D2Afreq ##]                  - Sample D/A Frequency used with pacer triggers\n");
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
      DAC_Mode = DAC_PACER_CLOCK;
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
    } else if (strcmp(argv[i], "-D2Afreq") == 0) {
      i++;
      if (i == argc) {
        Usage();
      } else {
        freq_D2A = atoi(argv[i]);
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

  if (( fdADC = open(DevName, ADC_Mode)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", DevName);
    exit(2);
  }

  strcpy(str, "/dev/das1602-16/da0_0");
  if (( fdDAC0 = open(str, DAC_Mode)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/das1602-16/da0_1");
  if (( fdDAC1 = open(str, DAC_SOFT_CONVERT)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/das1602-16/dio0_0A");
  if (( fdDIOA = open(str, O_RDWR)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/das1602-16/dio0_0B");
  if (( fdDIOB = open(str, O_RDWR)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  strcpy(str, "/dev/das1602-16/dio0_0C");
  if (( fdDIOC = open(str, O_RDWR)) < 0 ) {
    perror(str);
    printf("error opening device %s\n", str);
    exit(2);
  }

  ioctl(fdADC, ADC_SET_GAINS, UP_10_00V);
  
  ioctl(fdDAC0, DAC_SET_GAINS, UP_10_00V);
  ioctl(fdDAC1, DAC_SET_GAINS, BP_10_00V);

}

void testDIO()
{
  unsigned short value;
  BYTE bReg;
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
  WORD value;
  char str[80];
  WORD buf[MAX_COUNT];
  int i;

  if ( DAC_Mode == DAC_SOFT_CONVERT ) {
    while (1) {
      printf("Enter value for DAC0 [0-0xffff]: ");
      scanf("%hx", &value);
      write(fdDAC0, &value, 1);
      printf("Enter value for DAC1[0-0xffff]: ");
      scanf("%hx", &value);
      write(fdDAC1, &value, 1);
      printf("\n Continue? ");
      scanf("%s", str);
      if (str[0] == 'n' || str[0] == 'N') return;
    }
  }
  if ( DAC_Mode == DAC_PACER_CLOCK ) {
    ioctl(fdDAC0, DAC_RECYCLE, 1);
    for ( i = 0; i < MAX_COUNT; i++ ) {
      buf[i] = i*4;
    }
    write(fdDAC0, buf, MAX_COUNT);
    sleep(20);
    ioctl(fdDAC0, DAC_RECYCLE, 0);
  }
}

void testADC()
{
  WORD value[16384];
  char str[80];
  int i;
  int bytesRead;
  double mean, sd;
  WORD max, min;

  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);

  while (1) {
    bytesRead = read(fdADC, value, Count);

    if ( bytesRead != Count ) {
      printf("testADC: Error on read() \n");
    }

    for ( i = 0; i < Count; i++ ) {
      printf("Value number %d is %#x\n", i+1, value[i]);
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
    printf("Max = %#x,  Min = %#x,  Avg = %#x\n", max, min, (WORD) (mean+0.5));

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

void SetTriggers()
{
  int choice;

  system("/usr/bin/clear");
  printf("Select from the following choices:\n");
  printf("  1.    GATE_NEG_HYS.\n");
  printf("  2.    GATE_POS_HYS.\n");
  printf("  3.    GATE_ABOVE.\n");
  printf("  4.    GATE_BELOW.\n");
  printf("  5.    TRIG_ABOVE.\n");
  printf("  6.    TRIG_BELOW.\n");
  printf("  7.    GATE_OUT_WINDOW\n");
  printf("  8.    GATE_IN_WINDOW\n");
  printf("  9.    GATE_HIGH\n");
  printf(" 10.    GATE_LOW\n");
  printf(" 11.    TRIG_POS_EDGE\n");
  printf(" 12.    TRIG_NEG_EDGE\n");
  printf("\nOption: ");
  scanf("%d", &choice);
  switch(choice){
    case 1:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_NEG_HYS);
      break;
    case 2:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_POS_HYS);
      break;
    case 3:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_ABOVE);
      break;
    case 4:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_BELOW);
      break;
    case 5:
      ioctl(fdADC, ADC_SET_TRIGGER, TRIG_ABOVE);
      break;
    case 6:
      ioctl(fdADC, ADC_SET_TRIGGER, TRIG_BELOW);
      break;
    case 7:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_OUT_WINDOW);
      break;
    case 8:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_IN_WINDOW);
      break;
    case 9:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_HIGH);
      break;
    case 10:
      ioctl(fdADC, ADC_SET_TRIGGER, GATE_LOW);
      break;
    case 11:
      ioctl(fdADC, ADC_SET_TRIGGER, TRIG_POS_EDGE);
      break;
    case 12:
      ioctl(fdADC, ADC_SET_TRIGGER, TRIG_NEG_EDGE);
      break;
    default:
      printf("Illegal option.\n");
  }
}
   
void ChangeDACGains()
{
    int choice;
    long gain;

    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("  1.   Bipolar +/- 10.00V.\n");
    printf("  2.   Bipolar +/-  5.00V.\n");
    printf("  3.   Unipolar     0-10V.\n");
    printf("  4.   Unipolar     0-5V.\n");
    printf("\nOption: ");
    scanf("%d", &choice);
    switch(choice){
    case 1:
      ioctl(fdDAC0, DAC_SET_GAINS, BP_10_00V);
      break;
    case 2:
      ioctl(fdDAC0, DAC_SET_GAINS, BP_5_00V);
      break;
    case 3:
      ioctl(fdDAC0, DAC_SET_GAINS, UP_10_00V);
      break;
    case 4:
      ioctl(fdDAC0, DAC_SET_GAINS, UP_5_00V);
      break;
    default:
      printf("Illegal option.\n");
    }
   ioctl(fdDAC0, DAC_GET_GAINS, &gain);
   printf("DAC0: gain set to %ld\n", gain);
   sleep(3);
}

void SetADPacer()
{
  printf("***********************************************************\n");
  printf("* This program sets the A/D Pacer Frequency which can be  *\n");
  printf("* seen with a scope on Pin 95                             *\n");
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

void SetDAPacer()
{
  printf("***********************************************************\n");
  printf("* This program sets the D/A Pacer Frequency which can be  *\n");
  printf("* seen with a scope on Pin 96                             *\n");
  printf("***********************************************************\n\n");

  printf("Input the desired DA Pacer Frequency (Hz): ");
  scanf("%d", &freq_D2A);
  ioctl(fdDAC0, DAC_STOP_PACER, 0);
  ioctl(fdDAC0, DAC_SET_PACER_FREQ, freq_D2A);
  ioctl(fdDAC0, DAC_GET_PACER_FREQ, &freq_D2A);
  printf("\nThe DA Pacer Frequency is set to %d\n", freq_D2A);
  sleep(3);
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
    printf("    4. Change D/A Gains.\n");
    printf("    5. Change A/D Gains.\n");
    printf("    6. Set A/D Internal Pacer.\n");
    printf("    7. Set D/A Internal Pacer.\n");
    printf("    8. Set ADC Triggers.\n");
    printf("    9. Exit.\n");
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
        ChangeDACGains();
        break;
      case 5:
        ChangeADCGains();
        break;
      case 6:
        SetADPacer();
        break;
      case 7:
        SetDAPacer();
        break;
      case 8:  
        SetTriggers();
        break;
      case 9:  
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

