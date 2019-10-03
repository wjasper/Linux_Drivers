/*
 * Copyright (C) 2002  Warren Jasper <wjasper@ncsu.edu>
 * All rights reserved.
 *
 */


/***************************************************************************
 *
 *  test-das4020.c
 *
 *  This program is used to test the PCI-DAS4020/12 Analog to Digital
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
#include "pci-das4020.h"
#include <sys/mman.h>

/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/


int  ADC_Mode = ADC_SOFT_CONVERSION;
int  Count    = 1;
int  Gain     = BP_10_00V;
int  Board    = 0;
int  Channel  = 0;
int  Print    = 1;
int  NoStop   = 0;
int  Status;
int  freq_A2D = 100000;

int fdDAC0, fdDAC1;          		    /* D/A file descriptors */
int fdADC, fdADC0, fdADC1, fdADC2, fdADC3;  /* A/D file descriptors */
int fdDIOA, fdDIOB, fdDIOC;  		    /* DIO file descriptors */

float full_scale[4] = {5.0, 5.0, 5.0, 5.0};

void Usage( void )
{
    fprintf(stderr, "\n\
Usage: adcread \'options\'\n\
Options:\n\
              [-board ##]                       - Specify board # [0-NUM_BOARDS-1].\n\
              [-channel ##]                     - Specify channel # [0-3].\n\
              [-range (0-8)]                    - Voltage input range (see manual)\n\
              [-ct ##]                          - Number of samples to read\n\
              [-soft]                           - Use software triggers\n\
              [-dma]                            - Use dma and pacer clock triggers\n\
              [-noprint]                        - Don't print samples\n\
              [-nostop]                         - Sample forever\n\
              [-A2Dfreq ##]                     - Sample A/D Frequency used with pacer triggers\n\
              [-D2Afreq ##]                     - Sample D/A Frequency used with pacer triggers\n\n\
");
    exit(1);
}

void DoCommandLine(int argc, char **argv)
{
  int i = 1;

  while (i < argc) {
    if (strcmp(argv[i], "-range") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        Gain = atoi(argv[i]);
    } else if (strcmp(argv[i], "-ct") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        Count = atoi(argv[i]);
    } else if (strcmp(argv[i], "-board") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        Board = atoi(argv[i]);
    } else if (strcmp(argv[i], "-channel") == 0) {
      i++;
      if (i == argc)
        Usage();
      else 
        Channel = atoi(argv[i]);
    } else if (strcmp(argv[i], "-single") == 0) {
      ADC_Mode = ADC_SOFT_CONVERSION;
    } else if (strcmp(argv[i], "-dma") == 0) {
      ADC_Mode = ADC_DMA_CONVERSION;
    } else if (strcmp(argv[i], "-noprint") == 0) {
      Print = 0;
    } else if (strcmp(argv[i], "-nostop") == 0) {
      NoStop = 1;
    } else if (strcmp(argv[i], "-A2Dfreq") == 0) {
      i++;
      if (i == argc)
        Usage();
      else
        freq_A2D = atoi(argv[i]);
    } else {
      Usage();
    }
    i++;
 }
}

void DoOpenDevices()
{
  char str[80];
  char dio_chan;
  int dac_chan, adc_chan;
  int *adc_fds[] = { &fdADC0, &fdADC1, &fdADC2, &fdADC3 };
  int *dac_fds[] = { &fdDAC0, &fdDAC1 };
  int *dio_fds[] = { &fdDIOA, &fdDIOB, &fdDIOC};

  for( adc_chan = 0; adc_chan <= 3; adc_chan++ )
  {
    sprintf(str, "/dev/das4020-12/ad%d_%d", Board, adc_chan );
    if (( *adc_fds[adc_chan] = open(str, ADC_Mode)) < 0 ) {
      perror(str);
      printf("error opening device %s\n", str);
      exit(2);
    }
  }
  /* test for combining 2 output to one channel */
  //  ioctl(fdADC0, ADC_SET_CHAN_LOW, 0);
  //  ioctl(fdADC0, ADC_SET_CHAN_HIGH,1;
  ioctl(fdADC0, ADC_SET_GAINS, BP_5_00V);
  		
  for( dio_chan = 'A'; dio_chan <= 'C'; dio_chan++ ) {
    sprintf( str, "/dev/das4020-12/dio%d_0%c", Board, dio_chan );
    if (( *dio_fds[dio_chan - 'A'] = open(str, O_RDWR)) < 0 ) {
      perror(str);
      printf("error opening device %s\n", str);
      exit(2);
    }
  }

  for( dac_chan = 0; dac_chan <= 1; dac_chan++ )
  {
    sprintf( str, "/dev/das4020-12/da%d_%d", Board, dac_chan );
    if (( *dac_fds[dac_chan] = open(str, O_WRONLY)) < 0 ) {
      perror(str);
      printf("error opening device %s\n", str);
      exit(2);
    }
  }

  if( Channel < 0 || Channel > 3 ) {
      printf("Specified ADC channel of %d falls outside of range [0-3]\n", Channel);
      exit(2);
  }
  fdADC = *adc_fds[ Channel ];
}

void testDAC()
{
  unsigned short value;
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

void testDIO()
{
  unsigned short value;
  unsigned char bReg;
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

void testADC()
{
  unsigned short value[1024*64];
  char str[80];
  int i;
  int bytesRead;
  double mean, sd;
  unsigned short max, min;
  float volt;
  unsigned short *readBuff;
  unsigned int toggle = 0x0;

  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);
  ioctl(fdADC, ADC_GET_HALF_FIFO_SIZE, &i);
  printf("The size of half the FIFO in bytes = %#x\n", i);
  ioctl(fdADC, ADC_GET_DMA_BUF_SIZE, &i);
  printf("The size of the DMA buffer = %#x\n", i);

  if ( ADC_Mode == ADC_SOFT_CONVERSION ) {
    while (1) {
      toggle ^= 0x1;
      ioctl(fdADC, ADC_PSC_ENB, toggle);
      bytesRead = read(fdADC, value, Count);

      if ( bytesRead != Count ) {
	printf("testADC: Error on read() \n");
	printf("bytesRead = %d, and specified Count = %d\n",bytesRead,Count);
      }

      if ( Print ) {
    	for ( i = 0; i < Count; i++ ) {
  	  volt = full_scale[Channel]*value[i]/2048. - full_scale[Channel];
  	  printf("Value number %d is %#x, %d = %f Volts\n", i+1, value[i], value[i], volt);
       	}
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

      printf("\n\n Continue? ");
      scanf("%s", str);
      if (str[0] == 'n' || str[0] == 'N') return;

    }
  } else {
    if((readBuff= mmap(0, Count*2, PROT_READ, MAP_PRIVATE, fdADC, 0*getpagesize()))
       == (unsigned short *) MAP_FAILED) {
      printf("Test Failed: Mmap call failed \n");	
      sleep(3);
      return;
    } else{
      printf("Test Passed: Succesfully mmaped %d bytes\n", Count*2); 
    }

    /* In the following read calls, the argument value will be ignored */
    /* Since we DMA to stuff over to the address held by readBuff     */

    while(1) {
      bytesRead = read(fdADC, value, Count);

      if ( bytesRead != Count ) {
	printf("testADC: Error on read() \n");
	printf("bytesRead= %d, and specified Count = %d", bytesRead,Count);
      }

      if ( Print ) {
	for ( i = 0; i < Count; i++ ) {
	  volt = full_scale[Channel]*readBuff[i]/2048. - full_scale[Channel];
	  printf("Value number %d is %#hx, %hd = %f Volts\n", i+1, readBuff[i], readBuff[i], volt);
	}
      }

      printf("%d samples read\n", bytesRead);
      printf("\n*******  Statistics ******\n");
      mean = 0.0;
      max = 0x0;
      min = 0xffff;

      for ( i = 0; i < Count; i++ ) {
	mean += readBuff[i];
	if ( readBuff[i] > max ) max = readBuff[i];
	if ( readBuff[i] < min ) min = readBuff[i];
      }
      mean /= Count;
      sd = 0.0;
      for ( i = 0; i < Count; i++ ) {
	sd += (readBuff[i] - mean)*(readBuff[i] - mean);
      }
      sd = sqrt( sd / (Count - 1));
      printf("Mean = %f,  Standard deviation = %f.\n", mean, sd);
      printf("Max = %#x,  Min = %#x,  Avg = %#x\n", max, min, (unsigned short) (mean+0.5));
      printf("\n\n Continue? ");
      scanf("%s", str);
      if (str[0] == 'n' || str[0] == 'N') {
	if (munmap(readBuff, 2*Count) == -1 ) {
	  printf("munmap failure.\n");
	  sleep(3);
	}
	return;
      }
    }
  }
}

void ChangeDACGains()
{
    int choice;
    long gain;
    int chan;	
    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("  1.   DAC0: Bipolar +/- 10.00V.\n");
    printf("  2.   DAC0: Bipolar +/-  5.00V.\n");
    printf("  3.   DAC1: Bipolar +/- 10.00V.\n");
    printf("  4.   DAC1: Bipolar +/-  5.00V.\n");
    printf("\nOption: ");
    scanf("%d", &choice);
    switch(choice){
    case 1:
      ioctl(fdDAC0, DAC_SET_GAINS, BP_10_00V);
      ioctl(fdDAC0, DAC_GET_GAINS, &gain);
      chan=0;
      break;
    case 2:
      ioctl(fdDAC0, DAC_SET_GAINS, BP_5_00V);
      ioctl(fdDAC0, DAC_GET_GAINS, &gain);
      chan=0;
      break;
    case 3:
      ioctl(fdDAC1, DAC_SET_GAINS, BP_10_00V);
      ioctl(fdDAC1, DAC_GET_GAINS, &gain);
      chan=1;
      break;
    case 4: 
      ioctl(fdDAC1, DAC_SET_GAINS, BP_5_00V);
      ioctl(fdDAC1, DAC_GET_GAINS, &gain);
      chan=1;
      break;
    default:
      printf("Illegal option.\n");
    }
   
   printf("DAC%d: gain set to %ld\n",chan,gain);
   sleep(3);
}


void ChangeADCGains()
{
    int choice;

    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("  1.   Chan0 Bipolar +/-  5.00V.\n");
    printf("  2.   Chan0 Bipolar +/-  1.00V.\n");
    printf("  3.   Chan1 Bipolar +/-  5.00V.\n");
    printf("  4.   Chan1 Bipolar +/-  1.00V.\n");
    printf("  5.   Chan2 Bipolar +/-  5.00V.\n");
    printf("  6.   Chan2 Bipolar +/-  1.00V.\n");
    printf("  7.   Chan3 Bipolar +/-  5.00V.\n");
    printf("  8.   Chan3 Bipolar +/-  1.00V.\n");
    printf("\nOption: ");
    scanf("%d", &choice);
    switch(choice){
    case 1:
      ioctl(fdADC0, ADC_SET_GAINS, BP_5_00V);
      full_scale[0] = 5.0;
      break;
    case 2:
      ioctl(fdADC0, ADC_SET_GAINS, BP_1_00V);
      full_scale[0] = 1.0;
      break;
    case 3:
      ioctl(fdADC1, ADC_SET_GAINS, BP_5_00V);
      full_scale[1] = 5.0;
      break;
    case 4:
      ioctl(fdADC1, ADC_SET_GAINS, BP_1_00V);
      full_scale[1] = 1.0;
      break;
    case 5:
      ioctl(fdADC2, ADC_SET_GAINS, BP_5_00V);
      full_scale[2] = 5.0;
      break;
    case 6:
      ioctl(fdADC2, ADC_SET_GAINS, BP_1_00V);
      full_scale[2] = 1.0;
      break;
    case 7:
      ioctl(fdADC3, ADC_SET_GAINS, BP_5_00V);
      full_scale[3] = 5.0;
      break;
    case 8:
      ioctl(fdADC3, ADC_SET_GAINS, BP_1_00V);
      full_scale[4] = 1.0;
      break;
    default:
      printf("Illegal option.\n");
    }
}

void SetADChannel()
{
  printf("Input the desired A/D Channel [0-3]: ");
  scanf("%d", &Channel);
  switch (Channel) {
    case 0:
      fdADC = fdADC0;
      break;
    case 1:
      fdADC = fdADC1;
      break;
    case 2:
      fdADC = fdADC2;
      break;
    case 3:
      fdADC = fdADC3;
      break;
  }
}

void SetADPacer()
{
  printf("***********************************************************\n");
  printf("* This program sets the A/D Pacer Frequency.              *\n");
  printf("***********************************************************\n\n");
  
  printf("Input the desired AD Pacer Frequency (Hz): ");
  scanf("%d", &freq_A2D);
  ioctl(fdADC, ADC_SET_PACER_FREQ, freq_A2D);

  printf("\nThe AD Pacer Frequency is set to %d Hz.\n", freq_A2D);
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
    printf("    5. Change A/D Gains/Offsets.\n");
    printf("    6. Set A/D Internal Pacer.\n");
    printf("    7. Select A/D Channel.\n");
    printf("    8. Exit.\n");
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
        SetADChannel();
	break;
      case 8:  
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
  close(fdADC0);
  close(fdADC1);
  close(fdADC2);
  close(fdADC3);
  close(fdDIOA);
  close(fdDIOB);
  close(fdDIOC);
  close(fdDAC0);
  close(fdDAC1);
  
  return(1);
}
