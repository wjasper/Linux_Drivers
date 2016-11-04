/*
 * Copyright (C) 2005   Warren Jasper
 * All rights reserved.
 *
 */

/***************************************************************************
 *
 *  test-dio.c
 *
 *  This program is used to test the PCI-DIO24 Digital I/O
 *  Linux loadable module(pci-dio24).
 *
 ***************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include "pci-dio96.h"

/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/dio96/dio0_0A";
char DevNameIO[20];
int  Count    = 1;
int  NoStop   = 0;
int  Mode     = 0;
int  Status;

/***************************************************************************
 *
 *  Show user how the command line is used.
 *
 ***************************************************************************/

void Usage( void )
{
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage: test-dio96 \'options\'\n");
  fprintf(stderr, "Options:\n");
  fprintf(stderr, "    [-dev /dev/dio96_0{A,B,C}]  - Specify device file.\n");
  fprintf(stderr, "    [-ct ##]                     - Number of samples to write\n");
  fprintf(stderr, "    [-nostop]                    - Write forever\n");
  fprintf(stderr, "\n");
  exit(1);
}

/***************************************************************************
 *
 *  Process command line switches
 *
 ***************************************************************************/

void DoCommandLine(int argc, char **argv)
{
int i;

i = 1;

while (i < argc) {
  if (strcmp(argv[i], "-dev") == 0) {
    i++;
    if (i == argc)
      Usage();
    else 
      DevName = argv[i];
  } else if (strcmp(argv[i], "-ct") == 0) {
    i++;
    if (i == argc)
      Usage();
    else 
      Count = atoi(argv[i]);
  } else if (strcmp(argv[i], "-nostop") == 0) {
    NoStop = 1;
  } else {
    Usage();
  }
  i++;
}

}

/***************************************************************************
 *
 *  Main
 *
 ***************************************************************************/

int main(int argc, char **argv)
{

  int fd_0A, fd_0B, fd_0C;
  int fd_1A, fd_1B, fd_1C;
  int fd_2A, fd_2B, fd_2C;
  int fd_3A, fd_3B, fd_3C;  
  unsigned char  bReg;
  unsigned short value;

  DoCommandLine(argc, argv);
	
  printf("Running in mode %d\n", Mode);

  strcpy(DevNameIO, "/dev/dio96/dio0_0A");
  if ((fd_0A = open(DevNameIO, O_RDWR | O_NONBLOCK)) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_0B");
  if ((fd_0B = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_0C");
  if ((fd_0C = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_1A");
  if ((fd_1A = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_1B");
  if ((fd_1B = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_1C");
  if ((fd_1C = open(DevNameIO, O_RDWR | O_NONBLOCK)) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_2A");
  if ((fd_2A = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_2B");
  if ((fd_2B = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_2C");
  if ((fd_2C = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_3A");
  if ((fd_3A = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror("DevNameIO");
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_3B");
  if ((fd_3B = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }
  strcpy(DevNameIO, "/dev/dio96/dio0_3C");
  if ((fd_3C = open(DevNameIO, O_RDWR | O_NONBLOCK )) < 0) {
    perror(DevNameIO);
    printf("error opening device %s\n", DevNameIO);
    exit(2);
  }

  printf("***********************************************************\n");
  printf("* The program writes values on port A and reads them on   *\n");
  printf("* port B (8 bits), and writes on the low nibble of port C *\n");
  printf("* (4 bits) and reads them on the high nibble of port C.   *\n");
  printf("***********************************************************\n\n");

  ioctl(fd_0A, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fd_0B, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fd_0C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fd_0C, DIO_SET_DIRECTION, HIGH_PORT_INPUT);
  ioctl(fd_1A, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fd_1B, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fd_1C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fd_1C, DIO_SET_DIRECTION, HIGH_PORT_INPUT);

  ioctl(fd_2A, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fd_2B, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fd_2C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fd_2C, DIO_SET_DIRECTION, HIGH_PORT_INPUT);
  ioctl(fd_3A, DIO_SET_DIRECTION, PORT_OUTPUT);
  ioctl(fd_3B, DIO_SET_DIRECTION, PORT_INPUT);
  ioctl(fd_3C, DIO_SET_DIRECTION, LOW_PORT_OUTPUT);
  ioctl(fd_3C, DIO_SET_DIRECTION, HIGH_PORT_INPUT);

  do {
      printf("Enter value for port 0A in hex: ");
      scanf("%hx", &value);
      write(fd_0A, &value, 1);
      read(fd_0A, &bReg, 1);
      printf("Port 0A value = %#x\n", bReg);

      read(fd_0B, &bReg, 1);
      printf("Port 0B value = %#hx\n", bReg);

      printf("Enter value for port 0C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_0C, &value, 1);
      read(fd_0C, &bReg, 1);
      printf("Port 0C value entered = %#hx\n", bReg & 0xf);
      bReg >>= 4;
      printf("Port 0C value = %#hx\n", bReg);

      printf("Enter value for port 1A in hex: ");
      scanf("%hx", &value);
      write(fd_1A, &value, 1);
      read(fd_1A, &bReg, 1);
      printf("Port 1A value = %#x\n", bReg);

      read(fd_1B, &bReg, 1);
      printf("Port 1B value = %#hx\n", bReg);

      printf("Enter value for port 1C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_1C, &value, 1);
      read(fd_1C, &bReg, 1);
      printf("Port 1C value entered = %#hx\n", bReg & 0xf);
      bReg >>= 4;
      printf("Port 1C value = %#hx\n", bReg);

      printf("Enter value for port 2A in hex: ");
      scanf("%hx", &value);
      write(fd_2A, &value, 1);
      read(fd_2A, &bReg, 1);
      printf("Port 2A value = %#x\n", bReg);

      read(fd_2B, &bReg, 1);
      printf("Port 2B value = %#hx\n", bReg);

      printf("Enter value for port 2C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_2C, &value, 1);
      read(fd_2C, &bReg, 1);
      printf("Port 2C value entered = %#hx\n", bReg & 0xf);
      bReg >>= 4;
      printf("Port 2C value = %#hx\n", bReg);

      printf("Enter value for port 3A in hex: ");
      scanf("%hx", &value);
      write(fd_3A, &value, 1);
      read(fd_3A, &bReg, 1);
      printf("Port 3A value = %#x\n", bReg);

      read(fd_3B, &bReg, 1);
      printf("Port 3B value = %#hx\n", bReg);

      printf("Enter value for port 3C in hex: ");
      scanf("%hx", &value);
      value &= 0x0f;          /* mask off high nibble */
      write(fd_3C, &value, 1);
      read(fd_3C, &bReg, 1);
      printf("Port 3C value entered = %#hx\n", bReg & 0xf);
      bReg >>= 4;
      printf("Port 3C value = %#hx\n", bReg);
} while (NoStop);

  close(fd_0A);
  close(fd_0B);
  close(fd_0C);
  close(fd_1A);
  close(fd_1B);
  close(fd_1C);
  close(fd_2A);
  close(fd_2B);
  close(fd_2C);
  close(fd_3A);
  close(fd_3B);
  close(fd_3C);

  return 0;
}

	
	



