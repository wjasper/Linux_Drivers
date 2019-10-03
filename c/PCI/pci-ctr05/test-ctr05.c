/***************************************************************************
 Copyright (C) 2003-2007  Warren J. Jasper <wjasper@ncsu.edu>
 All rights reserved.

 This program, PCI-CTR05, is free software; you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation; either version 2 of the
 License, or (at your option) any later version, provided that this
 copyright notice is preserved on all copies.

 ANY RIGHTS GRANTED HEREUNDER ARE GRANTED WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND FURTHER,
 THERE SHALL BE NO WARRANTY AS TO CONFORMITY WITH ANY USER MANUALS OR
 OTHER LITERATURE PROVIDED WITH SOFTWARE OR THAM MY BE ISSUED FROM TIME
 TO TIME. IT IS PROVIDED SOLELY "AS IS".

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
***************************************************************************/

/***************************************************************************
 *
 *  test-ctr05.c
 *
 *  This program is used to test the PCI-CTR05 Counter Board
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
#include "pci-ctr05.h"

#define CTR2_BIT    0x2
#define CTR3_BIT    0x4
#define CTR5_BIT    0x10

/***************************************************************************
 *
 *  Global Data
 *
 ***************************************************************************/

char *DevName = "/dev/ctr05/ctr0_01";
int  Mode     = CTR05_COUNTER;
int  Print    = 1;
int  Status;

int fdDIOA;
int fdctr_1;
int fdctr_2;
int fdctr_3;
int fdctr_4;
int fdctr_5;

void DoOpenDevices()
{
  char str[80];

  strcpy(str, "/dev/ctr05/dio0_0A");
  if ((fdDIOA = open(str, Mode)) < 0) {
    perror(str);
    printf("error opening device %s\n",str);
    exit(2);
  }

  strcpy(str, "/dev/ctr05/ctr0_01");
  if ((fdctr_1 = open(str, CTR05_COUNTER)) < 0) {
    perror(str);
    printf("error opening device %s\n",str);
    exit(2);
  }

  strcpy(str, "/dev/ctr05/ctr0_02");
  if ((fdctr_2 = open(str, CTR05_COUNTER)) < 0) {
    perror(str);
    printf("error opening device %s\n",str);
    exit(2);
  }

  strcpy(str, "/dev/ctr05/ctr0_03");
  if ((fdctr_3 = open(str, CTR05_FREQUENCY)) < 0) {
    perror(str);
    printf("error opening device %s\n",str);
    exit(2);
  }
} 

void testCounter()
{
  unsigned short value;
  char str[80];
  int i, j;
  unsigned short source;

  printf("*********************************************************\n");
  printf("*     Connect the terminal blocks as follows:           *\n");
  printf("*          pin 10  <-------> pin 36                     *\n");
  printf("*********************************************************\n\n");

  source = SRC1;
  while (1) {
    for (i = 0; i < 10 ; i++)  {
      write(fdctr_1, &source, 2);
      for ( j = 0; j < i; j++ ) {
	value = 1;
        write(fdDIOA, &value, 1);
	value = 0;
        write(fdDIOA, &value, 1);
      }
      sleep(1);
      read(fdctr_1, &value, 1);
      printf("Count = %d\n", value);
    }
    printf("\nContinue? ");
    scanf("%s",str);
    if (str[0]=='N' || str[0]=='n') return;
  }
}


void testFreq()
{
  unsigned short value;
  char str[80];
  int source = SRC3;
  int freq;

  printf("*********************************************************\n");
  printf("*     Connect the terminal blocks as follows:           *\n");
  printf("*          pin 31  <-------> pin 16                     *\n");
  printf("*          pin 34  <-------> pin 17                     *\n");
  printf("*********************************************************\n\n");

  ioctl(fdctr_3, SET_GATE_INTERVAL, 5000);  // 5000 = 1 second */
  write(fdctr_3, &source, 2);

  while (1) {
    printf("Input desired frequency [Hz]: ");
    scanf("%d", &freq);
    ioctl(fdctr_2, SET_SQUARE_FREQ, freq);    
    sleep(2);
    read(fdctr_3, &value, 1);
    printf("Frequency Counter 3 is %d  ",value);
    printf("\nContinue? ");
    scanf("%s",str);
    if (str[0]=='N' || str[0]=='n') {
      ioctl(fdctr_5, LOAD_CMD_REG, DISARM | CTR5_BIT | CTR3_BIT);
      return;
    }
  }
}

void testSquare()
{
  char str[80];
  int freq;

  printf("*********************************************************\n");
  printf("*     Connect the terminal blocks as follows:           *\n");
  printf("*          pin 34  <------->  scope                     *\n");
  printf("*********************************************************\n\n");

  while (1) {
    printf("Input desired frequency [Hz]: ");
    scanf("%d", &freq);
    ioctl(fdctr_2, SET_SQUARE_FREQ, freq);    
    /*
    freq = 5000/(2*freq);
    ioctl(fdctr_2, LOAD_MODE_REG, 0x1e62);    // Counter 2 Mode Register value 0x1e62
    ioctl(fdctr_2, LOAD_LOAD_REG, freq);      // Counter 2 Load Register top half frequency
    ioctl(fdctr_2, LOAD_HOLD_REG, freq);      // Counter 2 Hold Register bottom half frequency
    ioctl(fdctr_2, LOAD_CMD_REG, 0x42);
    ioctl(fdctr_2, LOAD_CMD_REG, 0xe2);
    ioctl(fdctr_2, LOAD_CMD_REG, 0x22);
    */
    printf("\nContinue? ");
    scanf("%s",str);
    if (str[0]=='N' || str[0]=='n') {
      ioctl(fdctr_2, LOAD_CMD_REG, DISARM | CTR2_BIT);
      return;
    }
  }
}

void testDIO()
{
  unsigned  char bReg;
  unsigned short value;
  char str[80];

  printf("*********************************************************\n");
  printf("*     Connect the terminal blocks as follows:           *\n");
  printf("*          pin 3  <-------> pin 22                      *\n");
  printf("*          pin 4  <-------> pin 23                      *\n");
  printf("*          pin 5  <-------> pin 24                      *\n");
  printf("*          pin 6  <-------> pin 25                      *\n");
  printf("*          pin 7  <-------> pin 26                      *\n");
  printf("*          pin 8  <-------> pin 27                      *\n");
  printf("*          pin 9  <-------> pin 28                      *\n");
  printf("*          pin 10 <-------> pin 29                      *\n");
  printf("*********************************************************\n\n");

  while (1) {
    printf("Enter value for output in hex:");
    scanf("%hx", &value);
    write(fdDIOA, &value,1);
    read(fdDIOA, &bReg,1);
    printf("Input value =%x\n",bReg);
    printf("\n Continue? ");
    scanf("%s",str);
    if (str[0] == 'n' || str[0] == 'N') return;
  }
}

void Domenu()
{
  int choice;

  while (1) {
    system("/usr/bin/clear");
    printf("Select from the following choices:\n");
    printf("\t1. Test Counter.\n");
    printf("\t2. Measure Frequency.\n");
    printf("\t3. Test Digital I/O.\n");
    printf("\t4. Generate Square Wave.\n");
    printf("\t5. Exit.\n");
    printf("\nOption: ");

    scanf("%d", &choice);
    switch (choice) {
    case 1: 
      testCounter();
      break;
    case 2:
      testFreq();
      break;
    case 3:
      testDIO();
      break;
    case 4:
      testSquare();
      break;
    case 5:
      return;
      break;
    default:
      break;
    }
  }
}

int main(int argc, char **argv)
{
  DoOpenDevices();
  Domenu();
  close(fdDIOA);
  close(fdctr_1);
  close(fdctr_2);
  close(fdctr_3);
  return(0);
}
