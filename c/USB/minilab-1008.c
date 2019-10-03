/*
 *
 *  Copyright (c) 2004-2005  Warren Jasper <wjasper@ncsu.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include "pmd.h"
#include "minilab-1008.h"

/* Commands and Codes for miniLAB 1008 HID reports */
#define DCONFIG     (0x0D)     // Configure digital port
#define DIN         (0x00)     // Read digital port
#define DOUT        (0x01)     // Write digital port
#define DBIT_IN     (0x02)     // Read digital port bit
#define DBIT_OUT    (0x03)     // Write digital port bit

#define AIN         (0x06)     // Read analog input channel
#define AIN_SCAN    (0x0E)     // Scan analog channels
#define AIN_STOP    (0x10)     // Stop scan
#define ALOAD_QUEUE (0x07)     // Load the channel/gain queue

#define AOUT        (0x08)     // Write analog output channel

#define CINIT       (0x05)     // Initialize counter
#define CIN         (0x04)     // Read Counter

#define MEM_READ    (0x09)     // Read Memory
#define MEM_WRITE   (0x0A)     // Write Memory

#define BLINK_LED   (0x0B)     // Causes LED to blink
#define RESET       (0x11)     // Reset USB interface
#define SET_TRIGGER (0x14)     // Configure external trigger
#define SET_ID      (0x0C)     // Set the user ID
#define GET_ID      (0x0F)     // Get the user ID

#define LS_DELAY    (1500)


enum Mode {Differential, SingleEnded};

/* configures digital port */
void usbDConfigPort_miniLAB1008(hid_device* hid, uint8_t port, uint8_t direction)
{
  struct {
    uint8_t cmd;
    uint8_t port;
    uint8_t direction;
    uint8_t pad[5];
  } report;

  report.cmd = DCONFIG;
  report.port = port;
  report.direction = direction;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
}

/* reads digital port  */
void usbDIn_miniLAB1008(hid_device* hid, uint8_t port, uint8_t* din_value)
{
  struct t_report {
    uint8_t cmd;
    uint8_t port;
    uint8_t direction;
    uint8_t pad[5];
  } report;

  struct t_in {
    uint8_t value;
    uint8_t pad[7];
  } in;
    
  report.cmd = DIN;
  report.port = port;
  
  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, (uint8_t *) &in, sizeof(in), LS_DELAY);
  *din_value = in.value;
}

/* writes digital port */
void usbDOut_miniLAB1008(hid_device* hid, uint8_t port, uint8_t value) 
{
  uint8_t cmd[8];
  
  cmd[0] = DOUT;
  cmd[1] = port;
  cmd[2] = value;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

/* reads digital port bit */
uint8_t usbDBitIn_miniLAB1008(hid_device* hid, uint8_t port, uint8_t bit) 
{
  struct t_report {
    uint8_t cmd;
    uint8_t port;
    uint8_t bit;
    uint8_t pad[5];
  } report;

  struct t_in {
    uint8_t value;
    uint8_t pad[7];
  } in;

  report.cmd = DBIT_IN;
  report.port = port;
  report.bit = bit;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, (uint8_t*) &in, sizeof(in), LS_DELAY);

  return in.value;
}

/* writes digital port bit */
void usbDBitOut_miniLAB1008(hid_device* hid, uint8_t port, uint8_t bit, uint8_t value)
{
  struct t_report {
    uint8_t cmd;
    uint8_t port;
    uint8_t bit;
    uint8_t value;
    uint8_t pad[4];
  } report;
  
  report.cmd = DBIT_OUT;
  report.port = port;
  report.bit = bit;
  report.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
}

/* reads from analog in */
signed short usbAIn_miniLAB1008(hid_device* hid, uint8_t channel, uint8_t range)
{
  enum Mode mode;

  struct t_report {
    uint8_t cmd;
    uint8_t channel;
    uint8_t range;
    uint8_t pad[5];
  } report;

  struct t_ain {
    uint8_t lo_byte;
    uint8_t hi_byte;
    uint8_t pad[6];
  } ain;

  int16_t value;

  report.cmd = AIN;
  report.channel = channel;
  report.range = range;

  if ( range == SE_10_00V ) {
    mode = SingleEnded;
  } else {
    mode = Differential;
  }
  if (channel > 3 && mode == Differential ) {
    printf("usbAIN: channel out of range for differential mode.\n");
    return -1;
  }

  if (channel > 7 && mode == SingleEnded ) {
    printf("usbAIN: channel out of range for single ended mode.\n");
    return -1;
  }
  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, (uint8_t *) &ain, sizeof(ain), LS_DELAY);

  if ( mode == Differential ) {
    /* the data is a 2's compliment signed 12 bit number */
    value = (ain.hi_byte << 8) | (ain.lo_byte << 4);
    value /= (1 << 4);
  } else {
    /* the data is a  11 bit number signed offset*/
    value = (ain.hi_byte << 4) | (0x0f & ain.lo_byte);
    value -= 0x400;
  }
  return value;
}

/* writes to analog out */
void usbAOut_miniLAB1008(hid_device* hid, uint8_t channel, uint16_t value) 
{
  uint8_t cmd[8];

  cmd[0] = AOUT;
  cmd[1] = channel;
  cmd[2] = (uint8_t) (value & 0xff);
  cmd[3] = (uint8_t) ((value >> 8) & 0xff);
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

void usbAInScan_miniLAB1008(hid_device* hid, uint16_t count, int rate, uint8_t low_channel, uint8_t high_channel, uint8_t options, int16_t value[], uint8_t gainLoadQueue[])
{

  int i, idx;
  int ret;
  uint16_t scan_index;
  uint16_t actual_scan_index;
  uint8_t chanCount;
  uint8_t chanLoadQueue[8];
  
  struct {
    uint8_t cmd;
    uint8_t lo_count;
    uint8_t hi_count;
    uint8_t timer_preload;
    uint8_t timer_prescale;
    uint8_t options;
    uint8_t extra[2];
  } out;

  struct t_Feature_Report{
    uint8_t recordNum;
    uint8_t data[96];
    uint8_t error;
    uint8_t readAddress[2];
    uint8_t writeAddress[2];
    uint8_t scanIndex[2];
    uint8_t extra;
  } feature_report;
  
  uint8_t buffer[9] = { 0, 0, 0, 0, 0, 0, 0, 0};

  int timerMult;
  uint8_t timerVal;
  uint8_t prescale;
  uint8_t preload;
  uint8_t setupTime;

  if ((100 <= rate) && (rate < 200)) {       // Select 256:1 prescaler
    prescale = 7;
    setupTime = 0;
  } else if ((200 <= rate) && (rate < 400)) { // Select 128:1 prescaler
    prescale = 6;
    setupTime = 0;
  } else if ((400 <= rate) && (rate < 800)) { // Select 64:1 prescaler
    prescale = 5;
    setupTime = 0;
  } else if ((800 <= rate) && (rate < 1500)) { // Select 32:1 prescaler
    prescale = 4;
    setupTime = 1;
  } else if ((1500 <= rate) && (rate < 3000)) { // Select 16:1 prescaler
    prescale = 3;
    setupTime = 3;
  } else if ((3000 <= rate) && (rate < 6000)) { // Select 8:1 prescaler
    prescale = 2;
    setupTime = 6;
  } else if ((6000 <= rate) && (rate < 8192)) { // Select 4:1 prescaler
    prescale = 1;
    setupTime = 10;
  } else {
    printf("usbAInScan_miniLAB1008: sampling rate out of range.\n");
    return;
  }

  timerMult = 1 << (prescale + 1);
  timerVal = (uint8_t) ((256 - (MINILAB_CLOCK / (rate * timerMult))) + 0.5);
  preload = timerVal + setupTime;

  /* set up gain queue */
  chanCount = high_channel - low_channel + 1;
  for ( i = 0; i < 8; i++ ) {
    chanLoadQueue[i] = 0;
  }
  for ( i = 0; i < chanCount; i++ ) {
    chanLoadQueue[i] = low_channel + i;
  }
  usbAInLoadQueue_miniLAB1008(hid, chanCount, chanLoadQueue, gainLoadQueue);

  out.cmd = AIN_SCAN;
  out.lo_count = count & 0xff;
  out.hi_count = (count >> 8) & 0xff;
  out.timer_preload = preload;
  out.timer_prescale = prescale;
  out.options = options;

  PMD_SendOutputReport(hid, (uint8_t*) &out, sizeof(out));

  /*
     If in external trigger mode, then wait for the device to send back
     notice that the trigger has been received, then startup the acquisition
  */

  if ( options & AIN_TRIGGER ) {
    while ( buffer[1] != 0xC3 ) {  // wait until external trigger received
      buffer[0] = 0;
      PMD_GetInputReport(hid, (uint8_t  *) &buffer, sizeof(buffer), LS_DELAY*10);
    }
  }

  /*
    In Burst Mode, wait for End of Block acquisition flag (0xA5)
  */

  if ( options & AIN_BURST_MODE ) {
    buffer[1] = 0;
    while ( buffer[1] != 0xA5 ) {  // wait until external trigger received
      buffer[0] = 0;
      PMD_GetInputReport(hid, (uint8_t  *) &buffer, sizeof(buffer), LS_DELAY);
    }
  }
  
  /*
    Retrieve the AInScan Response
    GET HID Feature Report to collect the data buffer.  Each buffer will be 105
    bytes long.  The first bye will contain the record number and can be ignored.
    The following 96 bytes will represent 64 samples of data.
  */

  feature_report.scanIndex[0] = 0xff;
  feature_report.scanIndex[1] = 0xff;
  scan_index = 0;
  idx = 0;
  hid_set_nonblocking(hid, 1);
  while (count > 0) {
    //      PMD_GetFeatureReport(hid, (uint8_t *) &feature_report, sizeof(feature_report));
    ret = hid_get_feature_report(hid, (unsigned char *) &feature_report, sizeof(feature_report));
    scan_index++;
    actual_scan_index = (uint16_t) (feature_report.scanIndex[0] | feature_report.scanIndex[1] << 8);
    if (ret == -1){
      printf("Completed scan %d  error = %d\n", actual_scan_index, feature_report.error);
    }
    for (i = 0; i < 96; i += 3, idx += 2) {
      value[idx] = feature_report.data[i] | ((feature_report.data[i+1]<<4) & 0x0f00);
      if (value[idx] & 0x800) (*((uint16_t *)(&(value[idx])))) |= 0xf000;
      value[idx + 1] = feature_report.data[i+2] | ((feature_report.data[i+1]<<8) & 0x0f00);
      if (value[idx+1] & 0x800) (*((uint16_t *)(&(value[idx+1])))) |= 0xf000;
      count -= 2;
      if (count == 0) {
        usbAInStop_miniLAB1008(hid);   // just to make sure.
	return;
      }
    }
  }
}

void usbAInLoadQueue_miniLAB1008(hid_device* hid, uint8_t chanCount, uint8_t chanLoadQueue[], uint8_t gainLoadQueue[])
{
  int i;   
  struct {
    uint8_t cmd;
    uint8_t count;
    uint8_t gains[6];
  } out;

  out.cmd = ALOAD_QUEUE;
  out.count = chanCount;   // can be 1, 2, 4, or 8
  chanCount = (chanCount == 8) ? 6 : chanCount;
  for ( i = 0; i < chanCount; i++ ) {
    out.gains[i] = (chanLoadQueue[i] & 0x7) | gainLoadQueue[i] | 0x80;
  }
  PMD_SendOutputReport(hid, (uint8_t*) &out, sizeof(out));

  // Configure the rest of the channels (channel 6 and 7 )
  if ( out.count == 8 ) {
    out.count = 0x2;
    for ( i = 6; i <= 7; i++ ) {
    out.gains[i-6] = (chanLoadQueue[i] & 0x7) | gainLoadQueue[i] | 0x80;  
    }
    PMD_SendOutputReport(hid, (uint8_t*) &out, sizeof(out));
  }
}

void usbAInStop_miniLAB1008(hid_device* hid)
{
  uint8_t cmd[8];
  
  cmd[0] = AIN_STOP;
  
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}  


/* Initialize the counter */
void usbInitCounter_miniLAB1008(hid_device* hid)
{
  uint8_t cmd[8];
  
  cmd[0] = CINIT;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint32_t usbReadCounter_miniLAB1008(hid_device* hid)
{
  struct t_report {
    uint8_t cmd;
    uint8_t pad[7];
  } report;

  struct t_in {
    uint32_t count;
    uint8_t pad[4];
  } in;

  report.cmd = CIN;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, (uint8_t  *) &in, sizeof(in), LS_DELAY);

  return in.count;
}

void usbReadMemory_miniLAB1008(hid_device* hid, uint16_t address, uint8_t *buffer, uint8_t count)
{
  struct t_report {
    uint8_t cmd;
    uint8_t address_low;
    uint8_t address_hi;
    uint8_t count;
    uint8_t pad[4];
  } report;

  uint8_t data[8];

  if (count > 8) count = 8;     // max count is 8.

  report.cmd = MEM_READ;
  report.address_low = (uint8_t) (address & 0xff);  // low byte
  report.address_hi = (uint8_t) (address >> 0x8);  // high byte
  report.count = count;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, data, sizeof(data), LS_DELAY);
  memcpy(buffer, data, count);
}

/* blinks the LED of USB device */
void usbBlink_miniLAB1008(hid_device* hid)
{
  struct {
    uint8_t cmd;
    uint8_t pad[7];
  } report;

  report.cmd = BLINK_LED;
  PMD_SendOutputReport(hid,(uint8_t*) &report, sizeof(report));
}

/* resets the USB device */
int usbReset_miniLAB1008(hid_device* hid)
{
  uint8_t cmd[8];

  cmd[0] = RESET;

  return PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

/* configure external triger */
void usbSetTrigger_miniLAB1008(hid_device* hid, uint8_t type, uint8_t chan)
{
  uint8_t cmd[8];

  cmd[0] = SET_TRIGGER;
  cmd[1] = type;
  cmd[2] = chan;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint8_t usbGetID_miniLAB1008(hid_device* hid)
{
  struct t_report {
    uint8_t cmd;
    uint8_t pad[7];
  } report;

  struct t_in {
    uint8_t id;
    uint8_t pad[7];
  } in;

  report.cmd = GET_ID;

  PMD_SendOutputReport(hid, (uint8_t*) &report, sizeof(report));
  PMD_GetInputReport(hid, (uint8_t*) &in, sizeof(in), LS_DELAY);

  return in.id;
}

void usbSetID_miniLAB1008(hid_device* hid, uint8_t id)
{
  uint8_t cmd[8];

  cmd[0] = SET_ID;
  cmd[1] = id;

  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}
