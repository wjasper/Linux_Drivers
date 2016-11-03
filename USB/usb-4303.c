/*
 *
 *  Copyright (c) 2015 Warren J. Jasper <wjasper@tx.ncsu.edu>
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
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include "pmd.h"
#include "usb-4303.h"

#define FS_DELAY (1000)

/* reads digital port  */
uint8_t usbDIn_USB4303(hid_device *hid)
{
  uint8_t reportID = DIN;
  struct read_port_t {
    uint8_t reportID;
    uint8_t value;
  } read_port;

  read_port.reportID = DIN;

  PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
  PMD_GetInputReport(hid, (uint8_t *) &read_port, sizeof(read_port), FS_DELAY);
  return read_port.value;
}

/* writes digital port */
void usbDOut_USB4303(hid_device *hid, uint8_t value) 
{
  struct write_port_t {
    uint8_t reportID;
    uint8_t value;
  } write_port;

  write_port.reportID = DOUT;
  write_port.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &write_port, sizeof(write_port));
}

/* reads digital port bit */
uint8_t usbDBitIn_USB4303(hid_device *hid, uint8_t bit) 
{
  struct read_bit_t {
    uint8_t reportID;
    uint8_t value;
  } read_bit;

  read_bit.reportID = DBIT_IN;
  read_bit.value = bit;

  PMD_SendOutputReport(hid, (uint8_t*) &read_bit, sizeof(read_bit));
  PMD_GetInputReport(hid,   (uint8_t*) &read_bit, sizeof(read_bit), FS_DELAY);  
  return read_bit.value;
}

/* writes digital port bit */
void usbDBitOut_USB4303(hid_device *hid, uint8_t bit, uint8_t value)
{
  struct write_bit_t {
    uint8_t reportID;
    uint8_t bit;
    uint8_t value;
  } write_bit;

  write_bit.reportID = DBIT_OUT;
  write_bit.bit = bit;
  write_bit.value = value;

  PMD_SendOutputReport(hid, (uint8_t*) &write_bit, sizeof(write_bit));
  return;
}

void usbLoad_USB4303(hid_device *hid, uint8_t chip, uint8_t counters)
{
  /*
     This command loads the selected counters with current load or hold register value.  If counter is
     armed, the source register will be the one to be used for the upcoming terminal count value.  If
     the counter is disarmed, the counter will be loaded from the load register, unless in mode S or V,
     in which case, the counter gate input will select the source register.
  */

  struct load_t {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counters;  
  } load;

  load.reportID = LOAD;
  load.chip = chip;
  load.counters = counters;

  PMD_SendOutputReport(hid, (uint8_t *) &load, sizeof(load));
}

void usbSave_USB4303(hid_device *hid, uint8_t chip, uint8_t counters)
{
  /*
    This command saves the current counter value(s) to the associated hold register(s).  Any
    previous hold register contents will be overwritten.  This could cause problems if the hold
    register is being used for loading the counter (modes G-L, S and V).
  */

  struct save_t {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counters;  
  } save;

  save.reportID = SAVE;
  save.chip = chip;
  save.counters = counters;
  PMD_SendOutputReport(hid, (uint8_t *) &save, sizeof(save));
}

void usbArm_USB4303(hid_device *hid, uint8_t chip, uint8_t counters)
{
  /*
    This function arms the selected counters and begins counting.  If the counter is using
    mode G-L (alternate loading from load then hold register), arming the counter resets the
    load/hold logic; the counter will be loaded from the hold register on the next TC, and then
    alternate thereafter.
  */

  struct arm_t {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counters;  
  } arm;

  arm.reportID = ARM;
  arm.chip = chip;
  arm.counters = counters;
  PMD_SendOutputReport(hid, (uint8_t *) &arm, sizeof(arm));
}

void usbDisarm_USB4303(hid_device *hid, uint8_t chip, uint8_t counters)
{
  /*
    This function disarms the selected counter(s) and stops counting.
  */

  struct disarm_t {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counters;  
  } disarm;

  disarm.reportID = DISARM;
  disarm.chip = chip;
  disarm.counters = counters;
  PMD_SendOutputReport(hid, (uint8_t *) &disarm, sizeof(disarm));
}

uint16_t usbRead_USB4303(hid_device *hid, uint8_t chip, uint8_t counter)
{
  /*
    This function reads the current value of the 16-bit counter. To accomplish this, the counter
    value is latched out to the HOLD register and the HOLD register value is returned.  If the
    counter is in a mode that uses the HOLD register to load the counter (modes G-L, S and V), 
    this function returns 0.
  */

  struct read_t {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counter;   // Only use bits [0:2].  
  } read;

  struct read_counter_t {
    uint8_t reportID;
    uint8_t value[2];
  } read_counter;

  uint16_t value;

  read.reportID = READ;
  read.chip = chip;
  switch (counter) {
    case COUNTER_1:  read.counter = 0x1; break;
    case COUNTER_2:  read.counter = 0x2; break;
    case COUNTER_3:  read.counter = 0x3; break;
    case COUNTER_4:  read.counter = 0x4; break;
    case COUNTER_5:  read.counter = 0x5; break;
    default: return 0; break;
  }

  PMD_SendOutputReport(hid, (uint8_t *) &read, sizeof(read));
  PMD_GetInputReport(hid, (uint8_t *) &read_counter, sizeof(read_counter), FS_DELAY);
  memcpy(&value, read_counter.value, 2);
  return value;
}

void usbSetToggle_USB4303(hid_device *hid, uint8_t chip, uint8_t counter, uint8_t set)
{
  /*
    This function sets or clears the selected counter output if the counter output is set
    to toggle on terminal count. There is no effect for other ouptut modes.
  */
  struct setToggle_t {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counter;   // Only use bits [0:2]. 
    uint8_t set;       // 0 = clear output, 1 = set output
  } setToggle;

  setToggle.reportID = SET_TOGGLE;
  setToggle.chip = chip;
  switch (counter) {
    case COUNTER_1:  setToggle.counter = 0x1; break;
    case COUNTER_2:  setToggle.counter = 0x2; break;
    case COUNTER_3:  setToggle.counter = 0x3; break;
    case COUNTER_4:  setToggle.counter = 0x4; break;
    case COUNTER_5:  setToggle.counter = 0x5; break;
  default:
    return ;
    break;
  }
  setToggle.set = set;
  PMD_SendOutputReport(hid, (uint8_t *) &setToggle, sizeof(setToggle));
}

void usbStep_USB4303(hid_device *hid, uint8_t chip, uint8_t counter)
{
  /*
    This function steps the selected counter value up or down.  It will increment if the counter is
    configured to count up and decrement if the counter is configured to count down
  */

  struct t_step {
    uint8_t reportID;
    uint8_t chip;      // 1 = chip 1,  2 = chip 2
    uint8_t counter;   // Only use bits [0:2]. 
  } step;

  step.reportID = STEP;
  step.chip = chip;
  switch (counter) {
    case COUNTER_1:  step.counter = 0x1; break;
    case COUNTER_2:  step.counter = 0x2; break;
    case COUNTER_3:  step.counter = 0x3; break;
    case COUNTER_4:  step.counter = 0x4; break;
    case COUNTER_5:  step.counter = 0x5; break;
    default: return ; break;
  }

  PMD_SendOutputReport(hid, (uint8_t *) &step, sizeof(step));
}

void usbSet9513Config_USB4303(hid_device *hid, uint8_t chip, uint16_t settings)
{
  /*
    The command sets the configuration options for the 9513 chip through the Master Mode
    Register.  The default for the "settings" parameter upon initialization of the device
    is 0x8000.  Refer to the 9513A datasheet for details.
  */
  struct set9513Config_t {
    uint8_t reportID;
    uint8_t chip;          // 1 = chip 1,  2 = chip 2
    uint8_t settings[2];   
  } set9513Config;

  set9513Config.reportID = SET9513CONFIG;
  set9513Config.chip = chip;
  set9513Config.settings[0] = (uint8_t) (settings & 0xff);          // low byte
  set9513Config.settings[1] = (uint8_t) (settings >> 0x8) & 0xff;   // hight byte

  PMD_SendOutputReport(hid, (uint8_t *) &set9513Config, sizeof(set9513Config));
}

uint16_t usbGet9513Config_USB4303(hid_device *hid, uint8_t chip)
{
  /*
    The command returns the 9513 configuration settings.
  */
  struct get9513Config_t {
    uint8_t reportID;
    uint8_t value[2];
  } get9513Config;
  
  uint16_t value;

  get9513Config.reportID = GET9513CONFIG;
  get9513Config.value[0] = chip;

  PMD_SendOutputReport(hid, (uint8_t *) &get9513Config, sizeof(get9513Config)-1);
  PMD_GetInputReport(hid, (uint8_t *) &get9513Config, sizeof(get9513Config), FS_DELAY);
  memcpy(&value, get9513Config.value, 2);
  return value;
}

void usbSetOscOut_USB4303(hid_device *hid, uint8_t chip, uint8_t enable)
{
  /*
    This command enables or disables the oscillator output.
  */

  struct setOscOut_t {
    uint8_t reportID;
    uint8_t chip;    // 1 = chip 1 ,  2 = chip 2
    uint8_t enable;  // 0 = disable,  1 = enable
  } setOscOut;

  setOscOut.reportID = SETOSCOUT;
  setOscOut.chip = chip;
  setOscOut.enable = enable;
  PMD_SendOutputReport(hid, (uint8_t *) &setOscOut, sizeof(setOscOut));
}

void usbSetRegister_USB4303(hid_device *hid, uint8_t chip, uint8_t reg, uint16_t value)
{
  /*
    This command sets the value of the selected 9513 register.
  */

  struct t_setReg {
    uint8_t reportID;
    uint8_t chip;
    uint8_t reg;
    uint8_t value[2];
  } setReg;

  setReg.reportID = SETREGISTER;
  setReg.chip = chip;
  setReg.reg = reg;
  setReg.value[0] = (uint8_t) (value & 0xff);           // low  byte
  setReg.value[1] = (uint8_t) ((value >> 8) & 0xff);    // high byte

  PMD_SendOutputReport(hid, (uint8_t *) &setReg, sizeof(setReg));
}

uint16_t usbGetRegister_USB4303(hid_device *hid, uint8_t chip, uint8_t reg)
{
  /*
    This command returns the value of the selected 9513 register
  */

  struct getReg_t{
    uint8_t reportID;
    uint8_t value[2];
  } getReg;

  uint16_t value;

  getReg.reportID = GETREGISTER;
  getReg.value[0] = chip;
  getReg.value[1] = reg;

  PMD_SendOutputReport(hid, (uint8_t *) &getReg, sizeof(getReg));
  PMD_GetInputReport(hid, (uint8_t *) &getReg, sizeof(getReg), FS_DELAY);
  memcpy(&value, getReg.value, 2);
  return value;
}

void usbReset9513_USB4303(hid_device *hid, uint8_t chip)
{
  /* This command resets the 9513 chip to default values and initializes the counter registers to 
     all zeros.
  */
  uint8_t cmd[2];

  cmd[0] = RESET;
  cmd[1] = chip;
  PMD_SendOutputReport(hid, (uint8_t *) &cmd, sizeof(cmd));
}

void usbSelect9513Clock_USB4303(hid_device *hid, uint8_t clock)
{
  /*
    This command selects the initial clock frequency for the 9513 counter.  On the USB-4303,
    this clock frequency is input to both chips, they cannot be selected individually.

   Values of clock are:
       0x00  =  1 MHz
       0x01  =  1.66667 MHz
       0x02  =  3.33333 MHz
       0x03  =  5 MHz (default)
       all other values = no change
  */

  uint8_t cmd[2];

  cmd[0] = SELECT9513CLOCK;
  cmd[1] = clock;
  PMD_SendOutputReport(hid, (uint8_t *) &cmd, sizeof(cmd));
}

uint16_t usbReadFreq_USB4303(hid_device *hid, uint8_t chip, uint8_t source, uint16_t interval_counts)
{
  /*
    This command configures counters 4 and 5 on the selected chip to measure frequency.  This
    requires connecting the output pin of counter 4 to the gate of counter 5.  The gating 
    interval depends on the internal clock selected (with Select9513Clock).

    (gating interval in ms) = interval_counts / (internal clock frequency in MHz)

    The response is the total number of rising edges received on the input specificied by source
    during the gating interval.  To calculate the frequency from this use the following:

    (frequency in Hz) = [(received pulses) / (gating interval in ms)] * 1000

    If an overrun occurs (more than 65,535 rising edges are received during the gatin interval), the
    response will be 0xff.
  */

  struct readFreq_t {
    uint8_t reportID;
    uint8_t chip;
    uint8_t source;
    uint8_t interval_counts[2];
  } readFreq;

  struct readFreq2_t {
   uint8_t reportID;
   uint8_t freq[2];
  } readFreq2;

  uint16_t value;

  readFreq.reportID = READFREQ;
  readFreq.chip = chip;
  readFreq.source = source;
  memcpy(readFreq.interval_counts, &interval_counts, 2);

  PMD_SendOutputReport(hid, (uint8_t *) &readFreq, sizeof(readFreq));
  PMD_GetInputReport(hid, (uint8_t *) &readFreq2, sizeof(readFreq2), FS_DELAY);
  memcpy(&value, readFreq2.freq, 2);
  return value;
}

void usbReadMemory_USB4303(hid_device *hid, uint16_t address, uint8_t count, uint8_t* memory)
{
  /*
    The command reads data from the EEPROM configuration memory.  There are 256 bytes
    available in the EEPROM (address 0x00 to 0xff).
  */

  struct arg_t {
    uint8_t reportID;
    uint8_t address[2];  // the start address for the read
    uint8_t type;        // not used
    uint8_t count;       // the number of bytes to read (62 max)
  } arg;

  if (count > 62) count = 62;
  arg.reportID = MEM_READ;
  arg.type = 0;      // unused for this device.
  arg.address[0] = (uint8_t) (address & 0xff);         // low byte
  arg.address[1] = (uint8_t) ((address >> 8) & 0xff);  // high byte
  arg.count = count;

  PMD_SendOutputReport(hid, (uint8_t *) &arg, sizeof(arg));
  PMD_GetInputReport(hid, (uint8_t *) &memory, sizeof(memory), FS_DELAY);
}

int usbWriteMemory_USB4303(hid_device *hid, uint16_t address, uint8_t count, uint8_t* data)
{
  /*
    The command writes to the EEPROM on the device micrprocessor.  This non-volatile memory
    is used to store system information and user data.  

    The EEPROM may be written at any address with 1-60 bytes of data in a write.
  */

  int i;
  struct mem_write_report_t {
    uint8_t reportID;
    uint8_t address[2];   // the start address for the write
    uint8_t count;        // the number of bytes to write (60 max)
    uint8_t data[count];  // the data to be written (60 bytes max)
  } arg;

  if (count > 60) count = 60;

  arg.reportID = MEM_WRITE;
  arg.address[0] = (uint8_t) (address & 0xff);         // low byte
  arg.address[1] = (uint8_t) ((address >> 8) & 0xff);  // high byte
  arg.count = count;
  for ( i = 0; i < count; i++ ) {
    arg.data[i] = data[i];
  }
  PMD_SendOutputReport(hid, (uint8_t *) &arg, count+4);
  return 0;
}

/* blinks the LED of USB device */
int usbBlink_USB4303(hid_device *hid, uint8_t count)
{
  /* 
    This command causes the LED to blink "count" times.
  */
  uint8_t cmd[2];

  cmd[0] = BLINK_LED;
  cmd[1] = count;
  
  return PMD_SendOutputReport(hid, (uint8_t *) &cmd, sizeof(cmd));
}

int usbReset_USB4303(hid_device *hid)
{
  /*
    This function causes the device to perform a reset.  The device disconnects from the USB bus
    and resets the microcontroller.
  */

  uint8_t reportID = RESET;
  return PMD_SendOutputReport(hid, &reportID, sizeof(reportID));
}

uint32_t usbGetStatus_USB4303(hid_device *hid)
{
  /*
    This command retrives the status of the device.
  */
  uint32_t status;
  int nread;
  struct statusReport_t {
    uint8_t reportID;
    uint8_t status[4];
  } statusReport;

  PMD_SendOutputReport(hid, &statusReport.reportID, 1);
  nread = PMD_GetInputReport(hid, (uint8_t *) &statusReport, sizeof(statusReport), FS_DELAY);

  if (nread < 0) {
    perror("usbGetStatus_USB4303");
    return nread;
  }
  memcpy(&status, statusReport.status, 4);
  return status;
}

void usbInterruptConfig_USB4303(hid_device *hid, uint16_t config, uint16_t data[10])
{
  /*
    This command configures the external interrupt pin.  It can generate an event notification
    to the PC, latch the digital inputs, latch the digital outputs, and/or generate a SAVE command
    for and/all counters on either/both 9513s.  All interrupt settings initializes to off, with 
    rising edge triggering.

    The response for this command is sent to the PC whenever the interrupt edge is received if
    event notification is on.  If digital input latch in on, DIn and DBitIn will return the data 
    that was latched in the most recent interrupt edge.  If digital output latch is on, when the 
    interrupt edge is received, the most recently received data via DOut or DBitOut will be latched out.

    Note that setting digital input latch to on will immediately latch in the current input value, and
    setting digital output latch to on will initialize th edata to be latched out to 0.  Also note that the
    SAVE command overwrites the HOLD register contents and thus can cause unpredictable results on 
    counters that are using modes G-L, S or V.
  */

  struct interrupt_config_t {
    uint8_t reportID;
    uint8_t config[2];
  } interrupt_config;

  struct interrupt_config2_t {
    uint8_t reportID;
    uint8_t data[20];
  } interrupt_config2;

  memcpy(interrupt_config.config, &config, 2);

  PMD_SendOutputReport(hid, (uint8_t *) &interrupt_config, sizeof(interrupt_config));
  PMD_GetInputReport(hid, (uint8_t *) &interrupt_config2, sizeof(interrupt_config2), FS_DELAY);
  memcpy(data, interrupt_config2.data, 20);
}

  void usbPrepareDownload_USB4303(hid_device *hid)
{
  /*
    This command puts the device into code update mode.  The unlock code must be correct as a
    further safety device.  Call this once before sending code with WriteCode.  If not in code
    update mode, any WriteCode commands will be ignored.
  */

  struct download_t {
    uint8_t reportID;
    uint8_t unlock_code;
  } download;

  download.reportID = PREPARE_DOWNLOAD;
  download.unlock_code = 0xad;
  
  PMD_SendOutputReport(hid, (uint8_t *) &download, sizeof(download));
}

  void usbWriteCode_USB4303(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
{
  /*
    This command writes to the program memory in the device.  This command is not accepted
    unless the device is in update mode.  This command will normally be used when downloading
    a new hex file, so it supports memory ranges that may be found in the hex file.  

    The address ranges are:

    0x000000 - 0x007AFF:  Microcontroller FLASH program memory
    0x200000 - 0x200007:  ID memory (serial number is stored here on main micro)
    0x300000 - 0x30000F:  CONFIG memory (processor configuration data)
    0xF00000 - 0xF03FFF:  EEPROM memory

    FLASH program memory: The device must receive data in 64-byte segments that begin
    on a 64-byte boundary.  The data is sent in messages containing 32 bytes.  count
    must always equal 32.

    Other memory: Any number of bytes up to the maximum (32) may be sent.
    
  */

  struct writecode_t {
    uint8_t reportID;
    uint8_t address[3];
    uint8_t count;
    uint8_t data[32];
  } writecode;

  if (count > 32) count = 32;               // 32 byte max
  writecode.reportID = WRITE_CODE;
  memcpy(writecode.address, &address, 3);   // 24 bit address
  writecode.count = count;                  // the number of byes of data (max 32)
  memcpy(writecode.data, data, count);      // the program data
  PMD_SendOutputReport(hid, (uint8_t *) &writecode, count+5);
}

void usbWriteSerial_USB4303(hid_device *hid, uint8_t serial[8])
{
  /*
    This command sends a new serial number to the device.  The serial number consists of 8 bytes,
    typically ASCII numeric or hexadecimal digits (i.e. "00000001").  The new serial number will
    be programmed but not used until hardware reset.
  */
  struct writeSerialNumber_t {
    uint8_t reportID;
    uint8_t serial[8];
  } writeSerialNumber;

  writeSerialNumber.reportID = WRITE_SERIAL;
  memcpy(writeSerialNumber.serial, serial, 8);
  PMD_SendOutputReport(hid, (uint8_t*) &writeSerialNumber, sizeof(writeSerialNumber));
}

int usbReadCode_USB4303(hid_device *hid, uint32_t address, uint8_t count, uint8_t data[])
{
  /*
    This command reads from the program memory.
 
    The address ranges are:

    0x000000 - 0x007AFF:  Microcontroller FLASH program memory
    0x200000 - 0x200007:  ID memory (serial number is stored here on main micro)
    0x300000 - 0x30000F:  CONFIG memory (processor configuration data)

    Note: The EEPROM memory can only be read using the MemRead command.
  */

  struct readCode_t {
    uint8_t reportID;
    uint8_t address[3];    // 24 bit start address for the read
    uint8_t count;         // the number of bytes to read (62 byte max)
  } readCode;

  struct readCodeI_t {
    uint8_t reportID;
    uint8_t data[62];
  } readCodeI;

  if (count > 62) count = 62;  

  readCode.reportID = READ_CODE;
  memcpy(readCode.address, &address, 3);   // 24 bit address
  readCode.count = count;
  PMD_SendOutputReport(hid, (uint8_t *) &readCode, sizeof(readCode));
  if (PMD_GetInputReport(hid, (uint8_t *) &readCodeI, count+1, FS_DELAY) < 0) {
    perror("usbReadCode_USB4303 error on read.");
  }
  memcpy(data, readCodeI.data, count);
  return count;
}

void usbWrite9513Command_USB4303(hid_device *hid, uint8_t chip, uint8_t command)
{
  /*
    This command writes the specified command directly to the selected 9513 chip's command port.
  
    Note: Direct port access API commands should be used with caution; there are no safeguards to
    prevent configuration of the 9513 in such a way as to make it unusable.  (For instance, changing
    the data bus mode from 8-bit to 16-bit).  If this occurs, a device power-reset will always return
    the device to a working condition.
  */

  uint8_t cmd[3];

  cmd[0] = WRITE_COMMAND;
  cmd[1] = chip;      // 1 = chip 1,  2 = chip 2
  cmd[2] = command;   // the byte to write to the 9513 command port.
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint8_t usbRead9513Command_USB4303(hid_device *hid, uint8_t chip)
{
  /* 
     This command reads the selected 9513 chip's command port, which returns the current value of the 9513
     status register.
  */

  struct read_command {
    uint8_t reportID;
    uint8_t chip;
  } read_command;

  read_command.reportID = READ_COMMAND;
  read_command.chip = chip;
  
  PMD_SendOutputReport(hid, (uint8_t *) &read_command, sizeof(read_command));
  PMD_GetInputReport(hid, (uint8_t *) &read_command, sizeof(read_command), FS_DELAY);
  return read_command.chip;
}

void usbWrite9513Data_USB4303(hid_device *hid, uint8_t chip, uint8_t data)
{
  /*
    This command writes directly to the selected 9513 chip's data port.  A write to one of the data
    registers will typically involve first issuing a command to the command port to set the value of 
    the data pointer register, followed by two 8-bit writes to the data port to set the value of the
    desired 16-bit data register, LSB first.
  */

  uint8_t cmd[3];

  cmd[0] = WRITE_DATA;
  cmd[1] = chip;     // 1 = chip 1,  2 = chip 2
  cmd[2] = data;     // the 8-bit value to write to the data port
  PMD_SendOutputReport(hid, cmd, sizeof(cmd));
}

uint8_t usbRead9513Data_USB4303(hid_device *hid, uint8_t chip)
{
  /*
    This command reads directly to the selected 9513 chip's data port.  A read to one of the data
    registers will typically involve first issuing a command to the command port to set the value of 
    the data pointer register, followed by two 8-bit reads to the data port to retrieve the value of the
    desired 16-bit data register, LSB first.
  */

  uint8_t cmd[2];

  cmd[0] = READ_DATA;
  cmd[1] = chip;

  PMD_SendOutputReport(hid, (uint8_t *) &cmd, sizeof(cmd));
  PMD_GetInputReport(hid, (uint8_t *) &cmd, sizeof(cmd), FS_DELAY);
  return cmd[1];
}
