#! /usr/bin/python3
#
# Copyright (c) 2018 Warren J. Jasper <wjasper@ncsu.edu>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

from datetime import datetime
from struct import *
from mccEthernet import *

E1608_PID = 0x012F  # Product code for the MCC E-1608


# Gain Ranges
BP_10V             = 0x0  # +/- 10.0V
BP_5V              = 0x1  # +/- 5.0V
BP_2V              = 0x2  # +/- 2.0V
BP_1V              = 0x3  # +/- 1.0V

# Trigger options for AInScanStart
TRIGGER_NONE       = 0x0
TRIGGER_RISE       = (0x1 << 2)
TRIGGER_FALL       = (0x2 << 2)
TRIGGER_HIGH       = (0x3 << 2)
TRIGGER_LOW        = (0x4 << 2)

# Data Acquisition Modes
NMODE              = 2    # total number of modes
SE                 = 0    # Single ended mode
DF                 = 8    # Differential mode
NCHAN_AIN          = 8    # max number of ADC channles (8 single ended, 4 differential)
NCHAN_AOUT         = 2    # max number of DAC channels
NGAINS             = 4    # max number of gain levels (+/- 10V, +/- 5V, +/- 2V, +/- 1V)


"""
    Settings memory map
|=====================================================================================|
|    Address    |        Value                                        | Default value |
|=====================================================================================|
| 0x000 - 0x001 | Network options:                                    | 0x0000        |
|               |   Bit 0: 0 = DHCP enabled     1 = DHCP disabled     |               |
|               |   Bit 1: 0 = Auto IP enabled  1 = Auto IP disabled  |               |
|               |   Bits 2-15 reserved                                |               |
|-------------------------------------------------------------------------------------|
| 0x002 - 0x005 | Default IP address                                  | 192.168.0.101 |
|-------------------------------------------------------------------------------------|
| 0x006 - 0x009 | Default subnet mask                                 | 255.255.255.0 |
|-------------------------------------------------------------------------------------|
| 0x00A - 0x00D | Default gateway address                             | 192.168.0.1   |
|-------------------------------------------------------------------------------------|
| 0x00E - 0x00F | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x010 - 0x011 | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x012 - 0x015 | Connection code, 4 bytes                            | 0x00000000    |
|-------------------------------------------------------------------------------------|
| 0x016         | DOut connection mode.  This determines the DOut     | 0             |
|               | value when the connection status changes.           |               |
|               |   0 = no change                                     |               |
|               |   1 = apply specified tristate / latch values       |               |              
|-------------------------------------------------------------------------------------|
| 0x017         | DOut tristate mask for connection / disconnection   | 0xFF          |
|               | (bits set to 0 are outputs, bits set to 1 are no    |               |
|               | change                                              |               |
|-------------------------------------------------------------------------------------|
| 0x018         | Reserved                                            |               |
|-------------------------------------------------------------------------------------|
| 0x019         | DOut latch value when host is connected             | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x01A         | DOut latch value when host is disconnected          | 0x00          |
|-------------------------------------------------------------------------------------|
| 0x01B         | AOut channel 0 connection mode.  This determines    | 0             |
|               | the AOut value when the connection status changes.  |               |
|               |   0 = no change                                     |               |
|               |   1 = apply specified values to channel 0           |               |
|-------------------------------------------------------------------------------------|
| 0x01C - 0x01D | AOut channel 0 value when host is connected         | 32768         |
|-------------------------------------------------------------------------------------|
| 0x01E - 0x01F | AOut channel 0 value when host is disconnected      | 32768         |
|-------------------------------------------------------------------------------------|
| 0x020         | AOut channel 1 connection mode.  This determines    | 0             |
|               | the AOut value when the connection status changes.  |               |
|               |   0 = no change                                     |               |
|               |   1 = apply specified values to channel 1           |               |
|-------------------------------------------------------------------------------------|
| 0x021 - 0x022 | AOut channel 1 value when host is connected         | 32768         |
|-------------------------------------------------------------------------------------|
| 0x023 - 0x024 | AOut channel 1 value when host is disconnected      | 32768         |
|-------------------------------------------------------------------------------------|
| 0x025 - 0x1FF | Reserved                                            |               |
|=====================================================================================|

Note: The settings do not take effect until after device is reset or power cycled.


    User memory map
|=================================================================|
|    Address     |        Value                                   |
|=================================================================|
| 0x000 - 0x3FF  | Available for UL use                           |
|=================================================================|
"""

class E_1608:
  # Digital I/O Commands
  CMD_DIN_R             = 0x00   # Read DIO pins
  CMD_DOUT_R            = 0x02   # Read DIO latch value
  CMD_DOUT_W            = 0x03   # Write DIO latch value
  CMD_DCONF_R           = 0x04   # Read DIO configuration value
  CMD_DCONF_W           = 0x05   # Write DIO configuration value

  # Analog Input Commands
  CMD_AIN               = 0x10  # Read analog input channel
  CMD_AIN_SCAN_START    = 0x11  # Start analog input scan
  CMD_AIN_SCAN_STOP     = 0x13  # Stop analog input scan
  CMD_AIN_QUEUE_R       = 0x14  # Read analog gain queue
  CMD_AIN_QUEUE_W       = 0x15  # Write analog gain queue

  # Analog Output Commands
  CMD_AOUT_R            = 0x20  # Read analog output channel
  CMD_AOUT_W            = 0x21  # Write analog output channel

  # Counter Commands
  CMD_COUNTER_R         = 0x30  # Read event counter
  CMD_COUNTER_W         = 0x31  # Write event counter

  # Memory Command
  CMD_CAL_MEMORY_R      = 0x40  # Read calibration memory
  CMD_CAL_MEMORY_W      = 0x41  # Write calibraion memory
  CMD_USER_MEMORY_R     = 0x42  # Read user memory
  CMD_USER_MEMORY_W     = 0x43  # Write user memory
  CMD_SETTINGS_MEMORY_R = 0x44  # Read settings memory
  CMD_SETTINGS_MEMORY_W = 0x45  # Write settings memory
  CMD_BOOT_MEMORY_R     = 0x46  # Read bootloader memory
  CMD_BOOT_MEMORY_W     = 0x47  # Write bootloader memory

  # Miscellaneous Commands
  CMD_BLINKLED       = 0x50  # Blink the LED
  CMD_RESET          = 0x51  # Reset the device
  CMD_STATUS         = 0x52  # Read the device status
  CMD_NETWORK_CONF   = 0x54  # Read device network configuration

  def __init__(self, device):
    self.device = device        # inherit values from mccEthernetDevice

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept

    self.table_AInDF = [table(), table(), table(), table()]
    self.table_AInSE = [table(), table(), table(), table()]
    self.table_AOut =  [table(), table()]

    # Analog Input Calibration, differential 0x000 - 0x01C
    address = 0x0
    for i in range(NGAINS):
      self.table_AInDF[i].slope, = unpack('f', self.CalMemory_R(address, 4))
      address += 4
      self.table_AInDF[i].intercept, = unpack('f', self.CalMemory_R(address, 4))
      address += 4

    # Analog Input Calibration, single ended 0x20 - 0x03C
    address = 0x20
    for i in range(NGAINS):
      self.table_AInSE[i].slope, = unpack('f', self.CalMemory_R(address, 4))
      address += 4
      self.table_AInSE[i].intercept, = unpack('f',self.CalMemory_R(address, 4))
      address += 4

    # Analog Output Gain Tables
    address = 0x40
    for i in range(NCHAN_AOUT):
      self.table_AOut[i].slope, = unpack('f', self.CalMemory_R(address, 4))
      address += 4
      self.table_AOut[i].intercept, = unpack('f',self.CalMemory_R(address, 4))
      address += 4

    # create the gain queue
    self.queue = bytearray(17)

    # get the MAC address
    self.MACaddress()

    return

  #################################
  #     Digital I/O Commands      #
  #################################

  def DIn(self):
    """
    This command reads the current state of the DIO pins.  A 0 in a
    bit position indicates the corresponding pin is reading a low
    state, and a 1 indicates a high state.
    """
    
    dataCount = 0
    replyCount = 1
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DIN_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('DIn: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = r_buffer[MSG_INDEX_DATA]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DOut_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value
    
  def DOut_R(self):
    """
    This command reads the DIO output latch value.  The factory power
     on default is all zeros. A 0 in a bit position indicates the
     corresponding pin driver is low, a 1 indicates it is high.
    """
    
    dataCount = 0
    replyCount = 1
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DOUT_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('DOut_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = r_buffer[MSG_INDEX_DATA]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DOut_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value

  def DOut(self, value):
    """
    This command writes the DIO latch value.  The factory power on
    default is all ones (pins are floating). Writing a 0 to a bit will set
    the corresponding pin driver low, writing a 1 sets it high.
    """
    
    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DOUT_W
    s_buffer[MSG_INDEX_DATA]           = value
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('DOut: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DOut E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  def DConfig_R(self):
    """
    This command reads the DIO configuration value.  A 1 in a bit
    position indicates the corresponding pin is set to an input, a 0
    indicates it is set to an output.  The power on devault is all 1 (input)
    """
    
    dataCount = 0
    replyCount = 1
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DCONF_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('DConfig_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = r_buffer[MSG_INDEX_DATA]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DConfig_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value

  def DConfig_W(self, value):
    """
    This command writes the configuration value.  A 1 in a bit
    position sets the corresponding pin to an input, a 0 sets it to an 
    output.  The power on default is all ones (input)
    """
    
    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DCONF_W
    s_buffer[MSG_INDEX_DATA]           = value
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('DConfig_W: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DConfig_W E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

      
  #################################
  #     Analog Input Commands     #
  #################################

  def AIn(self, channel, gain):
    """
    This command reads the value of an analog input channel.  This command will
    not return valid data if AIn scan is currently running.

     channel 0-7  single ended
     channel 8-11 differential
    """

    dataCount = 2
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AIN
    s_buffer[MSG_INDEX_DATA]           = channel
    s_buffer[MSG_INDEX_DATA+1]         = gain
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('AIn: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = (r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AIn E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return

    if channel < DF:    # single ended
      data = round(float(value)*self.table_AInSE[gain].slope + self.table_AInSE[gain].intercept)
    else:               # differential
      data = round(float(value)*self.table_AInDF[gain].slope + self.table_AInDF[gain].intercept)
    if (data >= 65536):
      value = 65535
    elif (data < 0):
      value = 0
    else:
      value = data

    return value

  def AInScanStart(self, count, frequency, options):
    """
    This command starts an analog input scan.  The channel ordering
    and number of channels per scan is set by the channel gain queue
    which must first be set with the AInQueue_W command.  This
    command will respond with the busy error if an AIn scan is
    currently running.
  
    The pacer rate is set by an internal 32-bit timer running at a
    base rate of 80 MHz.  The timer is controlled by pacer_period.
    This value is the period of the scan and the A/D is clocked at
    this rate.  A pulse will be ouput at the AICKO pin at every
    pacer_period interval.  The equation for calculating
    tracer_period is:
  
       pacer_period = [80 MHz / (sample frequency)] -1
  
    If pacer_period is set to 0, the device does not generate an A/D
    clock.  It uses the AICKI pin as an input and the user must
    provice the pacer source.  The A/D acquires data on every rising
    edge of the pacer clock: the maximum allowable input frequency is
    250 kHz.
  
    The data is read and sent to the host using the AInScan data TCP
    port.  The device checks for a connection on this port when
    AInScanStart is called and will return an error code if it is not
    connected.  The scan will not start until the command reply ACK
    is received; see the Ethernet Communication Mechanism section for
    more details.
  
    Scan data will be acquired until an overrun occurs, the specified
    count is reached, or an AInScanStop command is sent. The scan
    data will be in the format:
  
    First channel sample 0: second channel sample 0: .. : last channel sample 0
    First channel sample 1: second channel sample 1: .. : last channel sample 1
    ...
    First channel sample n: second channel sample n: .. : last channel sample n
  
    If the host does not receive the data in a timely manner (due to
    a communications error, overrun, etc.) it can check the status of
    the scan with the Status command.  Any data in the scan data TCP
    buffer will be sent every 40ms or when the MTU size is reached.
  
    The external trigger may be used to start the scan.  If enabled,
    the device will wait until the appropriate trigger condition is
    detected then betgin sampling data at the specified rate.  No
    data will be available until the trigger is detected.
   
       count:      The total number of scans to acquire, 0 for continuous scan
       frequency:  the sampling frequency.  Use 0 for external clock.
       options:    Bit field that controls scan options
                   bits 0-1:   Reserved
                   bits 2-4:   Trigger setting:
                               0: no trigger
  		  	       1: Edge/rising
  			       2: Edge / falling
  			       3: Level / high
  			       4: Level / low
  	  	   bit 5:      Reserved
  		   bit 6:      Reserved
  		   bit 7:      Reserved
    """
  
    dataCount = 9
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    if (frequency > 250000.):
      frequency = 250000.     # 250kHz maximum
    if (frequency < 0.) :
      return False

    if (frequency == 0.):
      pacer_period = 0
    else:
      pacer_period = round((80.E6/frequency) - 1.0)

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AIN_SCAN_START
    s_buffer[MSG_INDEX_DATA]           = count & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (count >> 8)  & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = (count >> 16) & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count >> 24) & 0xff
    s_buffer[MSG_INDEX_DATA+4]         = pacer_period  & 0xff
    s_buffer[MSG_INDEX_DATA+5]         = (pacer_period >> 8)  & 0xff
    s_buffer[MSG_INDEX_DATA+6]         = (pacer_period >> 16) & 0xff
    s_buffer[MSG_INDEX_DATA+7]         = (pacer_period >> 24) & 0xff
    s_buffer[MSG_INDEX_DATA+8]         = options & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    # create a scan socket
    self.device.scan_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.device.scan_sock.connect((self.device.address, SCAN_PORT))

    self.device.scan_timeout = 0.1 + count/frequency
    self.device.sock.settimeout(.01)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('AInScanStart: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanStart E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return 

    # send ACK to start scan
    data = bytearray(1)
    data[0] = 0x0
    self.device.sendMessage(data)  # send a single byte

  def AInScanRead(self, nScan, nChan):
    count = 2*nScan*nChan   # total number of 2-byte samples
    self.device.scan_sock.settimeout(self.device.scan_timeout)
    data = bytearray(count)
    corrected_data = []
    value = []
    bytesReceived = 0

    while (bytesReceived  < count):
      try:
        data = self.device.scan_sock.recv(count - bytesReceived)
      except socket.timeout:
        raise TimeoutError('AInScanRead: timeout error.')
      bytesReceived += len(data)
      value[bytesReceived:] = unpack('H'*(len(data)//2), data)  # convert to unsigned short

    for i in range(nScan):                          # scan count
      for j in range(nChan):
        k = i*nChan + j                             # sample number
        channel = self.queue[2*j+1]
        gain = self.queue[2*j+2]
        if (channel < DF):                          # single ended
          corrected_data.append(round(value[k]*self.table_AInSE[gain].slope + self.table_AInSE[gain].intercept))
        else:
          corrected_data.append(round(value[k]*self.table_AInDF[gain].slope + self.table_AInDF[gain].intercept))
        if corrected_data[k] > 65536:
          corrected_data[k] = 65535
        elif corrected_data[k] < 0:
          corrected_data[k] = 0

    return corrected_data
                      
  def AInQueue_R(self):
    """
      This command reads the analog input scan channel gain queue
      count       the number of queue entries, max 8
      channel_0   the channel number of the first queue element [0-11]
      range_0     the range number of the first queue element   [0-3]
       ...
      channel_n   the channel number of the last queue element [0-11]
      range_n     the range number of the last queue element   [0-3]
    """
    dataCount = 0
    replyCount = 2*self.queue[0]+1
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AIN_QUEUE_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('AInQueue_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           for i in range(replyCount):
             self.queue[i] = r_buffer[MSG_INDEX_DATA+i]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInQueue_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AInQueue_W(self):
    """
    This command writes the analog input scan channel gain queue
       count       the number of queue entries, max 8
       channel_0   the channel number of the first queue element [0-11]
       range_0     the range number of the first queue element   [0-3]
       ...
       channel_n   the channel number of the last queue element [0-11]
       range_n     the range number of the last queue element   [0-3]
    """

    dataCount = 2*self.queue[0]+1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    if (dataCount > 17):
      return False

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AIN_QUEUE_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    for i in range(dataCount):
      s_buffer[MSG_INDEX_DATA+i]       = self.queue[i]
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('AInQueue_W: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInQueue_W E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AInScanStop(self, close_socket=0):
    """
    This command stops the analog input scan (if running).  It will clear the
    scan data FIFO.
    
       close_socket:    1 = close and reopen the data socekt
    """

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AIN_SCAN_STOP
    s_buffer[MSG_INDEX_DATA]           = close_socket
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('AInScanStop: timout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanStop E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  
  ########################
  #     Analog Output    #
  ########################

  def AOut_R(self, value):
    """
    This command reads the value of the analog output channels
    
      value[0]   the current value for analog output channel 0
      value[1]   the current value for analog output channel 1
    """
    
    dataCount = 0
    replyCount = 4
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    value = []
  
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AOUT_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('AOut_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AOut_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return

    data = unpack_from('H'*2, r_buffer, MSG_INDEX_DATA)
    value[0] = round(data[0]*self.table_AOut[0].slope + self.table_AOut[0].intercept)
    value[1] = round(data[1]*self.table_AOut[1].slope + self.table_AOut[1].intercept)
  
    return value

  def AOut(self, channel, value):
    """
    This command writes the value of the analog output channel.
    
      channel: the channel to write (0-1)
      value:   the value to write (0-4095)
    """

    dataCount = 3
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    value = round(value*self.table_AOut[channel].slope + self.table_AOut[channel].intercept)

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AOUT_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_DATA]           = channel
    s_buffer[MSG_INDEX_DATA+1]         = value & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = (value>>8) & 0xff
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('AOut: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AOut E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self, count=1):
    """
     This command will blink the device power LED "count" times
    """
    
    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_BLINKLED
    s_buffer[MSG_INDEX_DATA]           = count
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('Blink: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in blink E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def Reset(self):
    """
    This command resets the device
    """
    
    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_RESET
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('Reset: timout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in reset E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def Status(self):
    """
    This command reads the device status
      bit 0:     1 = data socket is open, 0 = data socket is closed
      bit 1:     1 = AIn scan running
      bit 2:     1 = AIn scan overrun
      bits 3-15: Reserved
    """

    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_STATUS
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise TimeoutError('Status: timout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           status = r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8 & 0xff)  # status
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in status E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return status

  def NetworkConfig(self):
    """
    This command reads the current network configuration.  Returns tuple
     (ip_address, subnet_mask, gateway_address)
    """
      
    dataCount = 0
    replyCount = 12
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_NETWORK_CONF
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('NetworkConfig: timout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           data = unpack_from('III', r_buffer, MSG_INDEX_DATA)
           value = (socket.inet_ntoa(pack('L', data[0])), socket.inet_ntoa(pack('L', data[1])), socket.inet_ntoa(pack('L', data[2])))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in networkConfig E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return value
  
  def FirmwareUpgrade(self):
    """
    This command causes the device to reset and enter the bootloader
    for a firmware upgrade.  It erases a portion of the program memory so
    the device must have firmware downloaded through the bootloder before
    it can be used again.
    """
    
    dataCount = 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FIRMWARE
    s_buffer[MSG_INDEX_DATA]           = 0xad                     # key
    s_buffer[MSG_INDEX_DATA+1]         = 0xad                     # key
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('FirmwareUpgrade: timout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in firmwareUpgrade E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  #################################
  #       Counter Commands        #
  #################################
  def Counter(self):
    """
    This command read the event counter
    """

    dataCount = 0
    replyCount = 4
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_COUNTER_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('Counter: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value ,= unpack_from('I', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Counter E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value
  
  def ResetCounter(self):
    """
    This command resets the event counter.  On a write, the
    counter will be reset to 0.
    """
    
    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_COUNTER_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('ResetCounter: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in restCounter_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  #################################
  #       Memory  Commands        #
  #################################

  def CalMemory_R(self, address, count):
    """
    This command reads the nonvolatile calibration memory.  The cal memory is
    512 byes (address 0 - 0x1ff)
    """

    if (count > 512 or address > 0x1ff):
      return False

    dataCount = 4
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CAL_MEMORY_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff                # low byte
    s_buffer[MSG_INDEX_DATA+1]         = (address >> 8) & 0xff         # high byte
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff                  # low byte
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff             # high byte
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('CalMemory_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in CalMemory_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value

  def CalMemory_W(self, address, count, data):
    """
    This command writes the nonvolatile calibration memory.  The cal
    memory is 512 bytes (address 0 - 0xff).  The cal memory should
    only be written during factory calibration and setup and has an
    additional lock mechanism to prevent inadvertent writes.  To
    enable writes to the cal memory, first write the unlock code
    0xAA55 to address 0x200.  Writes to the entire memory range are
    then possible.  Write any other value to address 0x200 to lock
    the mamory after writing.  The amount of data to be written is
    inferred from the frame count -2.
    """

    if (count > 512 or address > 0xff):
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CAL_MEMORY_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('CalMemory_W: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in CalMemory_W E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def UserMemory_R(self, address, count):
    """
    This command reads the nonvolatile user memory.  The user memory is
    1024 bytes (address 0 - 0x3ff)

      address: the start address for reading (0-0x3ff)
      count:   the number of bytes to read (max 512 due to protocol)
    """

    if (count > 512 or address > 0x3ff):
      return False

    dataCount = 4
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_USER_MEM_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('UserMemory_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder='little')
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in UserMemory_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value

  def UserMemory_W(self, address, count, data):
    """
    This command writes to the nonvolatile user memory.  The user memory
    is 1024 bytes (address 0 - 0x3ff).  The amount of data to be
    written is inferred from the frame count  - 2.  The maximum that
    can be written in one transfer is 512 bytes.
    """

    if (count > 512 or address > 0x3ff):
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_USER_MEMORY_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('UserMemory_W: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in UserMemory_W E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def SettingsMemory_R(self, address, count):
    """
    This command reads the nonvolatile settings memory.  The settings memory is
    512 bytes (address 0 - 0x1ff)
    
    address: the start address for reading (0-0x1ff)
    count:   the number of bytes to read (max 512 due to protocol)
    """

    if (count > 512 or address > 0x1ff):
      return False

    dataCount = 4
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_SETTINGS_MEMORY_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('SettingsMemory_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder='little')
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in SettingsMemory_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value

  def SettingsMemory_W(self, address, count, data):
    """
    This command writes to the nonvolatile settings memory.  The settings memory
    is 512 bytes (address 0 - 0x1ff).  The amount of data to be
    written is inferred from the frame count  - 2.  The maximum that
    can be written in one transfer is 512 bytes.  The settings will
    be implemented after a device reset.
    """

    if (count > 512 or address > 0x1ff):
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_SETTINGS_MEMORY_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('SettingsMemory_W: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in SettingsMemory_W E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  def BootloaderMemory_R(self, address, count):
    """
    This command reads the bootloader stored in nonvolatile FLASH
    memory.  The bootloader is located in program FLASH memory in two
    physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
    code and 0x1FC00000 - 0x1FC01FFF for C startup code and
    interrupts.  Reads may be performed at any time.
    
      address: the start address for reading (see above)
      count:   the number of bytes to read (max 512)
    """

    if (count > 512):
      return False

    dataCount = 4
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_BOOT_MEMORY_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('BootloaderMemory_R: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder='little')
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in BootloaderMemory_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return value

  def BootloaderMemory_W(self, address, count, data):
    """This command writes the bootloader stored in nonvolatile FLASH
    memory.  The bootloader is located in program FLASH memory in two
    physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
    code and 0x1FC00000 - 0x1FC01FFF for C startup code and
    interrupts.  Writes outside these ranges are ignored.  The
    bootloader memory is write protected and must be unlocked in
    order to write the memory.  The unlock proceedure is to write the
    unlock code 0xAA55 to address 0xFFFFFFFE.  Writes to the entire
    memory range are then possible.  Write any other value to address
    0xFFFFFFFE to lock the memory after writing.
    
    The FLASH memory must be erased prior to programming.  A bulk
    erase is perfomred by writing 0xAA55 to address 0x80000000 after
    unlocking the memory for write.  The bulk erase will require
    approximately 150ms to complete.  Once the erase is complete, the
    memory may be written; however, the device will not be able to
    boot unless it has a valid bootloader so the device shold not be
    reset until the bootloader is completely written and verified
    using BootloaderMemory_R().
    
    The writes are perfomred on 4-byte boundaries internally and it is
    recommended that the output data be sent in the same manner.  The
    amount of data to be written is inferred frolm the frame 
    count-2.
    """

    if (count > 512):
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_BOOT_MEMORY_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise TimeoutError('BootloaderMemory_W: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in BootloaderMemory_W E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def getMFGCAL(self):
    """
    get the manufacturers calibration data (timestamp) from the
    Calibration memory
    """

    # get the year (since 2000)
    address = 0x50
    data ,= unpack('B', self.CalMemory_R(address, 1))
    year  = 2000+data

    # get the month
    address = 0x51
    month ,= unpack('B', self.CalMemory_R(address, 1))

    # get the day
    address = 0x52
    day ,= unpack('B', self.CalMemory_R(address, 1))

    # get the hour
    address = 0x53
    hour ,= unpack('B', self.CalMemory_R(address, 1))
    
    # get the minute
    address = 0x54
    minute ,= unpack('B', self.CalMemory_R(address, 1))

    # get the second
    address = 0x55
    second ,= unpack('B', self.CalMemory_R(address, 1))

    mdate = datetime(year, month, day, hour, minute, second)

    return mdate

  def MACaddress(self):
    """
    Gets the MAC address
    """

    # get lowest thress byes of MAC address
    address = 0x1fd
    value =  self.CalMemory_R(address, 3)
    self.device.MAC = ((0x00802f) << 24) + (value[0]<<16) + (value[1]<<8) + value[2]
    return self.device.MAC

  @staticmethod
  def volts(value, gain):
    # converts raw values to volts
    if gain == BP_10V:
      volt = (value - 0x8000)*10.0/32768
    elif gain == BP_5V:
      volt = (value - 0x8000)*5.0/32768
    elif gain == BP_2V:
      volt = (value - 0x8000)*2.0/32768
    elif gain == BP_1V:
      volt = (value - 0x8000)*1.0/32768
    else:
      raise ValueError('volts: unkown gain.')
      return False
    return volt

  @staticmethod
  def valueAOut(volts):
    # converts volts to a 16 bit raw value for +/- 10V output
    if (volts >= 10.0):
      return 0xffff
    elif (volts <= -10.00):
      return 0x0
    else:
      return round(volts*32768/10. + 0x8000)

