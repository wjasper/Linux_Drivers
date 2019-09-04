#! /usr/bin/python3
#
# Copyright (c) 2019 Warren J. Jasper <wjasper@ncsu.edu>
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

import bluetooth
import time
from struct import *
from mccBluetooth import *
from datetime import datetime

BTH1208LS_PID = 6883

class BTH_1208LS:
  """
    Settings memory map
  |===========================================================================================|
  |    Address    |        Value                                                              |
  |===========================================================================================|
  | 0x000         | DOut Bluetooth connection mode.  This determines the DOut value           |
  |               | when the Bluetooth connection status changes (not applicable in USB mode) |
  |               |    0 = no change                                                          |
  |               |    1 = apply specified values                                             |
  |-------------------------------------------------------------------------------------------|
  | 0x001         | DOut value when Bluetooth connected                                       |
  |-------------------------------------------------------------------------------------------|
  | 0x002         | DOut value when Bluetooth disconnected                                    |
  |-------------------------------------------------------------------------------------------|
  | 0x003         | AOut channel 0 Bluetooth connection mode.  This determines the AOut value |
  |               | when the Bluetooth connection status changes (not applicable in USB mode) |
  |               |    0 = no change                                                          |
  |               |    1 = apply specified values                                             |
  |-------------------------------------------------------------------------------------------|
  | 0x004 - 0x005 | AOut channel 0 value when Bluetooth connected                             |
  |-------------------------------------------------------------------------------------------|
  | 0x006 - 0x007 | AOut channel 0 value when Bluetooth disconnected                          |
  |-------------------------------------------------------------------------------------------|
  | 0x008         | AOut channel 1 Bluetooth connection mode.  This determines the AOut value |
  |               | when the Bluetooth connection status changes (not applicable in USB mode) |
  |               |    0 = no change                                                          |
  |               |    1 = apply specified values                                             |
  |-------------------------------------------------------------------------------------------|
  | 0x009 - 0x00A | AOut channel 1 value when Bluetooth connected                             |
  |-------------------------------------------------------------------------------------------|
  | 0x00B - 0x00C | AOut channel 1 value when Bluetooth disconnected                          |
  |-------------------------------------------------------------------------------------------|
  | 0x00C         | Auto shutdown timer value in minutes (0 = no auto shutdown).  When this   |
  |               | is used the device will automatically power down when powered by          |
  |               | batteries and no Bluetooth connection is present for this amount of time. |
  |-------------------------------------------------------------------------------------------|
  | 0x00E         | Allow charging when Bluetooth connected                                   |
  |               |   0 = do not allow                                                        |
  |               |   1 = allow                                                               |
  |-------------------------------------------------------------------------------------------|
  | 0x00F - 0x3FF | Unused                                                                    |
  |===========================================================================================|
  """


  # Commands and Report ID for BTH-1208LS
  # Digital I/O
  DIN_R                = (0x00) # Read the current state of the DIO pins
  DOUT_R               = (0x02) # Read DIO latch value
  DOUT_W               = (0x03) # Write DIO latch value
  # Analog Input Commands
  AIN                    = (0x10) # Read analog input channel
  AIN_SCAN_START         = (0x11) # Start analog input scan
  AIN_SCAN_SEND_DATA     = (0x12) # Read data from the analog input scan FIFO
  AIN_SCAN_STOP          = (0x13) # Stop analog input scan
  AIN_CONFIG_R           = (0x14) # Read analog input configuration
  AIN_CONFIG_W           = (0x15) # Write analog input configuration
  AIN_SCAN_CLEAR_FIFO    = (0x16) # Clear the analog input scan FIFO
  AIN_SCAN_RESEND_DATA   = (0x18) # Resend data from the analog input scan
  # Analog Output Commands
  AOUT_R                 = (0x20) # Read analog output channel
  AOUT_W                 = (0x21) # Analog output channel
  #  Counter Commands
  COUNTER_R              = (0x30) # Read event counter
  COUNTER_W              = (0x31) # Reset event counter
  # Memory Commands
  CAL_MEMORY_R           = (0x40) # Read calibration memory
  USER_MEMORY_R          = (0x42) # Read user memory
  USER_MEMORY_W          = (0x43) # Write user memory
  SETTINGS_MEMORY_R      = (0x44) # Read settings memory
  SETTINGS_MEMORY_W      = (0x45) # Write settings memory
  # Miscellaneous Commands
  BLINK_LED              = (0x50) # Blink the LED
  RESET                  = (0x51) # Reset the device
  STATUS                 = (0x52) # Read device status
  SERIAL                 = (0x54) # Read serial number
  PING                   = (0x55) # Check device communications
  FIRMWARE_VERSION       = (0x56) # Read the firmware version
  RADIO_FIRMWARE_VERSION = (0x5A) # Read the radio firmware version
  BATTERY_VOLTAGE        = (0x58) # Read battery voltage

  NGAINS                 =  8     # max number of input gain levels (differential mode only)
  NCHAN_DE               =  4     # max number of A/D differential channels
  NCHAN_SE               =  8     # max number of A/D single-ended channels
  NCHAN_AOUT             =  2     # max number of D/A 12 bit 0-2.5V output channels

  # Analog Input
  SINGLE_ENDED    = 0
  DIFFERENTIAL    = 1

  # Analog Input Scan Options
  DIFFERENTIAL_MODE       = (0x2)
  SINGLE_ENDED_MODE       = (0x0)
  NO_TRIGGER              = (0x0)
  TRIG_EDGE_RISING        = (0x1 << 2)
  TRIG_EDGE_FALLING       = (0x2 << 2)
  TRIG_LEVEL_HIGH         = (0x3 << 2)
  TRIG_LEVEL_LOW          = (0x4 << 2)
  RETRIGGER_MODE          = (0x1 << 5)

  # Ranges
  BP_20V   = 0x0      # +/- 20 V
  BP_10V   = 0x1      # +/- 10V
  BP_5V    = 0x2      # +/- 5V
  BP_4V    = 0x3      # +/- 4V
  BP_2_5V  = 0x4      # +/- 2.5V
  BP_2V    = 0x5      # +/- 2V
  BP_1_25V = 0x6      # +/- 1.25V
  BP_1V    = 0x7      # +/- 1V
  UP_2_5V  = 0x8      # 0-2.5V

  # Status bit values
  AIN_SCAN_RUNNING   = (0x1 << 1)
  AIN_SCAN_OVERRUN   = (0x1 << 2)
  NO_BATTERY         = (0x0)
  FAST_CHARGE        = (0x1 << 8)
  MAINTENANCE_CHARGE = (0x2 << 8)
  FAULT_CHARGING     = (0x3 << 8)
  DISABLE_CHARGING   = (0x4 << 8)

  voltage = 0
  status = 0
  nDelay = 0.0
  frequency = 0.0
  options = 0       
  nChan = 0          # number of channels in the scan
  continuous_mode = False

  def __init__(self, device):
    self.device = device        # inherit values from mccBluetoothDevice

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #
    #  table_AInDE[NCHAN_DE][NGAIN]
    self.table_AInDE  = [[table(), table(), table(), table(), table(), table(), table(), table()],\
                         [table(), table(), table(), table(), table(), table(), table(), table()],\
                         [table(), table(), table(), table(), table(), table(), table(), table()],\
                         [table(), table(), table(), table(), table(), table(), table(), table()]]

    #  table_AInSE = [NCHAN_SE]                         
    self.table_AInSE = [table(), table(), table(), table(), table(), table(), table(), table()]

    self.BuildGainTable_DE()
    self.BuildGainTable_SE()

  def BuildGainTable_DE(self):
    """
    Builds a lookup table of differential mode calibration
    coefficents to translate values into voltages: The calibration
    coefficients are stored in onboard FLASH memory on the device in
    IEEE-754 4-byte floating point values.

       calibrated code = code * slope + intercept
    """
    address = 0x0
    for gain in range(self.NGAINS):
      for chan in range(self.NCHAN_DE):
        self.table_AInDE[chan][gain].slope, = unpack('f', self.CalMemoryR(address, 4))
        address += 4
        self.table_AInDE[chan][gain].intercept, = unpack('f', self.CalMemoryR(address, 4))
        address += 4

  def BuildGainTable_SE(self):
    """
    Builds a lookup table of differential mode calibration
    coefficents to translate values into voltages: The calibration
    coefficients are stored in onboard FLASH memory on the device in
    IEEE-754 4-byte floating point values.

       calibrated code = code * slope + intercept
    """

    address = 0x100
    for chan in range(self.NCHAN_SE):
      self.table_AInSE[chan].slope, = unpack('f', self.CalMemoryR(address, 4))
      address += 4
      self.table_AInSE[chan].intercept, = unpack('f', self.CalMemoryR(address, 4))
      address += 4

  def CalDate(self):
    """
    get the manufacturers calibration data (timestamp) from the
    Calibration memory
    """

    # get the year (since 2000)
    address = 0x200
    data ,= unpack('B', self.CalMemoryR(address, 1))
    year  = 2000+data

    # get the month
    address = 0x201
    month ,= unpack('B', self.CalMemoryR(address, 1))

    # get the day
    address = 0x202
    day ,= unpack('B', self.CalMemoryR(address, 1))

    # get the hour
    address = 0x203
    hour ,= unpack('B', self.CalMemoryR(address, 1))
    
    # get the minute
    address = 0x204
    minute ,= unpack('B', self.CalMemoryR(address, 1))

    # get the second
    address = 0x205
    second ,= unpack('B', self.CalMemoryR(address, 1))

    mdate = datetime(year, month, day, hour, minute, second)
    return mdate

  def volts(self, value, gain):
    volt = 0.0
    if (gain == self.BP_20V):
      volt = (value - 0x800)* 20.0/2048.
    elif (gain == self.BP_10V):
      volt = (value - 0x800)* 10.0/2048.
    elif (gain == self.BP_5V):
      volt = (value - 0x800)* 5.0/2048.
    elif (gain == self.BP_4V):
      volt = (value - 0x800)* 4.0/2048.
    elif (gain == self.BP_2_5V):
      volt = (value - 0x800)* 2.5/2048.
    elif (gain == self.BP_2V):
      volt = (value - 0x800)* 2.0/2048.
    elif (gain == self.BP_1_25V):
      volt = (value - 0x800)* 1.25/2048.
    elif (gain == self.BP_1V):
      volt = (value - 0x800)* 1.0/2048.
    elif (gain == self.UP_2_5V):
      volt = value*2.5/4096.             # analog output
    else:
      print("Unknown range: ",gain)

    return volt

  #############################################
  #        Digital I/O Commands               #
  #############################################

  def DIn(self):
    """
    This command reads the current state of the DIO pins.  A 0 in a
    bit position indicates the correspoing pin is reading a low
    state, and a 1 indicates a high state.
    """
    
    dataCount = 0
    replyCount = 1
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.DIN_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID 
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('DIn: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = r_buffer[MSG_INDEX_DATA]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DIn BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return data

  def DOutR(self):
    """
    This command reads the DIO output latch value. The factory power
    on default is all 1 (pins are floating.) Writing a 0 to a bit
    drives it low, writing a 1 allows it to float.
    """

    dataCount = 0
    replyCount = 1
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.DOUT_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except:
      print("DOutR: Error in receiveMessage")
      return -1

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = r_buffer[MSG_INDEX_DATA]

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DOutR BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return data

      
  def DOut(self, value):
    """
    This command reads the DIO output latch value. The factory power
    on default is all 1 (pins are floating.) Writing a 0 to a bit
    drives it low, writing a 1 allows it to float.
    """

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.DOUT_W
    s_buffer[MSG_INDEX_DATA]           = value
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('DOut: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DOut BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    
  #############################################
  #        Analog Input Commands              #
  #############################################

  def AIn(self, channel, mode, gain):
    """
    This command reads the value of an analog input channel.  This
    command will result in a bus stall if an AInScan is currently
    running.  The range parameter is ignored if the mode is specified
    as single ended.

      channel: the channel to read (0-7)
      mode:    the input mode 0 - single ended, 1 - differential
      range:   the input range for the channel (0-7)
    """

    dataCount = 3
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_DATA]           = channel
    s_buffer[MSG_INDEX_DATA+1]         = mode
    s_buffer[MSG_INDEX_DATA+2]         = gain
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AIn: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        value = (r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AIn BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      
    if mode == self.SINGLE_ENDED:    # single ended
      data = round(float(value)*self.table_AInSE[channel].slope + self.table_AInSE[channel].intercept)
    else:               # differential
      data = round(float(value)*self.table_AInDE[channel][gain].slope + self.table_AInDE[channel][gain].intercept)
    if (data >= 65536):
      value = 65535
    elif (data < 0):
      value = 0
    else:
      value = data
    return value
  

  def AInScanStart(self, count, retrig_count, frequency, channels, options):
    """
    This command starts an analog input scan. This command will
    respond with 0 if an AIn scan is currently running. The device
    will not generate an internal pacer faster than 50 kHz.

    The pacer rate is set by an internal 32-bit timer running at a
    base rate of 40 MHz. The timer is controlled by
    pacer_period. This value is the period of the scan and the A/Ds
    are clocked at this rate. A pulse will be output at the SYNC pin
    at every pacer_period interval if SYNC is configured as an
    output. The equation for calculating pacer_period is:
   
           pacer_period = [40 MHz / (sample frequency)] - 1 

    If pacer_period is set to 0 the device does not generate an A/D
    clock. It uses the SYNC pin as an input and the user must provide
    the pacer source. The A/Ds acquire data on every rising edge of
    SYNC; the maximum allowable input frequency is 50 kHz.  

    The scan will not begin until this command is sent and any trigger
    conditions are met. Data will be acquired until an overrun occurs,
    the specified count is reached, or an AInScanStop command is sent.

    The data is read using the AInScanSendData command. The data will
    be in the format:

     lowchannel sample 0 : lowchannel + 1 sample 0 : … : hichannel sample 0 
     lowchannel sample 1 : lowchannel + 1 sample 1 : … : hichannel sample 1
      … 
     lowchannel sample n : lowchannel + 1 sample n : … : hichannel sample n 

    If the host does not receive the data in a timely manner (due to
    a communications error, etc.) it can issue the AInScanResendData
    command. The device will resend the last packet that was
    transmitted. The device does not remove the sent data from its
    FIFO until a new AInScanSendData command is received. This keeps
    the data available for a resend. The host must send an
    AInScanStop command at the end of a finite scan to let the device
    know that the final packet was received successfully and allow
    the scan to end.

    The external trigger may be used to start the scan. If enabled,
    the device will wait until the appropriate trigger condition is
    detected then begin sampling data at the specified rate. No data
    will be available until the trigger is detected. In retrigger
    mode the trigger will be automatically rearmed and the scan will
    restart after retrig_count samples have been acquired. The count
    parameter specifies the total number of samples to acquire and
    should be >= retrig_count. Specifying 0 for count causes
    continuous retrigger scans.

    Overruns are indicated in the status field of the AInScanDataRead
    command response. The host may also read the status to verify.

     count:         the total number of samples to acquire, 0 for continuous scan
     retrig_count:  the number of samples to acquire for each trigger in retrigger mode
     pacer_period:  the pacer timer period (0 for external clock)
     channels:      bit field that selects the channels in the scan, upper 4 bits ignored in differential mode. 
        ---------------------------------------------------------
        | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
        ---------------------------------------------------------
        | Ch7  |  Ch6 | Ch5  | Ch4  | Ch3  | Ch2  | Ch1  | Ch 0 |
        ---------------------------------------------------------

	options:  bit field that controls scan options
	    bit 0: Reserved
	    bit 1: 1 = differential mode, 0 = single ended mode
	    bits 2-4: Trigger setting:
	      0: no trigger
  	      1: Edge / rising
	      2: Edge / falling
	      3: Level / high
	      4: Level / low
            bit 5: 1 = retrigger mode, 0 = normal trigger mode
	    bit 6: Reserved
	    bit 7: Reserved
    """

    if frequency > 50000.:
      frequency = 50000.
    if frequency > 0.:
      pacer_period = round((40.E6 / frequency) - 1)
    else:
      pacer_period = 0
    self.frequency = frequency
    self.options = options
    if count == 0:
      self.continuous_mode = True
    else:
      self.continuous_mode = False

    self.nChan = 0
    if (options & self.DIFFERENTIAL_MODE) == self.DIFFERENTIAL_MODE:
      for i in range(self.NCHAN_DE):
        if (channels & (0x1 << i)) != 0x0:
          self.nChan += 1
    else:
      for i in range(self.NCHAN_SE):
        if (channels & (0x1 << i)) != 0x0:
          self.nChan += 1

    dataCount = 14
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_SCAN_START
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_DATA]           = count & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (count >> 8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = (count >> 16) & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count >> 24) & 0xff
    s_buffer[MSG_INDEX_DATA+4]         = retrig_count & 0xff
    s_buffer[MSG_INDEX_DATA+5]         = (retrig_count >> 8) & 0xff
    s_buffer[MSG_INDEX_DATA+6]         = (retrig_count >> 16) & 0xff
    s_buffer[MSG_INDEX_DATA+7]         = (retrig_count >> 24) & 0xff
    s_buffer[MSG_INDEX_DATA+8]         = pacer_period & 0xff
    s_buffer[MSG_INDEX_DATA+9]         = (pacer_period >> 8) & 0xff
    s_buffer[MSG_INDEX_DATA+10]        = (pacer_period >> 16) & 0xff
    s_buffer[MSG_INDEX_DATA+11]        = (pacer_period >> 24) & 0xff
    s_buffer[MSG_INDEX_DATA+12]        = channels
    s_buffer[MSG_INDEX_DATA+13]        = options
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AInScanStart: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanStart BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return result

  def AInScanRead(self, nScan):
    nSamples = nScan*self.nChan
    data = []

    while nSamples > 0:
      if nSamples > 127:
        self.nDelay = 127./(self.frequency)
        time.sleep(self.nDelay*.95)                  # give system time to collect data
        data.extend(self.AInScanSendData(127))
        nSamples -= 127
      else:
        self.nDelay = (nSamples)/(self.frequency)
        time.sleep(self.nDelay*.95)                  # give system time to collect data
        data.extend(self.AInScanSendData(nSamples))
        if self.continuous_mode == False:
          self.AInScanStop()
          self.AInScanClearFIFO()
        return data

  def AInScanSendData(self, count):
    """
    This command reads data from the scan FIFO. The device will
    return all data currently in the FIFO up to the maximum amount
    allowed in a frame (255 bytes / 127 samples). Incrementing frame
    IDs are recommended in order to make use of the AInScanResendData
    command on an error.

      count: number of samples to send
    """

    dataCount = 0
    replyCount = count*2
    if (replyCount > 255):
      replyCount = 255      # 255 the maximum number of bytes transmitted in a frame
      print("AInScanSendData: count greater than 255\n")
      raise ResultError
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_SCAN_SEND_DATA
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except:
      print('AInScanResendError: receiveMessage')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = unpack('H'*count, r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount])
    else:
      # We did not get enough data.
      data = self.AInScanResendData(count)
      if len(data) == count:
        result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanSendData BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return data

  def AInScanResendData(self, count):
    """
    This command resends the previous scan data that matches the
    specified frame ID. This is used when the host doesn’t receive a
    response from the device after sending AInScanSendData. The
    device only buffers one most recently sent response and frame ID;
    if the frame ID matches the device assumes that its response was
    not received by the host and resends the previous data. If the
    specified frame ID doesn’t match the most recent frame ID the
    device assumes it did not receive the last AInScanSendData
    command and sends new data from the FIFO.
    """

    dataCount = 0
    replyCount = count*2
    if (replyCount > 255):
      replyCount = 255          # 255 the maximum number of bytes transmitted in a frame
    self.device.frameID -= 1    # decrement to the previous frame that needs resending.
    
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_SCAN_RESEND_DATA
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except:
      print('AInScanResendData: receiveMessage.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = unpack('H'*count, r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount])
    else:
      print("AInScanResendData: Error in length of return buffer.", len(r_buffer), MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount)
      error = MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount - len(r_buffer)
      data = unpack('H'*round((replyCount-error)/2) , r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+(replyCount-error)])
      return data
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanResendData BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return data

  def AInScanStop(self):
    """
    This command stops the analog input scan (if running)
    """

    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_SCAN_STOP
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AInScanStop: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanStop BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AInConfigR(self):
    """
    This command reads the analog input range configuration for
    AInScan in differential mode.
    """

    dataCount = 0
    replyCount = 4
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_CONFIG_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AInConfigR: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInConfigR BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return data

  def AInConfigW(self, ranges):
    """  
    This command writes the analog input range configuration for
    AInScan in differential mode.  This command will result in an
    error respone if an AIn scan is currently running.

        ranges: 4 channel ranges, each byte corresponds to an input channel.
                These values are ignored in single ended mode (only 1 range available).
    """
    
    dataCount = 4
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_CONFIG_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_DATA]           = ranges[0] & 0Xff
    s_buffer[MSG_INDEX_DATA+1]         = ranges[1] & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = ranges[2] & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = ranges[3] & 0xff
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AInConfigW: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInConfigW BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AInScanClearFIFO(self):
    """
    This command clears the scan data FIFO
    """
    
    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AIN_SCAN_CLEAR_FIFO
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AInScanClearFIFO: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AInScanClearFIFO BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  
  #############################################
  #        Analog Output Commands             #
  #############################################

  def AOutR(self):
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

    s_buffer[MSG_INDEX_COMMAND]        = self.AOUT_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AOutR: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = unpack_from('H'*2, r_buffer, MSG_INDEX_DATA)

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AOutR BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return data

  def AOut(self, channel, value):
    """
    This command writes the value of the analog output channels

      channel: the channel to write (0-1)
      value:   the value to write (0-4095)
    """

    dataCount = 3
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.AOUT_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_DATA]           = channel
    s_buffer[MSG_INDEX_DATA+1]         = value & 0xff          # low byte
    s_buffer[MSG_INDEX_DATA+2]         = (value>>8) & 0xff     # high byte
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('AOut: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AOut BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  #############################################
  #           Counter Commands                #
  #############################################

  def Counter(self):
    """
    The command reads the event counter.
    """
    dataCount = 0
    replyCount = 4
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.COUNTER_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('Counter: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        value ,= unpack_from('I', r_buffer, MSG_INDEX_DATA)

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Counter BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

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

    s_buffer[MSG_INDEX_COMMAND]        = self.COUNTER_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('ResetCounter: timeout error.')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in RestCounter_R E-1608.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    

  #############################################
  #        Memory Commands                    #
  #############################################

  def CalMemoryR(self, address, count):
    """
    This command reads the nonvolatile calibration memory.  The cal
    memory is 768 bytes (address 0 - 0x2FF)
    """
 
    if (count > 255):
      print("CalMemoryR: count must be less than 256.")
      return False

    if (address > 0x2ff):
      print("CalMemoryR: address must be between 0x0-0x2FF.")
      return False

    dataCount = 3
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CAL_MEMORY_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff        # low byte
    s_buffer[MSG_INDEX_DATA+1]         = (address >> 8) & 0xff # high byte    
    s_buffer[MSG_INDEX_DATA+2]         = count
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except:
      print("Error in CalMemoryR BTH-1208LS receiveMessage")

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in CalMemoryR BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return data


  def UserMemoryR(self, address, count):
    """
    This command reads the nonvolatile user memory.  The user
    memory is 256 bytes (address 0 - 0xFF)
    """

    if (count > 255):
      print("UserMemoryR: count must be less than 256.")
      return False

    if (address > 0xff):
      print("UserMemoryR: address must be between 0x0-0xFF.")
      return False

    dataCount = 3
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.USER_MEMORY_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff        # low byte
    s_buffer[MSG_INDEX_DATA+1]         = (address >> 8) & 0xff # high byte    
    s_buffer[MSG_INDEX_DATA+2]         = count
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('UserMemoryR: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in UserMemoryR BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return data

  def UserMemoryW(self, address, count, data):
    """
    This command writes the nonvolatile user memory.  The user memory
    is 256 bytes (address 0 - 0xFF).  The amount of data to be
    written is inferred from the frame count - 2;
    """

    if (count > 255):
      print("UserMemoryW: count must be less than 256.")
      return False

    if (address > 0xff):
      print("UserMemoryW: address must be between 0x0-0xFF.")
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.USER_MEMORY_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff       # low byte
    s_buffer[MSG_INDEX_DATA+1]         = (address >> 8)& 0xff # high byte
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('UserMemoryW: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in UserMemoryW BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    
  def SettingsMemoryR(self, address, count):
    """
    This command reads the nonvolatile settings memory.  The settings
    memory is 1024 bytes (address 0 - 0x3FF)

    """

    if (count > 255):
      print("SettingsMemoryR: count must be less than 256.")
      return False

    if (address > 0x3ff):
      print("SettingsMemoryR: address must be between 0x0-0x3FF.")
      return False

    dataCount = 3
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.SETTINGS_MEMORY_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff        # low byte
    s_buffer[MSG_INDEX_DATA+1]         = (address >> 8) & 0xff # high byte    
    s_buffer[MSG_INDEX_DATA+2]         = count
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('SettingsMemoryR: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
        data = r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in SettingsMemoryR BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return data

  def SettingsMemoryW(self, address, count, data):
    """
    This command writes the nonvolatile user memory.  The settings memory
    is 1024 bytes (address 0 - 0x3FF).  The amount of data to be
    written is inferred from the frame count - 2.  the
    settings will be implemented immediately.
    """

    if (count > 255):
      print("SettingsMemoryW: count must be less than 256.")
      return False

    if (address > 0x3ff):
      print("SettingsMemoryW: address must be between 0x0-0x3FF.")
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.SETTINGS_MEMORY_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff       # low byte
    s_buffer[MSG_INDEX_DATA+1]         = (address >> 8)& 0xff # high byte
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('SettingsMemoryW: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in SettingsMemoryW BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    
                         
  #############################################
  #        Miscellaneous Commands             #
  #############################################

  def BlinkLED(self, count=1):
    """
    This comman will blink the device power LED "count" times.
    """

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.BLINK_LED
    s_buffer[MSG_INDEX_DATA]           = count
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('Blink: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Blink BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def GetSerialNumber(self):
    """
    This commands reads the device serial number.  The serial number
    consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001").
    """
    
    dataCount = 0
    replyCount = 8
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.SERIAL
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('GetSerialNumber: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
      else:
        serial = unpack_from('8s',r_buffer,offset=MSG_INDEX_DATA)
        serial = serial[0].decode()
        return  serial
    except ResultError:
      print('Error in GetSerialNumber BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def Reset(self):
    """
    This command resets the device
    """
    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.RESET
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('Reset: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Reset BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      
  def Status(self):
    """
    This command reads the device status.

      status: bit 0:     Reserved
              bit 1:     1 = AIn scan running
              bit 2:     1 = AIn scan overrun
              bits 3-7:  Reserved
              bits 8-10: Charger status
                           0 = no battery
                           1 = fast charge
                           2 = maintenance charge
                           3 = fault (not charging)
	                   4 = disabled, operating on battery power
              bits 11-15:   Reserved
    """

    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.STATUS
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('Status: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
      else:
        status = unpack_from('H',r_buffer,offset=MSG_INDEX_DATA)
        self.status = status[0]
        return  (status[0])
    except ResultError:
      print('Error in Status BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def Ping(self):
    """
    This command checks communications with the device. Note that
    repetetive use of this command is not recommended for maximum
    battery life.
    """

    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.PING
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('Ping: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Ping BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return result

  def FirmwareVersion(self):
    """
    This command reads the firmware version.  The version consistes of
    16 bits in hexadecimal BCD notation, i.e. version 2.15 would be
    0x0215.
    """

    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.FIRMWARE_VERSION
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('FrimwareVersion: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
      else:
        version = unpack_from('H',r_buffer,offset=MSG_INDEX_DATA)
        return version[0]
    except ResultError:
      print('Error in FirmwareVersion BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def RadioFirmwareVersion(self):
    """
    This command reads the radio firmware version.  The version consistes of
    16 bits in hexadecimal BCD notation, i.e. version 6.15 would be
    0x0615.
    """
    
    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.RADIO_FIRMWARE_VERSION
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('RadioFrimwareVersion: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
      else:
        version = unpack_from('H',r_buffer,offset=MSG_INDEX_DATA)
        return version[0]
    except ResultError:
      print('Error in RadioFirmwareVersion BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def BatteryVoltage(self):
    """
    This command reads the battery voltage. The value is returned in millivolts.
    """

    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.BATTERY_VOLTAGE
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   
    self.device.frameID = (self.device.frameID + 1) % 256      # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage(len(r_buffer))
    except self.device.sock.socket.timeout:
      raise TimeoutError('BatteryVoltage: timeout error.')
      return

    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT] == replyCount & 0xff                         and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
        result = True

    try:
      if (result == False):
        raise ResultError
      else:
        voltage = unpack_from('H',r_buffer,offset=MSG_INDEX_DATA)
        self.voltage = voltage[0]
        return voltage[0]
    except ResultError:
      print('Error in BatteryVoltage BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
 
