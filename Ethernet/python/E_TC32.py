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

"""
    Configuration memory map
|=================================================================|
|    Address   |        Value                                     |
|=================================================================|
| 0x00 - 0x07  | Serial number                                    |
|-----------------------------------------------------------------|
| 0x08 - 0x09  | Reserved                                         |
|-----------------------------------------------------------------|
| 0x0A - 0x0F  | MAC address                                      |
|              |  If all 6 bytes are 0xFF then the firmware will  |
|              |  use the Microchip unique MAC address that       |
|              |  is programmed into the micro.                   |
|=================================================================|

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
| 0x016 - 0x01F | Reserved                                            |               |
|=====================================================================================|

Note: The settings do not take effect until after device is reset or power cycled.

    User memory map
|=================================================================|
|    Address      |        Value                                  |
|=================================================================|
| 0x0000 - 0x0EFF | Comms micro memory, available for UL use      |
|-----------------------------------------------------------------|
| 0x1000 - 0x1DFF | Measurement micro memory, available for UL use|
|-----------------------------------------------------------------|
| 0x2000 - 0x2DFF | EXP micro memory, available for UL use        |
|=================================================================|

"""

ETC32_PID =   0x0132

BASE                   = 0x1   # Base unit
EXP                    = 0x2   # expansion unit

CELSIUS                = 0x0   # read in Celsius
VOLTAGE                = 0x1   # read in Voltage
ADC_CODE               = 0x2   # uncalibraded

# Base class for lookup tables of calibration coefficients (slope and offset)
class table2:
  def __init__(self):
    self.slope_60_base = 0.0     # 60Hz slope values for ADC (base unit)
    self.slope_50_base = 0.0     # 50Hz slope values for ADC (base unit)
    self.intercept_60_base = 0.0 # 60Hz intercept values for ADC (base unit)
    self.intercept_50_base = 0.0 # 50Hz intercept values for ADC (base unit)
    self.slope_60_exp = 0.0      # 60Hz slope values for ADC (EXP unit)
    self.slope_50_exp = 0.0      # 50Hz slope values for ADC (EXP unit)
    self.intercept_60_exp = 0.0  # 60Hz intercept values for ADC (EXP unit)
    self.intercept_50_exp = 0.0  # 50Hz intercept values for ADC (EXP unit)

class E_TC32:
  # Digital I/O Commands
  CMD_DIN                = 0x00  # Read DIO pins
  CMD_DOUT_R             = 0x02  # Read DIO latch value
  CMD_DOUT_W             = 0x03  # Write DIO latch value

  # Temperature Input Commands
  CMD_TIN                = 0x10  # Read single thermocouple channel
  CMD_CJC                = 0x11  # Read single CJC sensor
  CMD_TIN_MULTIPLE       = 0x12  # Read multiple thermocouple channels
  CMD_CJC_MULTIPLE       = 0x13  # Read multiple CJC sensors
  CMD_TIN_CONFIG_R       = 0x14  # Read temperature channel configuration
  CMD_TIN_CONFIG_W       = 0x15  # Write temperature channel configuration
  CMD_TIN_STATUS         = 0x16  # Read temperature channel data status
  CMD_OTD_STATUS         = 0x17  # Read open thermocouple detect data status
  CMD_MEASURE_CONFIG_R   = 0x18  # Read measurement configuration
  CMD_MEASURE_CONFIG_W   = 0x19  # Write measurement configuration
  CMD_MEASURE_MODE_R     = 0x1a  # Read measurement mode
  CMD_MEASURE_MODE_W     = 0x1b  # Write measurement mode

  # Alarm Commands
  CMD_ALARM_CONFIG_R     = 0x20  # Read alarm configuration
  CMD_ALARM_CONFIG_W     = 0x21  # Write alarm configuration
  CMD_ALARM_STATUS_R     = 0x22  # Read temperature alarm status
  CMD_ALARM_STATUS_W     = 0x23  # Clear temperature alarm status

  # Memory Commands
  CMD_USER_MEMORY_R      = 0x30  # Read user memory
  CMD_USER_MEMORY_W      = 0x31  # Write user memory
  CMD_SETTINGS_MEMORY_R  = 0x32  # Read network settings memory
  CMD_SETTINGS_MEMORY_W  = 0x33  # Write network settings memory
  CMD_CONFIG_MEMORY_R    = 0x34  # Read device configuration memory
  CMD_CONFIG_MEMORY_W    = 0x35  # Write device configuration memory
  CMD_FACTORY_COEF_R     = 0x36  # Read factory calibration coefficients
  CMD_FACTORY_COEF_W     = 0x37  # Write factory calibration coefficients
  CMD_FIELD_COEF_R       = 0x38  # Read field calibration coefficients
  CMD_FIELD_COEF_W       = 0x39  # Write field calibration coefficients
  CMD_FACTORY_CAL_DATE_R = 0x3a  # Read factory calibration date
  CMD_FACTORY_CAL_DATE_W = 0x3b  # Write factory calibration date
  CMD_FIELD_CAL_DATE_R   = 0x3c  # Read field calibration date
  CMD_FIELD_CAL_DATE_W   = 0x3d  # Write field calibration date

  # Miscellaneous Commands
  CMD_BLINK_LED          = 0x50  # Blink the LED
  CMD_RESET              = 0x51  # Reset the device
  CMD_STATUS             = 0x52  # Read device status
  CMD_VERSION            = 0x53  # Read firmware versions
  CMD_NETWORK_CONFIG     = 0x54  # Read the current network configuration
  CMD_AD_CAL             = 0x55  # Run the A/D offset calibration

  def __init__(self, device):
    self.device = device                       # inherit values from mccEthernetDevice
    self.channel_mask = []                     # the channel bitmask for the base unit (channels 0-31)
                                               # the channel bitmask for the EXP  unit (channels 32-63)
    self.cjc_mask = []                         # the CJC bitmask for the base unit (channels 0-31)
                                               # the CJC bitmask for the EXP  unit (channels 32-63)
    self.Tin_status = []                       # the reading status of the Tin channels
    self.OTD_status = []                       # the status of the open thermocouple detect
    self.config_measure = []                   # the measurement configuration
    self.mode_measure = []                     # the measurement mode
    self.units = 0                             # the units for the returned values: 0 - Celsius, 1 - Voltage, 2 - ADC code (uncalibraded)
    self.wait = 0                              # 0 - return current value, 1 - wait for new value before returning.
    self.Tin_values = []                       # the values read from the configured channels
    self.CJC_values = []                       # the values read from the configured channels
    self.config_values = []                    # the configuration value of each channel (type of thermocouple);
    self.alarm_status = []                     # the alarm status of each channel
    self.alarm_config = bytearray(64)          # the alarm configuration
    self.alarm_threshold1 = [0.]*64            # the alarm threshold 1 values in Celsius
    self.alarm_threshold2 = [0.]*64            # the alarm threshold 2 values in Celsius
    self.calCoefFactory = [table2(), table2()] # the factory calibration coefficients (slope and offset).
    self.calCoefField   = [table2(), table2()] # the field calibration coefficients (slope and offset).
    self.status = 0                            # 1 - EXP detected, 0 - no EXP detected
    self.version = []                          # device firmware versions
    self.gain_voltages = []                    # gain calibration references values

    self.Status()                              # determine if EXP board present
    self.channel_mask.append(0x0)
    self.channel_mask.append(0x0)
    self.TinConfig_R()
    self.MeasureConfig_R()
    self.MeasureMode_R()
    self.AlarmConfig_R()
    self.AlarmStatus()
    self.MACaddress()                          # get the MAC address
    

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
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DIN
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
           value_base = r_buffer[MSG_INDEX_DATA]
           value_exp = r_buffer[MSG_INDEX_DATA+1]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DIn E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    if (self.status == 1):
      return value_base, value_exp
    else:
      return value_base
    
  def DOut_R(self):
    """
    This command reads the DIO output latch value.  The factory power
    on default is all zeros. A 0 in a bit position indicates the
    corresponding pin driver is low, a 1 indicates it is high.
    """
    
    dataCount = 0
    replyCount = 8
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
           value_base ,= unpack_from('I',r_buffer,MSG_INDEX_DATA)
           value_exp ,= unpack_from('I',r_buffer,MSG_INDEX_DATA+4)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DOut_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    if (self.status == 1):
      return value_base, value_exp
    else:
      return value_base

  def DOut(self, value, index=BASE):
    """
    This command writes the DIO latch value.  Writing a 0 to a bit will set
    the corresponding pin driver low, writing a 1 sets it high. If pin(s) are
    configured as alarm outputs, this command does not affect their value.
    
     index: bitmask the values to write
      bit 0: Base unit
      bit 1: EXP
    """
    
    dataCount = 5
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_DOUT_W
    s_buffer[MSG_INDEX_DATA]           = index
    pack_into('I', s_buffer, MSG_INDEX_DATA+1, value)
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
      print('Error in DOut E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  #################################
  #  Temperature Input Commands   #
  #################################

  def Tin(self, channel, units, wait):
    """
    This command reads the value of a single thermocouple channel.  There are some
    special return values:
    
    -777.0: Input voltage outside valid common-mode voltage range
    -888.0: Open thermocouple detected
    -999.0: Channel disabled (also returned if EXP channels are specified but 
            no EXP is connected) 
    
    channel: the channel to read (0-63)
    units:  0 - Celsius, linearized by TC type
            1 - Voltage
            2 - ADC code (uncalibrated)
    wait:   0 - return current value, 1 - wait for new reading before returning
    """

    dataCount = 3
    replyCount = 4
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN
    s_buffer[MSG_INDEX_DATA]           = channel
    s_buffer[MSG_INDEX_DATA+1]         = units
    s_buffer[MSG_INDEX_DATA+2]         = wait
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
      raise TimeoutError('Tin: timeout error.')
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
           value ,= unpack_from('f', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Tin E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def CJC(self, channel):
    """
    This command returns the most recent value of a single CJC sensor
    in Celsius.  The value -9999 will be returned if an EXP sensor is
    specified but no EXP is connected.
    
        channel: the channel to read (0-63)
    """

    dataCount = 1
    replyCount = 4
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CJC
    s_buffer[MSG_INDEX_DATA]           = channel
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
      raise TimeoutError('CJC: timeout error.')
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
           value ,= unpack_from('f', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in CJC E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def TinMultiple(self, wait, units, channel_mask_base, channel_mask_exp=0):
    """
    This command reads the value of multiple thermocouple channels 
    The channels to be read are passed as a bitmap when calling the command.
    The data will be returned in the order low channel number to high
    channel number.  The number of floating point values returned
    will be equal to the number of channels specified (max 64).  The
    special return values listed in the Tin command also apply to
    this command.
    
    wait:             0 - return current value
                       1 - wait for new reading before returning
    units:            0 - Celsius
                       1 - Voltage
                       2 - ADC code (uncalibraded)
    channel_mask_base: the channel bitmask for the base unit (channel 0-31)
    channel_mask_exp:  the channel bitmask for the EXP unit (channel 32-63)
    """

    dataCount = 10
    replyCount = 4*(self.nBits(channel_mask_base) + self.nBits(channel_mask_exp))
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN_MULTIPLE
    s_buffer[MSG_INDEX_DATA]           = wait
    s_buffer[MSG_INDEX_DATA+1]         = units
    pack_into('II',s_buffer, MSG_INDEX_DATA+2, channel_mask_base, channel_mask_exp)
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sock.settimeout(1.0)
    self.device.sendMessage(s_buffer)
    try:
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('TinMultiple: timeout error.')
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
           value = list(unpack_from('f'*(replyCount//4), r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in TinMultiple: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def CJCMultiple(self, channel_mask_base, channel_mask_exp=0):
    """
    This command reads the value of multiple CJC sensors.  The
    sensors to be read are passed as a bitmap when calling the
    command.  The data will be returned in the order low channel
    number to high channel.  The number of floating point values
    returned will be equal to the number of channels specified (max
    64).  The CJC values only update once per second so there is no
    need to call this faster.  The value -9999.0 will be returned
    if an EXO sensor is specified but no EXP is connected.
    
    cjc_mask_base: the channel bitmask for the base unit (channel 0-31)
    cjc_mask_exp:  the channel bitmask for the EXP unit (channel 32-63)
    """

    dataCount = 8
    replyCount = 4*(self.nBits(channel_mask_base) + self.nBits(channel_mask_exp))
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CJC_MULTIPLE
    pack_into('II',s_buffer, MSG_INDEX_DATA, channel_mask_base, channel_mask_exp)
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('CJCMultiple: timeout error.')
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
           value = list(unpack_from('f'*(replyCount//4), r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in CJCMultiple: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def TinConfig_R(self):
    """
    This command reads the thermocouple channel configurations.  Each
    configuration is a uint8_t with the following possible values:
    
    0 - channel disabled
    1 - TC type J
    2 - TC type K
    3 - TC type T
    4 - TC type E
    5 - TC type R
    6 - TC type S
    7 - TC type B
    8 - TC type N
    """

    dataCount = 0
    replyCount = 64
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN_CONFIG_R
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('TinConfig_R: timeout error.')
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
           self.config_values = list(unpack_from('B'*64, r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in TinConfig_R: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return self.config_values

  def TinConfig_W(self):
    """
    This command writes the temperature channel configurations.  The
    micro stores these values in EEPROM and load them from the EEPROM
    at power on.  Each configuration is a uint8 with the following
    possible values:
    
    0 - channel disabled
    1 - TC type J
    2 - TC type K
    3 - TC type T
    4 - TC type E
    5 - TC type R
    6 - TC type S
    7 - TC type B
    8 - TC type N
    """

    dataCount = 64
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN_CONFIG_W
    for i in range(64):
      s_buffer[MSG_INDEX_DATA+i] = self.config_values[i]
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('TinConfig_W: timeout error.')
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
      print('Error in TinConfig_W: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def TinStatus(self):
    """
    This command reads the status of the temperature readings.  If
    a bit is set the corresponding channel has a new reading that
    has not been read with either the Tin or TinMultiple command.
    """

    dataCount = 0
    replyCount = 8
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN_STATUS
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('TinStatus: timeout error.')
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
           self.Tin_status  = list(unpack_from('II',r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in TinStatus: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def OTDStatus(self):
    """
    This command reads the status of the open thermocouple
    detection.  If a bit is set an open thermocouple is currently
    detected on the corresponding channel.  The LED on the front
    of the device is on if any bits are set in this value.
    """

    dataCount = 0
    replyCount = 8
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_OTD_STATUS
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('OTDStatus: timeout error.')
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
           self.OTD_status = list(unpack_from('II',r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in OTDStatus: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MeasureConfig_R(self):
    """
    This command reads the measurement configuration for the base and exp units.
    
       bit 0: OTD disable               0 - OTD enable,    1 - OTD disabled
       bit 1: 50/60 Hz digital fileter  0 - notch @ 60 Hz, 1 - notch @ 50 Hz
       bit 2: Coefficient select        0 - factory coef.  1 - field coef.
       bits 3-7 - Reserved
    """

    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_MEASURE_CONFIG_R
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('MeasureConfig_R: timeout error.')
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
           self.config_measure = list(unpack_from('BB',r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in MeasureConfig_R: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MeasureConfig_W(self):
    """
    This command writes the measurement configuration for the base and exp units.
    The measurement microcontroller stores the configuration in EEPROM and
    restores it at power on.
    
       bit 0: OTD disable               0 - OTD enable,    1 - OTD disabled
       bit 1: 50/60 Hz digital fileter  0 - notch @ 60 Hz, 1 - notch @ 50 Hz
       bit 2: Coefficient select        0 - factory coef.  1 - field coef.
       bits 3-7 - Reserved
    """

    dataCount = 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_MEASURE_CONFIG_W
    s_buffer[MSG_INDEX_DATA]           = self.config_measure[0]
    s_buffer[MSG_INDEX_DATA+1]         = self.config_measure[1]
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
      r_buffer = self.device.sock.recv(512)
    except socket.timeout:
      raise TimeoutError('MeasureConfig_W: timeout error.')
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
      print('Error in MeasureConfig_W: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MeasureMode_R(self):
    """
    This command reads the measurement mode.  The power on default is mode 0 (normal mode)
    
       mode_base: the measurement mode for the base unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 0 and 16 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 0 and 16.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 0 and 16.
       mode_exp: the measurement mode for the EXP unit:
             0 = Normal mode, the measurement loop converts all of the configured channels in sequence
             1 = Test mode: the muxes are fixed on channel 32 and 48 and the ADCs continuously convert those channels.
             2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                 on that input. Value is stored in channel 32 and 48.
             3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                 that input. Value is stored in channel 32 and 48.
    """
    
    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_MEASURE_MODE_R
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
      raise TimeoutError('MeasureMode_R: timeout error.')
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
           self.mode_measure = list(unpack_from('BB',r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in MeasureMode_R: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MeasureMode_W(self):
    """
    This command writes the measurement mode.  The power on default is mode 0 (normal mode)
    
     mode_base: the measurement mode for the base unit:
            0 = Normal mode, the measurement loop converts all of the configured channels in sequence
            1 = Test mode: the muxes are fixed on channel 0 and 16 and the ADCs continuously convert those channels.
            2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                on that input. Value is stored in channel 0 and 16.
            3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                that input. Value is stored in channel 0 and 16.
      mode_exp: the measurement mode for the EXP unit:
            0 = Normal mode, the measurement loop converts all of the configured channels in sequence
            1 = Test mode: the muxes are fixed on channel 32 and 48 and the ADCs continuously convert those channels.
            2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                on that input. Value is stored in channel 32 and 48.
            3 = Gain measure mode: gain cal circuit is connected to cal mux and all conversions are performed on 
                that input. Value is stored in channel 32 and 48.
    """

    dataCount = 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_MEASURE_MODE_W
    s_buffer[MSG_INDEX_DATA]           = self.mode_measure[0]
    s_buffer[MSG_INDEX_DATA+1]         = self.mode_measure[1]
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
      raise TimeoutError('MeasureMode_W: timeout error.')
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
      print('Error in MeasureMode_W: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  #################################
  #      Alarm  Commands          #
  #################################

  def AlarmConfig_R(self):
    """
    This command reads the temperature alarm configurations. There
    are configuration values and two threshold values for each of the
    32/64 thermocouple channels.
    
    alarm_config: the alarm configuration
      bit 0: Alarm enable
             0 - alarm disabled, associated bit is controlled by DOut
             1 - alarm enabled,  associated bit is controlled by status
      bit 1: Alarm invert
             0 - normal polarity   (output is low when in alarm condition)
             1 - inverted polarity (output is high when in alarm condition)
      bits 2-3: Alarm type
             0 - High level: alarm when reading >= threshold 1, reset when reading < threshold 2
             1 - Low level: alarm when reading <= threshold 1, reset when reading > threshold 2
             2 - Outside window: alarm when reading < threshold 1 or > threshold 2
      bit 4: Alarm latch
             0 - no latch, alarm output status indicates current state of alarm
             1 - latch, alarm output is active if an alarm condition is detected 
                 and remains active until cleared with AlarmStatus command
      bits 5-6: Error alarms
            00 - Alarm can only be set by valid temperature reading
            01 - An open thermocouple or common-mode voltage error will also set the alarm
            10 - Only an open thermocouple or common-mode voltage will set the alarm,
                 termperature is ignored.
            11 - invalid.
      bit 7: reserved.
    """

    dataCount = 0
    replyCount = 576
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_ALARM_CONFIG_R
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
      raise TimeoutError('AlarmConfig_R: timeout error.')
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
           self.alarm_config         = list(unpack_from('B'*32, r_buffer, MSG_INDEX_DATA))
           self.alarm_threshold1     = list(unpack_from('f'*32, r_buffer, MSG_INDEX_DATA+32))
           self.alarm_threshold2     = list(unpack_from('f'*32, r_buffer, MSG_INDEX_DATA+160))
           self.alarm_config[32:64]  = list(unpack_from('B'*32, r_buffer, MSG_INDEX_DATA+288))
           self.alarm_threshold1[32:64] = list(unpack_from('f'*32, r_buffer, MSG_INDEX_DATA+320))
           self.alarm_threshold2[32:64] = list(unpack_from('f'*32, r_buffer, MSG_INDEX_DATA+448))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AlarmConfig_R: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AlarmConfig_W(self):
    """
    This command writes the temperature alarm configurations. There
    are configuration values and two threshold values for each of the
    32/64 thermocouple channels.  The configuration is stored in
    EEPROM and restored at power on.
    
    alarm_config: the alarm configuration
      bit 0: Alarm enable
             0 - alarm disabled, associated bit is controlled by DOut
             1 - alarm enabled,  associated bit is controlled by status
      bit 1: Alarm invert
             0 - normal polarity   (output is low when in alarm condition)
             1 - inverted polarity (output is high when in alarm condition)
      bits 2-3: Alarm type
             0 - High level: alarm when reading >= threshold 1, reset when reading < threshold 2
             1 - Low level: alarm when reading <= threshold 1, reset when reading > threshold 2
             2 - Outside window: alarm when reading < threshold 1 or > threshold 2
      bit 4: Alarm latch
             0 - no latch, alarm output status indicates current state of alarm
             1 - latch, alarm output is active if an alarm condition is detected 
                 and remains active until cleared with AlarmStatus command
      bits 5-6: Error alarms
            00 - Alarm can only be set by valid temperature reading
            01 - An open thermocouple or common-mode voltage error will also set the alarm
            10 - Only an open thermocouple or common-mode voltage will set the alarm,
                 termperature is ignored.
            11 - invalid.
      bit 7: reserved.
    """

    dataCount = 0
    replyCount = 576
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_ALARM_CONFIG_W
    pack_into('B'*32, s_buffer, MSG_INDEX_DATA,     self.alarm_config[0:32])
    pack_into('f'*32, s_buffer, MSG_INDEX_DATA+32,  self.alarm_threshold1[0:32])
    pack_into('f'*32, s_buffer, MSG_INDEX_DATA+160, self.alarm_threshold2[0:32])
    pack_into('B'*32, s_buffer, MSG_INDEX_DATA+288, self.alarm_config[32:64])
    pack_into('f'*32, s_buffer, MSG_INDEX_DATA+320, self.alarm_threshold1[32:64])
    pack_into('f'*32, s_buffer, MSG_INDEX_DATA+448, self.alarm_threshold2[32:64])
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
      raise TimeoutError('AlarmConfig_W: timeout error.')
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
      print('Error in AlarmConfig_W: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AlarmStatus(self):
    """
    This command reads the status of the temperature
    alarms. If a bit is set an alarm condition exists or is latched
    on the corresponding channel. If the alarm is configured for
    latching then the status will not clear when the alarm condition
    is no longer present. It must be cleared by writing a 1 to the
    corresponding bit. The LED on the front of the device is on if
    any bits are set in this value.
    """

    dataCount = 0
    replyCount = 8
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_ALARM_STATUS_R
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
      raise TimeoutError('AlarmStatus: timeout error.')
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
           self.alarm_status = list(unpack_from('II', r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in AlarmStatus: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return self.alarm_status

  def ClearAlarmStatus(self, clear_masks, index=BASE):
    """
    This command clears the alarm status.  Writing a 1 to a bit will
    clear the status for the corresponding channel.
    
      index:   bit 0: Base Unit
               bit 1: EXP
      clear_masks; the alarm status clear masks
    """

    dataCount = 5
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_ALARM_STATUS_W
    s_buffer[MSG_INDEX_DATA]           = index
    pack_into('I', s_buffer, MSG_INDEX_DATA+1, clear_masks)
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
      raise TimeoutError('ClearAlarmStatus: timeout error.')
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
      print('Error in ClearAlarmStatus: E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return self.alarm_status

  #################################
  #       Memory  Commands        #
  #################################
  
  def UserMemory_R(self, address, count):
    """
    This command reads the nonvolatile user memory. The
    user memory is spread among 3 EEPROM parts
    
        Address                   Value
     --------------------------------------------
     0x0000 - 0x0EFF     Comms micro memory
     0x1000 - 0x1DFF     Measurement micro memory
     0x2000 - 0x2DFF     EXP micro memory
    
    
     address: the start address for reading (0-0xdff)
     count:   the number of bytes to read (max 1024 due to protocol)
    """

    if (count > 1024 or address > 0xdff):
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
      r_buffer = self.device.sock.recv(1050)
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
      print('Error in UserMemory_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def UserMemory_W(self, address, count, data):
    """
    This command writes the nonvolatile user memory. The amount of data 
    to be written is inferred from the frame count - 2.  The maximum that
    can be written in one transfer is 1024 - 2 bytes.
    """

    if (count > 512 or address > 0xdff):
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_USER_MEM_W
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
      print('Error in UserMemory_W E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def SettingsMemory_R(self, address, count):
    """
    This command reads the nonvolatile settings memory.  The settings memory is
    32 bytes (address 0 - 0x1f)
    
    address: the start address for reading (0-0x1f)
    count:   the number of bytes to read (max 32 due to protocol)
    """
    
    if (count > 32 or address > 0x1f):
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
      print('Error in SettingsMemory_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    return value

  def SettingsMemory_W(self, address, count, data):
    """
    This command writes to the nonvolatile settings memory.  The settings memory
    is 32 bytes (address 0 - 0x1f).  The amount of data to be
    written is inferred from the frame count  - 2.  The maximum that
    can be written in one transfer is 32 bytes.  The settings will
    be implemented after a device reset.
    """

    if (count > 256 or address > 0x1f):
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
      print('Error in SettingsMemory_W E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def ConfigMemory_R(self, address, count):
    """
    This command reads the nonvolatile configuration memory.  The configuration memory is
    16 bytes (address 0 - 0x0f)
    """

    if (count > 16 or address > 0xf):
      return False

    dataCount = 4
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CONFIG_MEMORY_R
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
      raise TimeoutError('ConfigMemory_R: timeout error.')
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
      print('Error in ConfigMemory_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def ConfigMemory_W(self, address, count, data):
    """
    This command writes the nonvolatile configuration memory.  The
    config memory is 16 bytes (address 0 - 0xf) The config memory
    should only be written during factory setup.
    
    address: the start address for writing (0-0xf)
    data:    the data to be written (frame count -2)
    """

    if (count > 16 or address > 0xf):
      return False

    dataCount = count + 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CONFIG_MEMORY_W
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
      raise TimeoutError('ConfigMemory_W: timeout error.')
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
      print('Error in ConfigMemory_W E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FactoryCoef_R(self):
    """
    This command reads the factory calibration coefficients.  Each coefficient is
    a float.  The firmware applies the coefficients when calculating the voltage and
    temperature values for each channel.  The coefficients are applied immediately
    and stored in EEPROM.
    """

    dataCount = 0
    replyCount = 64
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FACTORY_COEF_R
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
      r_buffer = self.device.sock.recv(128)
    except socket.timeout:
      raise TimeoutError('FactoryCoef_R: timeout error.')
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
           self.calCoefFactory[0].slope_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA)
           self.calCoefFactory[1].slope_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+4)
           self.calCoefFactory[0].slope_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+8)
           self.calCoefFactory[1].slope_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+12)
           self.calCoefFactory[0].intercept_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+16)
           self.calCoefFactory[1].intercept_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+20)
           self.calCoefFactory[0].intercept_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+24)
           self.calCoefFactory[1].intercept_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+28)
           self.calCoefFactory[0].slope_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+32)
           self.calCoefFactory[1].slope_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+36)
           self.calCoefFactory[0].slope_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+40)
           self.calCoefFactory[1].slope_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+44)
           self.calCoefFactory[0].intercept_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+48)
           self.calCoefFactory[1].intercept_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+52)
           self.calCoefFactory[0].intercept_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+56)
           self.calCoefFactory[1].intercept_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+60)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FactoryCoef_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FactoryCoef_W(self, index):
    """
    This command writes the factory calibration coefficients.  The
    microcontroller stores the values in EEPROM and restores them at
    power on.
    
    index: the device to write, 0 - base,   1 - EXP
    """

    dataCount = 33
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FACTORY_COEF_W
    s_buffer[MSG_INDEX_DATA]           = index
    if (index == 0):
      pack_into('f', s_buffer, MSG_INDEX_DATA+1, self.calCoefFactory[0].slope_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+5, self.calCoefFactory[1].slope_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+9, self.calCoefFactory[0].slope_50_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+13, self.calCoefFactory[1].slope_50_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+17, self.calCoefFactory[0].intercept_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+21, self.calCoefFactory[1].intercept_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+25, self.calCoefFactory[0].intercept_50_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+29, self.calCoefFactory[1].intercept_50_base)
    else:
      pack_into('f', s_buffer, MSG_INDEX_DATA+1, self.calCoefFactory[0].slope_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+5, self.calCoefFactory[1].slope_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+9, self.calCoefFactory[0].slope_50_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+13, self.calCoefFactory[1].slope_50_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+17, self.calCoefFactory[0].intercept_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+21, self.calCoefFactory[1].intercept_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+25, self.calCoefFactory[0].intercept_50_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+29, self.calCoefFactory[1].intercept_50_exp)
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
      raise TimeoutError('FactoryCoef_W: timeout error.')
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
      print('Error in FactoryCoef_W E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FieldCoef_R(self):
    """
    This command reads the field calibration coefficients.  Each coefficient is
    a float.  The firmware applies the coefficients when calculating the voltage and
    temperature values for each channel.  The coefficients are applied immediately
    and stored in EEPROM.
    """

    dataCount = 0
    replyCount = 64
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FIELD_COEF_R
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
      r_buffer = self.device.sock.recv(128)
    except socket.timeout:
      raise TimeoutError('FieldCoef_R: timeout error.')
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
           self.calCoefField[0].slope_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA)
           self.calCoefField[1].slope_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+4)
           self.calCoefField[0].slope_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+8)
           self.calCoefField[1].slope_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+12)
           self.calCoefField[0].intercept_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+16)
           self.calCoefField[1].intercept_60_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+20)
           self.calCoefField[0].intercept_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+24)
           self.calCoefField[1].intercept_50_base ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+28)
           self.calCoefField[0].slope_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+32)
           self.calCoefField[1].slope_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+36)
           self.calCoefField[0].slope_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+40)
           self.calCoefField[1].slope_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+44)
           self.calCoefField[0].intercept_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+48)
           self.calCoefField[1].intercept_60_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+52)
           self.calCoefField[0].intercept_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+56)
           self.calCoefField[1].intercept_50_exp ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+60)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FieldCoef_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FieldCoef_W(self, index):
    """
    This command writes the field calibration coefficients.  The
    microcontroller stores the values in EEPROM and restores them at
    power on.
    
    index: the device to write, 0 - base,   1 - EXP
    """

    dataCount = 33
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FIELD_COEF_W
    s_buffer[MSG_INDEX_DATA]           = index
    if (index == 0):
      pack_into('f', s_buffer, MSG_INDEX_DATA+1, self.calCoefField[0].slope_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+5, self.calCoefField[1].slope_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+9, self.calCoefField[0].slope_50_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+13, self.calCoefField[1].slope_50_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+17, self.calCoefField[0].intercept_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+21, self.calCoefField[1].intercept_60_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+25, self.calCoefField[0].intercept_50_base)
      pack_into('f', s_buffer, MSG_INDEX_DATA+29, self.calCoefField[1].intercept_50_base)
    else:
      pack_into('f', s_buffer, MSG_INDEX_DATA+1, self.calCoefField[0].slope_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+5, self.calCoefField[1].slope_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+9, self.calCoefField[0].slope_50_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+13, self.calCoefField[1].slope_50_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+17, self.calCoefField[0].intercept_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+21, self.calCoefField[1].intercept_60_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+25, self.calCoefField[0].intercept_50_exp)
      pack_into('f', s_buffer, MSG_INDEX_DATA+29, self.calCoefField[1].intercept_50_exp)
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
      raise TimeoutError('FieldCoef_W: timeout error.')
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
      print('Error in FieldCoef_W E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FactoryCalDate_R(self):
    """
    This command reads the factory calibration dates.
    """
      
    dataCount = 0
    replyCount = 12
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FACTORY_CAL_DATE_R
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
      raise TimeoutError('FactoryCalDate_R: timeout error.')
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
           year ,= unpack_from('B', r_buffer, MSG_INDEX_DATA)
           year += 2000
           month ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+1)
           day ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+2)
           hour ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+3)
           minute ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+4)
           second ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+5)
           date_base = datetime(year, month, day, hour, minute, second)
           if (self.status == 1): # EXP detected
             year ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+6)
             year += 2000
             month ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+7)
             day ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+8)
             hour ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+9)
             minute ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+10)
             second ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+11)
             date_exp = datetime(year, month, day, hour, minute, second)           
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FactoryCalDate_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    if (self.status == 1): # EXP detected
      return (date_base, date_exp)
    else:
      return date_base

  def FactoryCalDate_W(self, mdate, index=0):
    """
    This command writes the factory calibration date
    
     index:  the device to write  0 - base unit,  1 - EXP
    """
      
    dataCount = 7
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CAL_DATE_W
    s_buffer[MSG_INDEX_DATA]           = index
    s_buffer[MSG_INDEX_DATA+1]         = mdate.year - 2000
    s_buffer[MSG_INDEX_DATA+2]         = mdate.month
    s_buffer[MSG_INDEX_DATA+3]         = mdate.day
    s_buffer[MSG_INDEX_DATA+5]         = mdate.hour
    s_buffer[MSG_INDEX_DATA+6]         = mdate.minute
    s_buffer[MSG_INDEX_DATA+7]         = mdate.second
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
      raise TimeoutError('FactoryCalDate_W: timeout error.')
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
      print('Error in FactoryCalDate_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FieldCalDate_R(self):
    """
    This command reads the field calibration dates.
    """
      
    dataCount = 0
    replyCount = 12
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FIELD_CAL_DATE_R
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
      raise TimeoutError('FieldCalDate_R: timeout error.')
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
           year ,= unpack_from('B', r_buffer, MSG_INDEX_DATA)
           year += 2000
           month ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+1)
           day ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+2)
           hour ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+3)
           minute ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+4)
           second ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+5)
           date_base = datetime(year, month, day, hour, minute, second)
           if (self.status == 1): # EXP detected
             year ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+6)
             year += 2000
             month ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+7)
             day ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+8)
             hour ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+9)
             minute ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+10)
             second ,= unpack_from('B', r_buffer, MSG_INDEX_DATA+11)
             date_exp = datetime(year, month, day, hour, minute, second)           
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FieldCalDate_R E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      date_base = datetime.today()     # send back current date
      date_exp =  datetime.today()     # send back current date
    if (self.status == 1): # EXP detected
      return (date_base, date_exp)
    else:
      return date_base

  def FieldCalDate_W(self, mdate, index=0):
    """
    This command writes the field calibration date
    
    index:  the device to write  0 - base unit,  1 - EXP
    """
      
    dataCount = 7
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CAL_DATE_W
    s_buffer[MSG_INDEX_DATA]           = index
    s_buffer[MSG_INDEX_DATA+1]         = mdate.year - 2000
    s_buffer[MSG_INDEX_DATA+2]         = mdate.month
    s_buffer[MSG_INDEX_DATA+3]         = mdate.day
    s_buffer[MSG_INDEX_DATA+5]         = mdate.hour
    s_buffer[MSG_INDEX_DATA+6]         = mdate.minute
    s_buffer[MSG_INDEX_DATA+7]         = mdate.second
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
      raise TimeoutError('FieldCalDate_W: timeout error.')
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
      print('Error in FieldCalDate_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

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
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_BLINK_LED
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
      print('Error in blink E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

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
      raise TimeoutError('Reset: timeout error.')
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
      print('Error in reset E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def Status(self):
    """
    This command reads the device status
       bit 0:  1 = EXP detected,  0 = no EXP
       bits 1-15: Reserved
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
      raise TimeoutError('Status: timeout error.')
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
           self.status = r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8 & 0xff)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in status E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return self.status

  def Version(self):
    """
    This command reads the device firmware versions.  Each version
    will be in hex BCD (i.e. 0x0103 is version 1.03)
    
     version_comms           The communications micro firmware version
     boot_version_comms      The communcations micro bootloader firmware version
     version_base            The base measurement micro firmware version
     boot_version_base       The base measurement micro bootloader firmware version
     version_EXP             The EXP measurement micro firmware version
     boot_version_EXP        The EXP measurement micro bootloader firmware version
    """

    dataCount = 0
    replyCount = 12
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_VERSION
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
      r_buffer = self.device.sock.recv(32)
    except socket.timeout:
      raise TimeoutError('Version: timeout error.')
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
           self.version = unpack_from('H'*6, r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in version E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return self.version

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
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_NETWORK_CONFIG
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
      raise TimeoutError('NetworkConfig: timeout error.')
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
      print('Error in networkConfig E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def ADCal(self):
    """
    This command causes the measurement loop to pause and an A/D
    system offset calibration to run.  The calibration requires
    approximately 50 ms to complete then the measurement loop
    automatically returns to the current mode.  The calibration will
    run on all A/Ds on both the main unit and EXP simultaneousluy.
    The command reply will not be sent until the calibration completes.
    """

    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_AD_CAL
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sock.settimeout(1.0)
    self.device.sendMessage(s_buffer)
    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('ADCal: timeout error.')
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
        print('Error in ADCal E-TC32.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MACaddress(self):
    """
    Gets the MAC address
    """
    
    address = 0x0a
    value =  self.ConfigMemory_R(address, 6)
    self.device.MAC = (value[0]<<40) + (value[1]<<32) + (value[2]<<24) + (value[3]<<16) + (value[4]<<8) + value[5]
    return self.device.MAC

  @staticmethod
  def nBits(num):
    # counts the numbers of bits in a 32 bit number
    count = 0
    for i in range(32):
      if (num & 0x1) == 0x1:
        count += 1
      num = (num >> 0x1)
    return count
