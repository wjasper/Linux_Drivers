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
| 0x00 - 0x07  | Serial number (not used by firmware)             |
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
|    Address     |        Value                                   |
|=================================================================|
| 0x000 - 0xDFF  | Available for UL use                           |
|=================================================================|

"""

ETC_PID = 0x0138   # Product code for the MCC E-TC

CELSIUS      = 0x0 # read in Celsius
VOLTAGE      = 0x1 # read in Voltage
ADC_CODE     = 0x2 # uncalibraded

class E_TC:
  # Digital I/O Commands
  CMD_DIN                 = 0x00  # Read DIO pins
  CMD_DOUT_R              = 0x02  # Read DIO latch value
  CMD_DOUT_W              = 0x03  # Write DIO latch value
  CMD_DCONF_R             = 0x04  # Read DIO configuration value
  CMD_DCONF_W             = 0x05  # Write DIO configuration value

  # Temperature Input Commands
  CMD_TIN                 = 0x10  # Read single thermocouple channel
  CMD_CJC                 = 0x11  # Read single CJC sensor
  CMD_TIN_CONFIG_R        = 0x12  # Read temperature channel configuration
  CMD_TIN_CONFIG_W        = 0x13  # Write temperature channel configuration
  CMD_TIN_STATUS          = 0x14  # Read temperature channel data status
  CMD_OTD_STATUS          = 0x15  # Read open thermocouple detect data status
  CMD_MEASURE_CONFIG_R    = 0x16  # Read measurement configuration
  CMD_MEASURE_CONFIG_W    = 0x17  # Write measurement configuration
  CMD_MEASURE_MODE_R      = 0x18  # Read measurement mode
  CMD_MEASURE_MODE_W      = 0x19  # Write measurement mode
  CMD_FACTORY_COEF_R      = 0x1A  # Write factory calibration coefficients
  CMD_FACTORY_COEF_W      = 0x1B  # Write factory calibration coefficients
  CMD_FIELD_COEF_R        = 0x1C  # Read field calibration coefficients
  CMD_FIELD_COEF_W        = 0x1D  # Write field calibration coefficients
  CMD_FACTORY_CAL_DATE_R  = 0x1E  # Read factory calibration date
  CMD_FACTORY_CAL_DATE_W  = 0x1F  # Write factory calibration date
  CMD_FIELD_CAL_DATE_R    = 0x20  # Read field calibration date
  CMD_FIELD_CAL_DATE_W    = 0x21  # Write field calibration date
  CMD_AD_CAL              = 0x22  # Run the A/D offset calibration
  CMD_CJC_OFFSET_R        = 0x24  # Read user CJC offset values
  CMD_CJC_OFFSET_W        = 0x25  # Write user CJC offset values
  
  # Alarm Commands
  CMD_ALARM_CONFIG_R      = 0x28  # Read temperature alarm configuration
  CMD_ALARM_CONFIG_W      = 0x29  # Write temperature alarm configuration
  CMD_ALARM_STATUS_R      = 0x2A  # Read temperature alarm status
  CMD_ALARM_STATUS_W      = 0x2B  # Clear temperature alarm status

  # Counter Commands
  CMD_COUNTER_R           = 0x30  # Read event counter
  CMD_COUNTER_W           = 0x31  # Write event counter

  # Memory Commands
  CMD_CONF_MEMORY_R       = 0x40 # Read device configuration memory
  CMD_CONF_MEMORY_W       = 0x41 # Write device configuration memory
  CMD_USER_MEMORY_R       = 0x42 # Read user memory
  CMD_USER_MEMORY_W       = 0x43 # Write user memory
  CDM_SETTINGS_MEMORY_R   = 0x44 # Read network settings memory
  CMD_SETTINGS_MEMORY_W   = 0x45 # Write network settings memory
  CMD_BOOT_MEMORY_R       = 0x46 # Read bootloader memory
  CDM_BOOT_MEMORY_W       = 0x47 # Write bootloader memory

  # Miscellaneous Commands
  CMD_BLINK_LED           = 0x50 # Blink the LED
  CMD_RESET               = 0x51 # Reset the device
  CMD_STATUS              = 0x52 # Read device status
  CMD_NETWORK_CONFIG      = 0x54 # Read the current network configuration
  CMD_FIRMWARE_UPGRADE    = 0x60 # Enter firmware upgrade mode

  def __init__(self, device):
    self.device = device                     # inherit values from mccEthernetDevice
    self.channel_mask = 0                    # the channel bitmask (channels 0-7)
    self.Tin_status = 0                      # the reading status of the Tin channels
    self.OTD_status = 0                      # the status of the open thermocouple detect
    self.config_measure = 0                  # the measurement configuration
    self.mode_measure = 0                    # the measurement mode
    self.units = 0                           # the units for the returned values: 0 - Celsius, 1 - Voltage, 2 - ADC code (uncalibraded)
    self.wait = 0                            # 0 - return current value, 1 - wait for new value before returning.
    self.Tin_values = []                     # the values read from the configured channels
    self.CJC_offsets = []                    # the per channel CJC user offsets
    self.CJC_value= []                       # the CJC values in Celsius.
    self.config_values = bytearray(8)        # the configuration value of each channel (type of thermocouple);
    self.alarm_status = 0                    # the alarm status of each channel
    self.alarm_config = bytearray(8)         # the alarm configuration
    self.alarm_threshold1 = []               # the alarm threshold 1 values in Celsius
    self.alarm_threshold2 = []               # the alarm threshold 2 values in Celsius
    self.calCoefFactory = [table(), table()] # the factory calibration coefficients (slope and offset).
    self.calCoefField = [table(), table()]   # the field calibration coefficients (slope and offset).
    self.MACaddress()                        # get the MAC address
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
           value = r_buffer[MSG_INDEX_DATA]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in DIn E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
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
      print('Error in DOut_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    return value

  def DOut(self, value):
    """
    This command writes the DIO latch value.  Writing a 0 to a bit will set
    the corresponding pin driver low, writing a 1 sets it high. If pin(s) are
    configured as alarm outputs, this command does not affect their value.
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
      print('Error in DOut E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

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
      print('Error in DConfig_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    return value

  def DConfig_W(self, value):
    """
    This command writes the configuration value.  A 1 in a bit
    position sets the corresponding pin to an input, a 0 sets it to an 
    output.  The power on default is all ones (input).  If one or more
    alarms are configured, they will force the corresponding DIO bit to
    and output and may not be overridden with this command.
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
      print('Error in DConfig_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  #############################################
  #      Temperature Input  Commands          #
  #############################################

  def Tin(self, channel_mask, units, wait):
    """
    This command reads the value of one or more thermocouple
    channels. The channels to be read are passed as a bitmask when
    calling the command. The data will be returned in the order low
    channel number to high channel. The number of floating point
    values returned will be equal to the number of channels specified
    (max 8). Each temperature is calculated by performing cold
    junction compensation as follows:
    
    T = reverse_nist_function(Vin + Vcjc), where Vin is input
    voltage, Vcjc is calculated cold junction voltage based on the
    terminal temperature, and reverse_nist_function() is the NIST
    thermocouple polynomial for voltage to temperature.
    
    Vcjc = nist_function(Tcjc), where Tcjc is the temperature of the
    screw terminal and nist_function() is the NIST thermocouple
    polynomial for temperature to voltage.
    
    Tcjc = Tsensor + Toffset + Tuser, where Tsensor is the
    appropriate CJC sensor temperature, Toffset is a per-channel
    offset characterized over a number of boards and fixed in
    firmware, and Tuser is an optional user-specified offset (see
    CJCOffset_r and CJCOffset_w).  
    
    There are some special return values: 
       -6666.0: Over range on input (temperature < -300C, only reported with units = 0) 
       -8888.0: Open thermocouple detected (only reported with units = 0) 
       -9999.0: Channel disabled
    
    channel_mask:  bitmask, the channels to be read
    units:         the units for the returned values
                    0 - Celsius, linerarized by TC type
                    1 - Voltage
                    2 - ADC code (uncalibrated)
    wait:           0 - return current value, 1 - wait for new readings before returning
    """

    dataCount = 3
    replyCount = 4*self.nBits8(channel_mask)
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN
    s_buffer[MSG_INDEX_DATA]           = channel_mask & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = units & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = wait & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sock.settimeout(.5)
    self.device.sendMessage(s_buffer)
    try:
      r_buffer = self.device.sock.recv(128)
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
           value = list(unpack_from('f'*self.nBits8(channel_mask), r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in Tin E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def CJC(self):
    """
    This command reads the most recent value of the CJC sensors in Celsius.
    """

    dataCount = 0
    replyCount = 8
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CJC
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
           self.CJC_value = list(unpack_from('ff', r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in CJC E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return False
    
    return self.CJC_value

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
    replyCount = 8
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
      r_buffer = self.device.sock.recv(64)
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
           for i in range(8):
             self.config_values[i]= r_buffer[MSG_INDEX_DATA+i]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in TinConfig_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def TinConfig_W(self):
    """
    This command writes the thermocouple channel
    configurations. The micro stores these values in EEPROM and
    loads them from EEPROM at power on.  Each configuration is a
    uint8_t with the following possible values:
    
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

    dataCount = 8
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_TIN_CONFIG_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    for i in range(8):
      s_buffer[MSG_INDEX_DATA+i] = self.config_values[i]
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
      print('Error in TinConfig_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def TinStatus(self):
    """
    This command reads the status of the temperature readings.  If
    a bit is set the corresponding channel has a new reading that
    has not been read with the Tin command.
    """
    
    dataCount = 0
    replyCount = 1
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
      r_buffer = self.device.sock.recv(64)
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
           self.Tin_status ,= unpack_from('B', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in TinStatus E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1

    return self.Tin_status

  def OTDStatus(self):
    """
    This command reads the status of the open thermocouple
    detection.  If a bit is set an open thermocouple is currently
    detected on the corresponding channel.  
    """
    
    dataCount = 0
    replyCount = 1
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
      r_buffer = self.device.sock.recv(64)
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
           self.OTD_status ,= unpack_from('B', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in OTDStatus E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    return self.OTD_status

  def MeasureConfig_R(self):
    """
    This command reads the measurement configuration. 
    
     bit 0: OTD disable         0 - OTD enable,          1 - OTD disabled
     bit 1: Coefficient select  0 - factory coefficients 1 - field coefficients
     bit 2-7:  Reserved 
    """

    dataCount = 0
    replyCount = 1
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
      r_buffer = self.device.sock.recv(64)
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
           self.config_measure ,= unpack_from('B', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in MeasureConfig_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    return self.config_measure

  def MeasureConfig_W(self):
    """
    This command writes the measurement configuration. 
    
     bit 0: OTD disable         0 - OTD enable,          1 - OTD disabled
     bit 1: Coefficient select  0 - factory coefficients 1 - field coefficients
     bit 2-7:  Reserved 
    """

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_MEASURE_CONFIG_W
    s_buffer[MSG_INDEX_DATA]           = self.config_measure 
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
      print('Error in MeasureConfig_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MeasureMode_R(self):
    """
    This command reads the measurement mode.  The power on default is mode 0 (normal mode) 
    mode_base: the measurement mode for the base unit:
            0 = Normal mode, the measurement loop converts all of the configured channels in sequence
            1 = Test mode: the muxes are fixed on channel 0 and 4 and the ADCs continuously convert those channels.
            2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                on that input. Value is stored in channel 0 and 4.
    """

    dataCount = 0
    replyCount = 1
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
           self.mode_measure ,= unpack_from('B', r_buffer, MSG_INDEX_DATA)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in MeasureMode_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return -1
    return self.mode_measure

  def MeasureMode_W(self):
    """
    This command writes the measurement mode.  The power on default is mode 0 (normal mode) 
    mode_base: the measurement mode for the base unit:
            0 = Normal mode, the measurement loop converts all of the configured channels in sequence
            1 = Test mode: the muxes are fixed on channel 0 and 4 and the ADCs continuously convert those channels.
            2 = Offset measure mode: offset cal circuit is connected to cal mux and all conversions are performed 
                on that input. Value is stored in channel 0 and 4.
    """

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_MEASURE_MODE_W
    s_buffer[MSG_INDEX_DATA]           = self.mode_measure
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
      print('Error in MeasureMode_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  def FactoryCoefficients_R(self):
    """
    This command reads the factory calibration coefficients.  Each
    coefficient is a float.  The firmware applies the coefficients
    when calculating the voltage and temperature values for each
    channel.  The coefficients are applied immediately and stored in
    the EEPROM.
    """

    dataCount = 0
    replyCount = 1
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
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('FactoryCoefficients_R: timeout error.')
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
           self.calCoefFactory[0].slope ,= unpack_from('f', r_buffer, MSG_INDEX_DATA)
           self.calCoefFactory[1].slope ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+4)
           self.calCoefFactory[0].intercept ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+8)
           self.calCoefFactory[1].intercept ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+12)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FactoryCoefficients_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  def FactoryCoefficients_W(self):
    """
    This command writes the factory calibration coefficients.  The
    microcontroller stores the values in EEPROM and restores them at
    power on. 
    """

    dataCount = 16     # 4*sizeof(float)
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FACTORY_COEF_W
    pack_into('ffff', s_buffer, MSG_INDEX_DATA, self.calCoefFactory[0].slope, self.calCoefFactory[1].slope, \
              self.calCoefFactory[0].intercept, self.calCoefFactory[1].intercept)
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
      raise TimeoutError('FactoryCoefficients_W: timeout error.')
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
      print('Error in FactoryCoefficients_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def FieldCoefficients_R(self):
    """
    This command reads the field calibration coefficients.  Each
    coefficient is a float.  The firmware applies the coefficients
    when calculating the voltage and temperature values for each
    channel.  The coefficients are applied immediately and stored in
    the EEPROM.
    """

    dataCount = 0
    replyCount = 1
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
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise TimeoutError('FieldCoefficients_R: timeout error.')
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
           self.calCoefField[0].slope ,= unpack_from('f', r_buffer, MSG_INDEX_DATA)
           self.calCoefField[1].slope ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+4)
           self.calCoefField[0].intercept ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+8)
           self.calCoefField[1].intercept ,= unpack_from('f', r_buffer, MSG_INDEX_DATA+12)
    if (result == False):
      print('Error in FieldCoefficients_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return result

  def FieldCoefficients_W(self):
    """
    This command writes the field calibration coefficients.  The
    microcontroller stores the values in EEPROM and restores them at
    power on. 
    """

    dataCount = 16     # 4*sizeof(float)
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FIELD_COEF_W
    pack_into('ffff', s_buffer, MSG_INDEX_DATA, self.calCoefField[0].slope, self.calCoefField[1].slope, \
              self.calCoefField[0].intercept, self.calCoefField[1].intercept)
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
      raise TimeoutError('FieldCoefficients_W: timeout error.')
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
      print('Error in FieldCoefficients_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  def FactoryCalDate_R(self):
    """
    This command reads the factory calibration date
    """
      
    dataCount = 0
    replyCount = 6
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
           mdate = datetime(year, month, day, hour, minute, second)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FactoryCalDate_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return mdate

  def FactoryCalDate_W(self, mdate):
    """
    This command writes the factory calibration date.
    """
      
    dataCount = 6
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FACTORY_CAL_DATE_W
    s_buffer[MSG_INDEX_DATA]           = mdate.year - 2000
    s_buffer[MSG_INDEX_DATA+1]         = mdate.month
    s_buffer[MSG_INDEX_DATA+2]         = mdate.day
    s_buffer[MSG_INDEX_DATA+3]         = mdate.hour
    s_buffer[MSG_INDEX_DATA+4]         = mdate.minute
    s_buffer[MSG_INDEX_DATA+5]         = mdate.second
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
    This command reads the field calibration date.
    """
      
    dataCount = 0
    replyCount = 6
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
           mdate = datetime(year, month, day, hour, minute, second)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in FieldCalDate_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return mdate

  def FieldCalDate_W(self, mdate):
    """
    This command writes the field calibration date.
    """
      
    dataCount = 6
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_FACTORY_CAL_DATE_W
    s_buffer[MSG_INDEX_DATA]           = mdate.year - 2000
    s_buffer[MSG_INDEX_DATA+1]         = mdate.month
    s_buffer[MSG_INDEX_DATA+2]         = mdate.day
    s_buffer[MSG_INDEX_DATA+3]         = mdate.hour
    s_buffer[MSG_INDEX_DATA+4]         = mdate.minute
    s_buffer[MSG_INDEX_DATA+5]         = mdate.second
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

  def ADCal(self):
    """
    This command causes the measurement loop to pause and an A/D
    system offset calibration to run.  The calibration requires
    approximately 800ms to complete then the measurement loop
    automatically returns to the current mode.  The calibration will
    run on both A/Ds simultaneously.  The command reply will not be
    sent until the calibration completes.
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
        print('Error in ADCal E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def CJCOffset_R(self):
    """
    This command reads the CJC user offset values.  Each value is
    added to the appropriate CJC sensor and factory offset reading
    prior to performing te NIST calculation for the temperature for
    that channel.
    """

    dataCount = 0
    replyCount = 32
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CJC_OFFSET_R
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
      raise TimeoutError('CJCOffset_R: timeout error.')
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
        self.CJC_offset = list(unpack_from('f'*8, r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
        print('Error in CJCOffset_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    
  def CJCOffset_W(self):
    """
    This command writes the CJC user offset values.  Each value is
    added to the appropriate CJC sensor and factory offset reading
    prior to performing te NIST calculation for the temperature for
    that channel.
    """

    dataCount = 32
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CJC_OFFSET_W
    for i in range(8):
      pack_into('f', s_buffer, MSG_INDEX_DATA+i, self.CJCOffset[i])
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
      raise TimeoutError('CJCOffset_W: timeout error.')
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
        self.CJC_offset = list(unpack_from('f'*8, r_buffer, MSG_INDEX_DATA))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
        print('Error in CJCOffset_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

      
  ####################################
  #         Alarm  Commands          #
  ####################################

  def AlarmConfig_R(self):
    """
    This command reads the temperature alarm configurations. There
    are configuration values and two threshold values for each of the
    thermocouple channels.
    
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
    
    threshold1:  float[8] the new alarm threshold 1 values in Celsius
    threshold2:  float[8] the new alarm threshold 2 values in Celsius
    """
    
    dataCount = 0
    replyCount = 72
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
      r_buffer = self.device.sock.recv(592)
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
        self.alarm_config     = list(unpack_from('B'*8, r_buffer, MSG_INDEX_DATA))
        self.alarm_threshold1 = list(unpack_from('f'*8, r_buffer, MSG_INDEX_DATA+8))
        self.alarm_threshold2 = list(unpack_from('f'*8, r_buffer, MSG_INDEX_DATA+40))
    try:
      if (result == False):
        raise ResultError
    except ResultError:
        print('Error in AlarmConfig_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AlarmConfig_W(self):
    """
    This command writesthe temperature alarm configurations. There
    are configuration values and two threshold values for each of the
    thermocouple channels. The configuration is stored in EEPROM and
    restored at power on.
    
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
      
    threshold1:  float[8] the new alarm threshold 1 values in Celsius
    threshold2:  float[8] the new alarm threshold 2 values in Celsius
    """

    dataCount = 72
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_ALARM_CONFIG_W
    for i in range(8):
      pack_into('B', s_buffer, MSG_INDEX_DATA+i, self.alarm_config[i])
    for i in range(8):
      pack_into('f', s_buffer, MSG_INDEX_DATA+8+i, self.alarm_threshold1[i])
    for i in range(8):
      pack_into('f', s_buffer, MSG_INDEX_DATA+40+i, self.alarm_threshold2[i])
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
      r_buffer = self.device.sock.recv(592)
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
        print('Error in AlarmConfig_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def AlarmStatus_R(self):
    """
    This command reads the status of the temperature
    alarms. If a bit is set an alarm condition exists or is latched
    on the corresponding channel. If the alarm is configured for
    latching then the status will not clear when the alarm condition
    is no longer present. It must be cleared by writing a 1 to the
    corresponding bit using the AlarmStatusW command.
    """

    dataCount = 0
    replyCount = 1
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
      raise TimeoutError('AlarmStatus_R: timeout error.')
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
        self.alarm_status = r_buffer[MSG_INDEX_DATA]
    try:
      if (result == False):
        raise ResultError
    except ResultError:
        print('Error in AlarmStatus_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

    return self.alarm_status

  def AlarmStatus_W(self):
    """
    This command clears the alarm status.  Writing a 1 to a bit will
    dlear the status for the corresponding channel. 
    """

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_ALARM_STATUS_W
    s_buffer[MSG_INDEX_DATA]           = self.alarm_status
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
      raise TimeoutError('AlarmStatus_W: timeout error.')
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
        print('Error in AlarmStatus_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


  #################################
  #       Counter Commands        #
  #################################

  def Counter(self):
    """
    This command reads the event counter
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
      print('Error in counter E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
      return-1
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
      raise TimeoutError('ResetCounter: timeout error.')
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
      print('Error in restCounter E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
  
    
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
      raise TimeoutError('Blink: timout error.')
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
      print('Error in blink E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))


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
      print('Error in reset E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def Status(self):
    """
    This command reads the device status
      bits 0-15: Reserved
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
           status = r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8 & 0xff)
    try:
      if (result == False):
        raise ResultError
    except ResultError:
      print('Error in status E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
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
      print('Error in networkConfig E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value


  #################################
  #       Memory  Commands        #
  #################################
  
  def ConfigMemory_R(self, address, count):
    """
    This command reads the nonvolatile configuration memory.  The configuration memory is
    """

    if (count > 16 or address > 0xf):
      return False

    dataCount = 4
    replyCount = count
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CONF_MEMORY_R
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
      print('Error in ConfigMemory_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
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

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_CONF_MEMORY_W
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
      print('Error in configMemory_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def UserMemory_R(self, address, count):
    """
    This command reads the nonvolatile user memory.  The user memory is
    3583 bytes (address 0 - 0xdff)
    
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

    s_buffer[MSG_INDEX_COMMAND]        = self.CMD_USER_MEMORY_R
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
      print('Error in UserMemory_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def UserMemory_W(self, address, count, data):
    """
    This command writes the nonvolatile user memory.  The user memory
    is 3583 bytes (address 0 - 0xdff). The amount of data to be
    written is inferred from the frame count - 2.  The maximum that
    can be written in one transfer is 1024 bytes.
    """

    if (count > 512 or address > 0xdff):
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
      print('Error in UserMemory_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

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

    s_buffer[MSG_INDEX_COMMAND]        = self.CDM_SETTINGS_MEMORY_R
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
      print('Error in SettingsMemory_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
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
      print('Error in SettingsMemory_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def BootloaderMemory_R(self, address, count):
    """
    This command reads the bootloader stored in nonvolatile FLASH
    memory.  The bootloader is located in program FLASH memory in two
    physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
    code and 0x1FC00000 - 0x1FC01FFF for C startup code and
    interrupts.  Reads may be performed at any time.
    
     address: the start address for reading (see above)
     count:   the number of bytes to read (max 1024)
    """

    if (count > 1024):
      return False

    if (not((address >= 0x1d000000 and address <= 0x1d007fff) or (address >= 0x1fc00000 and address <= 0x1fc01fff))):
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
      r_buffer = self.device.sock.recv(2048)
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
      print('Error in BootloaderMemory_R E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))
    return value

  def BootloaderMemory_W(self, address, count, data):
    """
    This command writes the bootloader stored in nonvolatile FLASH
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
    
    The writes are perfomred on 4-byte boundaries internally and it
    is recommended that the output data be sent in the same manner.
    The amount of data to be written is inferred frolm the frame
    count - 2. The maximum count value is 1024.
    """

    if (count > 1024):
      return False

    if (not((address >= 0x1d000000 and address <= 0x1d007fff) or (address >= 0x1fc00000 and address <= 0x1fc01fff))):
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
      r_buffer = self.device.sock.recv(2048)
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
      print('Error in BootloaderMemory_W E-TC.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

  def MACaddress(self):
    """
    Gets the MAC address
    """
    
    address = 0x0a
    value =  self.ConfigMemory_R(address, 6)
    self.device.MAC = (value[0]<<40) + (value[1]<<32) + (value[2]<<24) + (value[3]<<16) + (value[4]<<8) + value[5]
    return self.device.MAC

  @staticmethod
  def nBits8(num):
    """
    counts the number of bits in a number
    """

    count = 0
    for i in range(8):
      if ((num & 0x1) == 0x1):
        count += 1
      num >>= 0x1
    return count
