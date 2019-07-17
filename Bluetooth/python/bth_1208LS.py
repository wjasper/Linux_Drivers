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

import bluetooth 
from struct import *
from mccBluetooth import *

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

  # Aanalog Input
  SINGLE_ENDED    = 0
  DIFFERENTIAL    = 1

  # Analog Input Scan Options
  IMMEDIATE_TRANSFER_MODE = (0x1)
  BLOCK_TRANSFER_MODE     = (0x0)
  DIFFERENTIAL_MODE       = (0x2)
  SINGLE_ENDED_MODE       = (0x0)
  NO_TRIGGER              = (0x0)
  TRIG_EDGE_RISING        = (0x1 << 2)
  TRIG_EDGE_FALLING       = (0x2 << 2)
  TRIG_LEVEL_HIGH         = (0x3 << 2)
  TRIG_LEVEL_LOW          = (0x4 << 2)
  RETRIGGER_MODE          = (0x1 << 5)
  STALL_ON_OVERRUN        = (0x0)
  INHIBIT_STALL           = (0x1 << 7)

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

  # Status bit values */
  AIN_SCAN_RUNNING   = (0x1 << 1)
  AIN_SCAN_OVERRUN   = (0x1 << 2)
  NO_BATTERY         = (0x0)
  FAST_CHARGE        = (0x1 << 8)
  MAINTENANCE_CHARGE = (0x2 << 8)
  FAULT_CHARGING     = (0x3 << 8)
  DISABLE_CHARGING   = (0x4 << 8)


  voltage = 0
  status = 0

  def __init__(self, device):
    self.device = device        # inherit values from mccBluetoothDevice

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #  table_AInDE[NGAIN][NCHAN_DE]
    self.table_AInDE  = [[table(), table(), table(), table()],\
                         [table(), table(), table(), table()],\
                         [table(), table(), table(), table()],\
                         [table(), table(), table(), table()],\
                         [table(), table(), table(), table()],\
                         [table(), table(), table(), table()],\
                         [table(), table(), table(), table()],\
                         [table(), table(), table(), table()]]

    #  table_AInSE = [NCHAN_SE]                         
    self.table_AInSE = [table(), table(), table(), table(), table(), table(), table(), table()]
                         
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
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID   # increment frame ID with every send
    self.device.frameID = (self.device.frameID + 1) % 256              # increment frame ID with every send    
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT]          =  (dataCount & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] =  0xff - self.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)

    self.device.sendMessage(s_buffer)

    try:
      r_buffer = self.device.receiveMessage()
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
      print('Error in blink BTH-1208LS.  Status =', hex(r_buffer[MSG_INDEX_STATUS]))

