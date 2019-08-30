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

import libusb1
import usb1
import time
import sys
from struct import *
from datetime import datetime
from mccUSB import *

class usb_20x(mccUSB):
  """
    Calibration memory map
  |===================================================================|
  |    Address    |        Value                                      |
  |===================================================================|
  | 0x000         | Slope, channel 0 (float)                          |
  |-------------------------------------------------------------------|
  | 0x004         | Intercept, channel 0 (float)                      |
  |-------------------------------------------------------------------|
  | 0x008         | Slope, channel 1 (float)                          |
  |-------------------------------------------------------------------|
  | 0x00c         | Intercept, channel 1 (float)                      |
  |-------------------------------------------------------------------|
  | 0x010         | Slope, channel 2 (float)                          |
  |-------------------------------------------------------------------|
  | 0x014         | Intercept, channel 2 (float)                      |
  |-------------------------------------------------------------------|
  | 0x018         | Slope, channel 3 (float)                          |
  |-------------------------------------------------------------------|
  | 0x01c         | Intercept, channel 3 (float)                      |
  |-------------------------------------------------------------------|
  | 0x020         | Slope, channel 4 (float)                          |
  |-------------------------------------------------------------------|
  | 0x024         | Intercept, channel 4 (float)                      |
  |-------------------------------------------------------------------|
  | 0x028         | Slope, channel 5 (float)                          |
  |-------------------------------------------------------------------|
  | 0x02c         | Intercept, channel 5 (float)                      |
  |-------------------------------------------------------------------|
  | 0x030         | Slope, channel 6 (float)                          |
  |-------------------------------------------------------------------|
  | 0x034         | Intercept, channel 6 (float)                      |
  |-------------------------------------------------------------------|
  | 0x038         | Slope, channel 7 (float)                          |
  |-------------------------------------------------------------------|
  | 0x03c         | Intercept, channel 7 (float)                      |
  |-------------------------------------------------------------------|
  | 0x040         | Calibration date, year (byte)                     |
  |-------------------------------------------------------------------|
  | 0x041         | Calibration date, month (byte)                    |
  |-------------------------------------------------------------------|
  | 0x042         | Calibration date, day (byte)                      |
  |-------------------------------------------------------------------|
  | 0x043         | Calibration date, hour (byte)                     |
  |-------------------------------------------------------------------|
  | 0x044         | Calibration date, minute (byte)                   |
  |-------------------------------------------------------------------|
  | 0x045         | Calibration date, second (byte)                   |
  |-------------------------------------------------------------------|
  |               |                                                   |
  |-------------------------------------------------------------------|
  | 0x2F8 - 0x2ff | Serial number (8 characters)                      |
  |===================================================================|

    User memory map
  |===================================================================|
  |    Address    |        Value                                      |
  |===================================================================|
  | 0x000 - 0x0ff | Available for UL use                              |
  |===================================================================|

    MBD memory map
  |===================================================================|
  |    Address    |        Value                                      |
  |===================================================================|
  | 0x000 - 0x03f | ID, 64 bytes                                      |
  |-------------------------------------------------------------------|
  | 0x040 - 0x3ff | Available for MBD use                             |
  |===================================================================|
   """

    
  # Commands and Report ID for USB-CTR
  ###########################################
  #  Digital I/O Commands 
  DTRISTATE           = 0x00  # Read/Write digital port tristate register
  DPORT               = 0x01  # Read digital port pins / write output latch register
  DLATCH              = 0x02  # Read/Write digital port output latch register

  # Analog Input Commands
  AIN                 = 0x10  # Read analog input channel
  AIN_SCAN_START      = 0x11  # Start analog input scan
  AIN_SCAN_STOP       = 0x12  # Stop analog input scan
  AIN_SCAN_CLEAR_FIFO = 0x15  # Clear the analog input scan FIFO
  AIN_BULK_FLUSH      = 0x16  # Flush the bulk IN endpoint with empty packets

  # Analog Output Commands
  AOUT                = 0x18  # Read/write Analog output channel

  # Counter Commands
  COUNTER             = 0x20  # Read/reset event counter

  # Memory Commands
  CAL_MEMORY          = 0X30  # Read/write calibration memory
  USER_MEMORY         = 0x31  # Read/write user memory
  MBD_MEMORY          = 0x32  # Read/write MBD memory

  # Miscellaneous Commands
  BLINK_LED               = 0x41  # Blink the LED
  RESET                   = 0x42  # Reset the device
  STATUS                  = 0x44  # Read device status
  SERIAL                  = 0x48  # Read/write serial number
  DFU                     = 0x50  # Enter device firmware upgrade mode

  # MBD Commands
  MBD_COMMAND             = 0x80  # Text-based MBD command/response
  MBD_RAW                 =x 0x81 # Raw MBD response

  MAX_PACKET_SIZE         = 64    # max packet size for FS device
  NCHAN                   = 8     # max number of A/D channels in the device

  # Status bit values
  AIN_SCAN_RUNNING        = 0x1 << 1
  AIN_SCAN_OVERRUN        = 0x1 << 2

  def __init__(self, serial=None):

    # need to get wMaxPacketSize
    self.wMaxPacketSize = self.getMaxPacketSize()
    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #
    #  table_AIn[NGAIN]
    self.table_AIn  = [[table(), table(), table(), table(), table(), table(), table(), table()]]
    self.BuildGainTable()

    def BuildGainTable(self):
      
    """
    Builds a lookup table of single ended multiplexed calibration
    coefficents to translate values into voltages: The calibration
    coefficients are stored in onboard FLASH memory on the device in
    IEEE-754 4-byte floating point values.

       calibrated code = code * slope + intercept
    """
    address = 0x000
    for chan in range(self.NCHAN):
      self.table_AIn[chan].slope, = unpack('f', self.CalMemoryR(address, 4))
      address += 4
      self.table_AIn[chan].intercept, = unpack('f', self.CalMemoryR(address, 4))
      address += 4

  def CalDate(self):
    """
    get the manufacturers calibration data (timestamp) from the
    Calibration memory
    """
    # get the year (since 2000)
    address = 0x40
    data ,= unpack('B', self.CalMemoryR(address, 1))
    year  = 2000+data

    # get the month
    address = 0x41
    month ,= unpack('B', self.CalMemoryR(address, 1))

    # get the day
    address = 0x42
    day ,= unpack('B', self.CalMemoryR(address, 1))

    # get the hour
    address = 0x43
    hour ,= unpack('B', self.CalMemoryR(address, 1))
    
    # get the minute
    address = 0x44
    minute ,= unpack('B', self.CalMemoryR(address, 1))

    # get the second
    address = 0x45
    second ,= unpack('B', self.CalMemoryR(address, 1))

    mdate = datetime(year, month, day, hour, minute, second)
    return mdate

  #############################################
  #        Digital I/O Commands               #
  #############################################


  #############################################
  #        Analog Input Commands              #
  #############################################

    
  #############################################
  #        Analog Output Commands             #
  #############################################

    
  #############################################
  #           Counter Commands                #
  #############################################
    
  #############################################
  #        Memory Commands                    #
  #############################################

    
  #############################################
  #        Miscellaneous Commands             #
  #############################################


class usb_201(usb_20x):
  USB_201_PID = 0x0113

  def __init__(self, serial=None):
    self.productID = USB_201_PID    # usb-201
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-201 not found")
      return
    usb_ctr.__init__(self)

class usb_202(usb_20x):
  USB_202_PID = 0x12b

  def __init__(self, serial=None):
    self.productID = USB_202_PID    # usb-202
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-202 not found")
      return
    usb_ctr.__init__(self)

 class usb_204(usb_20x):
  USB_204_PID = 0x0114

  def __init__(self, serial=None):
    self.productID = USB_204_PID    # usb-204
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-204 not found")
      return
    usb_ctr.__init__(self)

class usb_205(usb_20x):
  USB_205_PID = 0x12c

  def __init__(self, serial=None):
  self.productID = USB_205_PID    # usb-205
  self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
  if not self.udev:
    raise IOError("MCC USB-205 not found")
    return
  usb_ctr.__init__(self)

  
