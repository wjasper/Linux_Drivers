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

import libusb1
import usb1
import time
import sys
from struct import *

class Error(Exception):
  ''' Base class for other exceptions.'''
  pass

class OverrunError(Error):
  ''' Raised when overrun on AInScan'''
  pass


# Base class for lookup tables of calibration coefficients (slope and offset)
class table:
  def __init__(self):
    self.slope = 0.0
    self.intercept = 0.0

class usb_1616FS:
    
  # Gain Ranges
  BP_10_00V   = 0x0         # Differential +/- 10.0 V
  BP_5_00V    = 0x1         # Differential +/- 5.00 V
  BP_2_50V    = 0x2         # Differential +/- 2.50 V
  BP_2_00V    = 0x3         # Differential +/- 2.00 V
  BP_1_25V    = 0x4         # Differential +/- 1.25 V
  BP_1_00V    = 0x5         # Differential +/- 1.00 V
  BP_0_625V   = 0x6         # Differential +/- 0.626 V
  BP_0_3125V  = 0x7         # Differential +/- 0.3125 V

  # Status Bits
  SYNC               = 0x1  # 0 = Sync slave,             1 = Sync master
  EXT_TRIG_EDGE      = 0x2  # 0 = trigger falleng edge,   1 = trigger rising edge
  UPDATE_MODE        = 0x80 # 1 = program memory update mode

  EEPROM             = 0    # read from EEPROM
  SRAM               = 1    # read from SRAM
  NCHAN              = 16   # max number of A/D channels in the device
  NGAIN              = 8    # max number of gain levels

  # Option values for AInScan
  AIN_EXECUTION      = 0x1  # 1 = single execution, 0 = continuous execution
  AIN_BURST_MODE     = 0x2  # 1 = burst I/O mode,   0 = normal I/O mode
  AIN_TRANSFER_MODE  = 0x4  # 1 = Immediate Transfer mode  0 = block transfer mode
  AIN_TRIGGER        = 0x8  # 1 = Use External Trigger
  AIN_EXTERN_SYNC    = 0x10 # 1 = Use External Sync
  AIN_DEBUG          = 0x20 # 1 = debug mode.

  DIO_PORTA       =  0
  DIO_PORTB       =  1
  DIO_DIR_IN      =  1
  DIO_DIR_OUT     =  0

  # Commands and Codes for USB1616FS HID reports
  # Digital I/O Commands
  DCONFIG          = 0x01   # Configure digital port
  DCONFIG_BIT      = 0x02   # Configure individual digital port bits
  DIN              = 0x03   # Read digital port
  DOUT             = 0x04   # Write digital port
  DBIT_IN          = 0x05   # Read digital port bit
  DBIT_OUT         = 0x06   # Write digital port bit

  # Analog Input Commands
  AIN              = 0x10   # Read analog input channel
  AIN_SCAN         = 0x11   # Scan analog channels
  AIN_STOP         = 0x12   # Stop input scan
  ALOAD_QUEUE      = 0x13   # Load the channel/gain queue

  # Counter Commands
  CINIT            = 0x20   # Initialize counter
  CIN              = 0x21   # Read Counter

  # Memory Commands
  MEM_READ         = 0x30   # Read Memory
  MEM_WRITE        = 0x31   # Write Memory

  # Miscellaneous Commands
  BLINK_LED        = 0x40   # Causes LED to blink
  RESET            = 0x41   # Reset USB interface
  SET_TRIGGER      = 0x42   # Configure external trigger
  SET_SYNC         = 0x43   # Configure sync input/ouput
  GET_STATUS       = 0x44   # Retrieve device status
  SET_CAL          = 0x45   # Set CAL output

  # Code Update Commands
  PREPARE_DOWNLOAD = 0x50   # Prepare for program memory download
  WRITE_CODE       = 0x51   # Write program memory
  READ_CHECKSUM    = 0x52   # Return program memory checksum
  WRITE_SERIAL     = 0x53   # Write new serial number to device
  UPDATE_CODE      = 0x54   # Update program memory and reset
  READ_CODE        = 0x55   # Read program memory

  def __init__(self, serial=None):
    self.productID = 0x0081                            # MCC USB-1616FS
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1616FS not found")

    for i in range(7):
      if sys.platform.startswith('linux'):
        if self.udev.kernelDriverActive(i):
          self.udev.detachKernelDriver(i)
          self.udev.resetDevice()
      # claim all the needed interfaces for AInScan
      self.udev.claimInterface(i)

    # need to get wMaxPacketSize
    self.wMaxPacketSize = self.getMaxPacketSize()

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #   self.Cal[channel][gain]  0 < chan < 16,  0 < gain < 7
    self.Cal = [[table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()], \
                [table(), table(), table(), table(), table(), table(), table(), table()]]

    # Use 2 point calibration points stored in the EEPROM
    # Read in the internal ground reference at 0x90 in the EEPROM
    addr = 0x90
    v0 ,= unpack('f', self.MemRead(addr, self.EEPROM, 4))
    # Calculate the corresponding calibrated value y0
    y0 = v0*65536./20. + 0x8000    # Calculate the corresponding calibrated value y0

    # Read in the internal reference for +/- 10V at 0x80 in the EEPROM (+5.0 Nominal)
    addr = 0x80
    v1 ,= unpack('f', self.MemRead(addr, self.EEPROM, 4))
    y1 = v1*65536./20. + 0x8000    # Calculate the corresponding calibrated value y0

    data = [0]*32
    addr = 0xb0                    # +/- 10V Uncalibrated readings
    data1 = unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    addr += 32
    data2 = unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    for i in range(16):
      data[i] = data1[i]
      data[i+16] = data2[i]

    for j in range(self.NCHAN):
      x0 = data[2*j]    # offset
      x1 = data[2*j+1]  # positive gain

      self.Cal[j][self.BP_10_00V].slope = (y1 - y0)/(x1 - x0)            # slope
      self.Cal[j][self.BP_10_00V].intercept = (y0*x1 - y1*x0)/(x1 - x0)  # intercept

    ################################################################

    # Calculate the corresponding calibrated value y0
    y0 = v0*65536./10. + 0x8000;

    # Read in the internal reference for +/- 5V at 0x84 in the EEPROM
    addr = 0x84
    v1 ,= unpack('f', self.MemRead(addr, self.EEPROM, 4))
    y1 = v1*65536./10. + 0x8000    # Calculate the corresponding calibrated value y1

    data = [0]*32    
    addr = 0xf0                    # +/- 5V Uncalibrated readings
    data1= unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    addr += 32
    data2 = unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    for i in range(16):
      data[i] = data1[i]
      data[i+16] = data2[i]

    for j in range(self.NCHAN):
      x0 = data[2*j]             # offset
      x1 = data[2*j + 1]         # positive gain

      self.Cal[j][self.BP_5_00V].slope = (y1 - y0)/(x1 - x0)            # slope
      self.Cal[j][self.BP_5_00V].intercept = (y0*x1 - y1*x0)/(x1 - x0)  # intercept

    ################################################################

    # Calculate the corresponding calibrated value y0
    y0 = v0*65536./4. + 0x8000;

    # Read in the internal reference for +/- 2V at 0x88 in the EEPROM
    addr = 0x88
    v1 ,= unpack('f', self.MemRead(addr, self.EEPROM, 4))
    y1 = v1*65536./4. + 0x8000    # Calculate the corresponding calibrated value y1

    data = [0]*32
    addr = 0x130                  # +/- 2V Uncalibrated readings
    data1= unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    addr += 32
    data2 = unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    for i in range(16):
      data[i] = data1[i]
      data[i+16] = data2[i]
      
    for j in range(self.NCHAN):
      x0 = data[2*j]             # offset
      x1 = data[2*j + 1]         # positive gain

      self.Cal[j][self.BP_2_00V].slope = (y1 - y0)/(x1 - x0)            # slope
      self.Cal[j][self.BP_2_00V].intercept = (y0*x1 - y1*x0)/(x1 - x0)  # intercept
      self.Cal[j][self.BP_2_50V].slope = (y1 - y0)/(x1 - x0)            # slope
      self.Cal[j][self.BP_2_50V].intercept = (y0*x1 - y1*x0)/(x1 - x0)  # intercept

    ################################################################

    # Calculate the corresponding calibrated value y0
    y0 = v0*65536./2. + 0x8000;

    # Read in the internal reference for +/- 1V at 0x8c in the EEPROM
    addr = 0x8c
    v1 ,= unpack('f', self.MemRead(addr, self.EEPROM, 4))
    y1 = v1*65536./2. + 0x8000    # Calculate the corresponding calibrated value y1

    data = [0]*32    
    addr = 0x170                    # +/- 1V Uncalibrated readings
    data1= unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    addr += 32
    data2 = unpack('H'*16, self.MemRead(addr, self.EEPROM, 32))
    for i in range(16):
      data[i] = data1[i]
      data[i+16] = data2[i]

    for j in range(self.NCHAN):
      x0 = data[2*j]             # offset
      x1 = data[2*j + 1]         # positive gain

      self.Cal[j][self.BP_1_00V].slope = (y1 - y0)/(x1 - x0)             # slope
      self.Cal[j][self.BP_1_00V].intercept = (y0*x1 - y1*x0)/(x1 - x0)   # intercept
      self.Cal[j][self.BP_1_25V].slope = (y1 - y0)/(x1 - x0)             # slope
      self.Cal[j][self.BP_1_25V].intercept = (y0*x1 - y1*x0)/(x1 - x0)   # intercept
      self.Cal[j][self.BP_0_625V].slope = (y1 - y0)/(x1 - x0)            # slope
      self.Cal[j][self.BP_0_625V].intercept = (y0*x1 - y1*x0)/(x1 - x0)  # intercept
      self.Cal[j][self.BP_0_3125V].slope = (y1 - y0)/(x1 - x0)           # slope
      self.Cal[j][self.BP_0_3125V].intercept = (y0*x1 - y1*x0)/(x1 - x0) # intercept
    
  #################################
  #     Digital I/O  Commands     #
  #################################

  def DConfig(self, direction):
    """
    This command sets the direction of the DIO port to input or output.  The I/O bits can be
    individually configured using the DConfigBit function.
       direction:  0 - output,   1 - input
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.DCONFIG  # HID output
    wIndex = 0                         # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DCONFIG, direction], timeout = 100)

  def DConfigBit(self, bit_num, direction):
    """
    This command sets the direction of the an individual DIO bit to input or output

      bit_num:    the bit to configure (0-7)      
      direction:  0 - output,   1 - input
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                          # HID Set_Report
    wValue =  (2 << 8) | self.DCONFIG_BIT  # HID output
    wIndex = 0                             # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DCONFIG_BIT, bit_num, direction], timeout = 100)

  def DIn(self):
    """
    This command reads the current state of the digital port.  The
    digital port is bitwise configurable, so may be any combination of
    inputs and outputs.  The return value will be the value seen at
    the port pins.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                  # HID Set_Report
    wValue =  (2 << 8) | self.DIN  # HID output
    wIndex = 0                     # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DIN], timeout = 100)
    value = unpack('BB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, 2, timeout = 100))    
    return value[1]

  def DOut(self, value):
    """
     This command writes data to the DIO port bits that are configured as outputs.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                   # HID Set_Report
    wValue =  (2 << 8) | self.DOUT  # HID output
    wIndex = 0                      # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DOUT, value], timeout = 100)

  def DBitIn(self, bit_num):
    """
    This command reads an individual digital port bit.  It will return
    the value seen at the port pin, so may be used for input or output
    bit.

    bit_num:  the bit to read (0-7)    
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.DBIT_IN  # HID output
    wIndex = 0                         # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DBIT_IN, bit_num], timeout = 100)
    value = unpack('BB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, 2, timeout = 100))    
    return value[1]

  def DBitOut(self, bit_num, value):
    """
    This command writes an individual digital port bit.  

    bit_num: the bit to write (0-7)    
    value:   the value to write to the bit (0 or 1)
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.DBIT_OUT  # HID output
    wIndex = 0                         # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DBIT_OUT, bit_num, value], timeout = 100)

  #################################
  #    Analog Input  Commands     #
  #################################

  def AIn(self, channel, gain):
    """
    This command reads the value from an analog input channel, setting the
    desired gain range first.  The returned value is a 2's complement signed
    16-bit number.
    
      channel = the channel to read (0-7 differential, 8-15 single ended)
      gain    = the gain range to select (0-7)
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                  # HID Set_Report
    wValue =  (2 << 8) | self.AIN  # HID output
    wIndex = 0                     # interface

    if channel < 0 or channel > 7:
      raise ValueError('AIn: channel out of range.')
      return
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.AIN, channel, gain], timeout = 100)
    value = unpack('BBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, 3, timeout = 1000))    
    value = value[1] | (value[2] << 8)
    value = int(value*self.Cal[channel][gain].slope + self.Cal[channel][gain].intercept)
    try:
      if value > 0xffff:
        raise SaturationError
    except SaturationError:
      value = 0xffff
    if value < 0:
      value = 0
    if value >= 0x8000:
      value -= 0x8000
    else:
      value = 0x8000 - value
      value *= -1

    return value

  def AInStop(self):
    """
    This command stops the analog scan (if running)
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                       # HID Set_Report
    wValue =  (2 << 8) | self.AIN_STOP  # HID output
    wIndex = 0                          # interface
    value = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.AIN_STOP], timeout = 100)

  def ALoadQueue(self, gain):
    """
    The device can scan analog input channels with different gain settings.  This
    function provides the mechanism for configuring each channel with a unique
    range
    
    gainQueue: the 8 gain values for the 16 channels.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                          # HID Set_Report
    wValue =  (2 << 8) | self.ALOAD_QUEUE  # HID output
    wIndex = 0                             # interface
    buf = [0]*(self.NCHAN+1)
    buf[0] = self.ALOAD_QUEUE              # first byte is the report ID.
    for i in range(self.NCHAN):
      buf[i+1] = gain[i]
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, buf, timeout = 100)

  #################################
  #     Counter  Commands         #
  #################################

  def CInit(self):
    """
    This command initializes the event counter and resets
    the count to zero
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                    # HID Set_Report
    wValue =  (2 << 8) | self.CINIT  # HID output
    wIndex = 0                       # interface
    value = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.CINIT], timeout = 100)

  def CIn(self):
    """
    This function reads the 32-bit event counter on the device.  This
    counter tallies the transitions of an external input attached to
    the CTR pin on the screw terminal of the device.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                  # HID Set_Report
    wValue =  (2 << 8) | self.CIN  # HID output
    wIndex = 0                     # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.CIN], timeout = 100)
    value = unpack('BBBBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, 5, timeout = 100))    
    return (value[1] | (value[2]<<8) | (value[3]<<16) | (value[4]<<24))
    
  #################################
  #     Memory  Commands          #
  #################################

  def MemRead(self, address, mem_type, count):
    """
    This command reads data from the configuration memeory (EEPROM).
    All of the memory may be read

    Address 0x000 - 0x07F are reserved for firmware data
    Address 0x080 - 0x3FF are available for use as calibration or user data
    
     address:  the start addess for the read
     mem_type: the memory type to read (0 = EEPROM, 1 = SRAM)
     count:    the number of byes to read (62 max)
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                       # HID Set_Report
    wValue =  (2 << 8) | self.MEM_READ  # HID output
    wIndex = 0                          # interface
    value = [0]*count
    data = bytearray(64)
    value = bytearray(count)

    if (count > 62):
      raise ValueError('MemRead: max count is 62')
      return
    buf = [self.MEM_READ, address & 0xff, (address >> 8) & 0xff, mem_type, count] 
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, buf, timeout = 5000)
    try:
      # always read 63 bytes regardless.  Only the first count are meaningful.
      data = self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, 63, timeout = 5000)
    except:
      print('MemRead Error')
    for i in range(count):
      value[i] = data[i+1]
    return value

  def MemWrite(self, address, count, data):
    """
    This command writes data to the non-volatile EEPROM memory on the
    device.  The non-volatile memory is used to store calibration
    coefficients, system information and user data.  Locations 0x000 -
    0x07F are reserved for firmware use and may not be written.

    address: the start address for the write
    count:   the number of bytes to write (59 max)
    data:    the data to be written (59 bytes max)
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                        # HID Set_Report
    wValue =  (2 << 8) | self.MEM_WRITE  # HID output
    wIndex = 0                           # interface

    if (count > 59):
      raise ValueError('MemWrite: max count is 59')
      return

    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.MEM_WRITE,address, data[:count]], timeout = 100)

  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self):
    """
    This command causes the LED to blink.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                        # HID Set_Report
    wValue =  (2 << 8) | self.BLINK_LED  # HID output
    wIndex = 0                           # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.BLINK_LED], timeout = 100)
    
  def Reset(self):
    """
    The command causes the device to perform a soft reset. The device
    disconnect from the USB bus and resets its microcontroller.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.RESET    # HID output
    wIndex = 0                         # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.RESET], timeout = 100)

  def SetTrigger(self, trig_type):
    """
    This command configures the external trigger for analog input.  The
    trigger may be configured to activate with either a logic rising
    edge or falling edge input.  Once the trigger is received, the analog
    input will proceed as configured.  The EXTTRIG option must be used
    in the AInScan command to utilize this feature.
    
      trig_type:  the type of trigger  (0 = external trigger falling edge, 1 = external trigger rising edge)
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                          # HID Set_Report
    wValue =  (2 << 8) | self.SET_TRIGGER  # HID output
    wIndex = 0                             # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.SET_TRIGGER], timeout = 100)

  def SetSync(self, sync_type):
    """
    This command configures the sync signal. The sync signal may be
    used to synchronize the analog input scan of multiple devices.
    When multiple devices are to be used, one device is selected as
    the master and the rest as slaves.  The sync signal of all
    devices must be wired together.  The master will output a pulse
    every sample, and all of the devices will acquire their samples
    simultaneously.  This may also be used to pace one or more
    devices from an external TTL/CMOS clock signal (max rate = 50
    kHz) 

    This may also be used with an external trigger; the external
    trigger signal should be brought to the master device, and all
    devices will begin sampling when the master is triggered.

    If a device is configred as a slave, it will not
    acquire data when given an AInScan ommand until it detects a
    pulse on the sync input.  
    
    sync_type: 0 = master,
               1 = slave 
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                        # HID Set_Report
    wValue =  (2 << 8) | self.SET_SYNC   # HID output
    wIndex = 0                           # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.SET_SYNC, sync_type], timeout = 100)

  def Status(self):
    """
    This command retrives the status of the device
     Bit 0: 0 = Sync slave,               1 = sync master
     Bit 1: 0 = Trigger falling edge,     1 = trigger rising edge
     Bits 2-14 unused.
     Bit 15: 1 = program memory update mode
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                         # HID Set_Report
    wValue =  (2 << 8) | self.GET_STATUS  # HID output
    wIndex = 0                            # interface
    value = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.GET_STATUS], timeout = 100)
    value = unpack('BBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, 3, timeout = 100))

    return (value[1] | (value[2]<<8)) & 0xf

  def SetCal(self, setting):
    """
    This command sets the voltage on the CAL output. The mux will be
    disabled on power up.
    
       setting:  bits 0-2: 0 = 0V
                           1 = 2.5V
                           2 = 1.25V
                           3 = 2.5V
                           4 = 5V
                           5 = Temp sensor
                 bit 3:  polarity 0 = positive,  1 = negative
                 bit 4:  0 = disable (inputs connected to analog channels)
                         1 = enabled (CAL mode)
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.SET_CAL  # HID output
    wIndex = 0                         # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.SET_CAL, setting], timeout = 100)

  #################################
  #     Code Update Commands      #
  #################################

  def PrepareDowload(self):
    """
    This command puts the device into code update mode, initializes
    the external SRAM, and resets the program memory checksum to 0 The
    unlock code must be correct as a further safety device.  Call this
    once before sending code with WriteCode.  If not in code update
    mode, any downloaded code will be ignored.

    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                               # HID Set_Report
    wValue =  (2 << 8) | self.PREPARE_DOWNLOAD  # HID output
    wIndex = 0                                  # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.PREPARE_DOWNLOAD, 0xad], timeout = 100)

  def WriteCode(self, address, count, data):
    """
    This command sends the new program memory image to the device.  The
    downloaded program memory image is stored in external SRAM.  The
    image will be written in FLASH program memory when the UpdateCode
    command is issued (updates must be enabled with UnlockCode first.)
    A running checksum will be calculated as the program memory image
    is sent, and the host should compare its own checksum with this
    value (retrieved with ReadChecksum) prior to sending the
    UpdateCode command.
    
    The address ranges are:
    
    0x000000 - 0x007AFF:  FLASH program memory
    0x200000 - 0x200007:  ID memory (serial number is stored here on main micro)
    0x300000 - 0x30000F:  CONFIG memory (processor configuration data)
    0xF00000 - 0xF03FFF:  EEPROM memory
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                         # HID Set_Report
    wValue =  (2 << 8) | self.WRITE_CODE  # HID output
    wIndex = 0                            # interface

    if (count > 58):
      raise ValueError('WriteCode: count greater than 58')
      return
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, \
            [self.WRITE_CODE, address&0xff, (address>>8)&0xff, (address>>16)&0xff, count, data[0:count]], timeout = 100)
    
  def ReadChecksum(self):
    """
    This command return the current program memory checksum.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                            # HID Set_Report
    wValue =  (2 << 8) | self.READ_CHECKSUM  # HID output
    wIndex = 0                               # interface

  def WriteSerial(self, serial):
    """
    This command sends a new serial number to the device.  The serial number consists
    of 8 bytes, typically ASCII numberic or hexadecimal digits (i.e. "00000001").
    Note: The new serial number will be programmed but not used until hardware reset.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                           # HID Set_Report
    wValue =  (2 << 8) | self.WRITE_SERIAL  # HID output
    wIndex = 0                              # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.WRITE_SERIAL, serial[0:8]], timeout = 100)

  def UpdateCode(self):
    """
    This command performs the program memeory update and resets the processor.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                           # HID Set_Report
    wValue =  (2 << 8) | self.UPDATE_CODE   # HID output
    wIndex = 0                              # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.UPDATE_CODE], timeout = 100)

  def ReadCode(self):
    """
    This command reads from program memory.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                           # HID Set_Report
    wValue =  (2 << 8) | self.READ_CODE     # HID output
    wIndex = 0                              # interface

    if (count > 62):
      raise ValueError('ReadCode: count greater than 62')
      return
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, \
             [self.READ_CODE, address&0xff, (address>>8)&0xff, (address>>16)&0xff, count], timeout = 100)                

    value = unpack('B'*(count+1),self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 2, count+1, timeout = 100))    
    return (value[1,count+1])

  
  def volts(self, gain, num):
    # converts signed short value to volts
    if gain == self.BP_10_00V:
      volt = num * 10.0 / 0x7fff
    elif gain == self.BP_5_00V:
      volt = num * 5.0 / 0x7fff
    elif gain == self.BP_2_50V:
      volt = num * 2.5 / 0x7fff
    elif gain == self.BP_2_00V:
      volt = num * 2.0 / 0x7fff
    elif gain == self.BP_1_25V:
      volt = num * 1.25 / 0x7fff
    elif gain == self.BP_1_0V:
      volt = num * 1.00 / 0x7fff
    elif gain == self.BP_0_625V:
      volt = num * 0.625 / 0x7fff
    elif gain == self.BP_0_3125V:
      volt = num * 0.3125 / 0x7fff
    return volt

  def printStatus(self):
    status = self.Status()
    print('**** USB-1616FS Status ****')
    if status & self.SYNC:
      print('    Sync Master')
    else:
      print('    Sync Slave')
    if status & self.EXT_TRIG_EDGE:
      print('    Trigger rising edge')
    else:
      print('    Trigger falling edge')
    if status & self.UPDATE_MODE:
      print('    Program memory update mode')

  ##############################################################################################

  def openByVendorIDAndProductID(self, vendor_id, product_id, serial):
    self.context = usb1.USBContext()
    for device in self.context.getDeviceIterator(skip_on_error=False):
      if device.getVendorID() == vendor_id and device.getProductID() == product_id:
        if serial == None:
          return device.open()
        else:
          if device.getSerialNumber() == serial:
            return device.open()
    return None      

  def getSerialNumber(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getSerialNumber())

  def getProduct(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getProduct())

  def getManufacturer(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getManufacturer())

  def getMaxPacketSize(self):
    with usb1.USBContext() as context:
      for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() == 0x9db and device.getProductID() == self.productID:
          return(device.getMaxPacketSize0())
