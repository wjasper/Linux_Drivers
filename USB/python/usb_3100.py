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

import hid
import time
from struct import *
from mccUSB import *
    
class usb_3100(mccUSB):    # HID USB-31XX devices
  DIO_DIR_IN   = 0x1  # input direction
  DIO_DIR_OUT  = 0x0  # output direction

  UP_10_00V  = 0  # 0 - 10V
  BP_10_00V  = 1  # +/- 10V
  I_0_20_mA  = 2  # 0-20 mA

  SYNC_MASTER  = 0
  SYNC_SLAVE   = 1
  NCHAN        = 16   # number of channels

  # Commands and Codes for USB-3100  HID reports
  # Digital I/O Commands
  DCONFIG          = 0x01  # Configure digital port
  DCONFIG_BIT      = 0x02  # Configure individual digital port bits
  DIN              = 0x03  # Read digital port
  DOUT             = 0x04  # Write digital port
  DBIT_IN          = 0x05  # Read digital port bit
  DBIT_OUT         = 0x06  # Write digital port bit

  # Analog Output Commands
  AOUT             = 0x14  # Write analog output channel
  AOUT_SYNC        = 0x15  # Synchronously update outputs
  AOUT_CONFIG      = 0x1C  # Configure analog output channel

  # Counter Commands
  CINIT            = 0x20  # Initialize counter
  CIN              = 0x21  # Read Counter

  # Memory Commands
  MEM_READ         = 0x30  # Read Memory
  MEM_WRITE        = 0x31  # Write Memory

  # Miscellaneous Commands
  BLINK_LED        = 0x40  # Causes LED to blink
  RESET            = 0x41  # Cause the device to reset
  SET_SYNC         = 0x43  # Configure sync input/output
  GET_STATUS       = 0x44  # Retrieve device status

  # Code Update Commands
  PREPARE_DOWNLOAD = 0x50  # Prepare for program memory download
  WRTE_CODE        = 0x51  # Write program memory
  WRITE_SERIAL     = 0x53  # Write a new serial number to device

  productID        = 0
  
  def __init__(self):
    try:
      self.h = hid.device()
    except:
      print('Error in creating hid device')
      return

    self.CalTable = [table(), table(), table(), table(), table(), table(), table(), table(), \
                     table(), table(), table(), table(), table(), table(), table(), table()]


  #################################
  #     Digital I/O  Commands     #
  #################################

  def DConfigPort(self, direction):
    """
    This command sets the direction of the DIO port bits to input or output
    direction:  0 = output,  1 = input
    """
    self.h.write([self.DCONFIG, direction])

  def DConfigBit(self, bit_num, direction):
    """
    This command sets the direction of individual DIO bits to input or output.
      bitnum:    the bit to configure (0-7)
      direction: 0 = output,   1 = input
    """
    self.h.write([self.DCONFIG_BIT, bit_num, direction])

  def DIn(self):
    """
    This command reads the current state of the digital port.  The
    return value will be the value seen at the port pins.
    """
    self.h.write([self.DIN])
    try:
      value = self.h.read(2,500)
    except:
      print('DIn: error in reading.')
    return(value[1])

  def DOut(self, value):
    """
    This command writes data to the DIO port bits that are configured
    as outputs.
      value:  value to write to the port
    """
    self.h.write([self.DOUT, value])

  def DBitIn(self, bit_num):
    """
    This command reads an individual digital port bit.  It will return
    the value seen at the port pin, so may be used for an input or
    output bit.
      bit_num:            The bit to read (0-7)
    """
    self.h.write([self.DBIT_IN, bit_num])  
    try:
      value = self.h.read(2,500)
    except:
      print('DBitIn: error in reading.')
    return(value[1])

  def DBitOut(self, bit_num, value):
    """
    This command writes an individual digital port bit.  
    
     bit_num:  The bit to read (0-7)
     value:    The value to write to the bit (0 or 1)
    """
    self.h.write([self.DBIT_OUT, bit_num, value])

    
  #################################
  #   Analog Output  Commands     #
  #################################

  def AOut(self, channel, value, update = 0):
    """
    This command writes the value to an analog output channel.  The
    value is a 16-bit unsigned value.  The output range for a channel
    may be set with AOutConfig.  The equation for the output voltage is:

    V_out = (k - 2^15/2^15) * 10V    for +/- 10V range

    V_out = (k/2^16) * 10V           for 0-10V range 

    where k is the value written to the device.  The equation for
    current output is:
    
    I_out = (k/2^16) * 5V/249 Ohm    for 0-20mA range.

    where k is the value written to the device.  The current output
    value is independent of the rnage selection.  A current and
    voltage are always ouptut for a given vluae.

    channel: the channel to write (0-15)
    value:   the value to write
    update:  update mode:  0 = update immediately, 1 = update on sync signal
    """

    if update != 1:
      update = 0
    if channel >= self.NCHAN:
      raise ValueError('AOut: channel out of range.')
      return

    value = int(self.CalTable[channel].slope*value + self.CalTable[channel].intercept)

    if value > 0xffff:
      value = 0xffff
    elif value < 0:
      value = 0
    
    self.h.write([self.AOUT, channel, (value & 0xff), (value>>8) & 0xff, update])

  def AOutSync(self):
    """
    This command sends the output update signal to all D/A converters.
    If the sync signal is configured as an input with SetSync, this
    command has no effect; if the sync signal is configured as an
    output, a pulse will be generated on the sync pin.
    """
    self.h.write([self.AOUT_SYNC])

  def AOutConfig(self, channel, gain):
    """
    This command configures the output range of an analog outout
    channel.  The output will be set to 0V in the selected range, so
    an AOut is not needed after this command.

    channel: the channel to write (0-15)
    gain:    the output gain (don't care when doing current output)
             0 = 0-10V 1 = +/- 10V
    """
    if channel >= self.NCHAN:
      raise ValueError('AOutConfig: channel out of range')
      return
    
    self.h.write([self.AOUT_CONFIG, channel, gain])

    if gain == self.UP_10_00V:
      address = 0x100 + 0x10*channel;
    elif gain == self.BP_10_00V:
      address = 0x108 + 0x10*channel
    elif gain == self.I_0_20_mA:
      address = 0x200 + 0x8*channel
    else:
      raise ValueError('AOutConfig: gain value out of range')
      return

    self.CalTable[channel].slope ,= unpack('f', self.MemRead(address, 4))
    address += 4
    self.CalTable[channel].intercept ,= unpack('f', self.MemRead(address, 4))


  #################################
  #     Counter  Commands         #
  #################################

  def CInit(self):
    """
     This command initializes the event counter and resets the count to zero
    """
    self.h.write([self.CINIT])

  def CIn(self):
    """
    This function reads the 32-bit event counter on the device.  This
    counter tallies the transitions of an external input attached to
    the CTR pin on the screw terminal of the device.
    """
    self.h.write([self.CIN])
    try:
      value = self.h.read(5, 100)
    except:
      raise ValueError('Error in CIn.')
      return
    return (value[1] | (value[2]<<8) | (value[3]<<16) | (value[4]<<24))

  #################################
  #     Memory  Commands          #
  #################################
  def MemRead(self, address, count):
    """
    This command reads data from the configuration memory (EEPROM or
    FLASH).  All of the memory may be read.  The EEPROM address are
    from 0x0000 to 0x00FF.  The FLASH addresses are from 0x0100 -
    0x02FF.

    address:  the start address for the read.
    type:     not used
    count:    the number of bytes to read (62 max)
    """

    if (count > 62):
      raise ValueError('MemRead: max count is 62.')
      return
    self.h.write([self.MEM_READ, (address & 0xff), (address>>8 & 0xff), 0, count])
    try:
      value = self.h.read(count+1, 100)
    except:
      print('MemRead: read error.')
    return(bytes(value[1:count+1]))

  def MemWrite(self, address, count, data):
    """
    This command writes to the non-volatile memory on the device.  The
    non-volatile memory is used to store calibration coefficients,
    system information, and user data.  There are 256 bytes of EEPROM
    (addresses (0x0000- 0x00FF) available for general use and 512
    bytes of FLASH (addresses 0x0100-0x02FF) used for calibration.

    The EEPROM may be written at any address with 1-59 byes of data in a write.

    The FLASH memory has stricter requirements for writing.  The
    memory is written in blocks of 32 bytes and erased in sectors of
    64 bytes.  The FLASH memory must be erased before being written.
    To ensure this, when writing to FLASH, the firmware will AND the
    start address with 0xFFE0.  If the bit 0x0020 is clear, this write
    is the first 32 bytes of a sector and the sector will be erased.
    Once erased, the first block will be written.  Do not have
    starting addresses to FLASH that don't begin on a 32-byte
    boundary.  The count should be 32 bytes when writing to FLASH; any
    more than that will be truncated.

    The FLASH memory is reserved for calibration data and requires the
    unlock sequence prior to writing.

    Unlock sequence: Write 2 bytes with the value 0xAA55 to address
    0x500 to unlock.  The unlock status can be checked with GetStatus.
    The unlock will remian in effect until the device is powered off
    or reset.

    address:  the start address for the write
    count:    the number of bytes to write (59 max)
    data:     the data to be written (59 bytes max)
    """
    self.h.write([self.MEM_WRITE, (address & 0xff), (address>>8 & 0xff), count, data[0:count]])

    
  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self, count):
    """
     This command causes the LED to flash several times.
    """
    self.h.write([self.BLINK_LED, count])

  def Reset(self):
    """
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
    """
    self.h.write([self.RESET])

  def SetSync(self, direction):
    """
    This command configures the sync signal.  The sync signal may be
    used to synchronize the analog output updates.  When multiple
    devices are to be used, one device is selected as the master and
    the rest as slaves.  The sync signal of all devices must be wired
    together.  The master will output a pulse when the AOutSync
    command is issued, and all of the devices will update their
    outputs simultaneously.  The sunc signal is rising edge triggered.
    
    The device will switch the SYNC pin to the appropriate
    input/output state when this command is received.

    direction:  0 = master (pin in output), 1 = slave (pin is input)
    """
    self.h.write([self.SET_SYNC, direction])

  def Status(self):
    """
    This command retrieves the status of the device
    bit 0:  0 = sync slave, 1 = sync master
    bits 1-7: TBD
    """
    self.h.write([self.GET_STATUS])
    try:
      value = self.h.read(2, 100)
    except:
      print('Error in reading Status')
    return value[1]
    
  #################################
  #      Code Update  Commands    #
  #################################
    
  def PrepareDownload(self):
    """
    This command puts the device into code update mode.  The unlock
    code must be correct as a further safety device.  Call this once
    before sending code with WriteCode.  If not in code update mode,
    any WriteCode commands will be ignored.
    """
    self.h.write([self.PREPARE_DOWNLOAD, 0xad])

  def WriteCode(self, address, count, data):
    """
    This command writes to the program memory in the device.  This command is not accepted
    unless the device is in update mode.  This command will normally be used when downloading a
    new hex file, so it supports the memory ranges that may be found in the hex file.

    address: the start address to write.
         |---------------------------------------------------------------------|
         |       Range         |                    Usage                      |
         |---------------------------------------------------------------------|
         | 0x000000 - 0x007AFF | FLASH program memory                          |
         |---------------------------------------------------------------------|
         | 0x200000 - 0x200007 | ID memory (serial number is stored here)      |
         |---------------------------------------------------------------------|
         | 0x300000 - 0x30000F | CONFIG memory (processor configuration data)  |
         |---------------------------------------------------------------------|
         | 0xF00000 - 0xF03FFF | EEPROM memory                                 |
         |---------------------------------------------------------------------|

    FLASH program memory: The device must receive data in 64-byte
    segments that begin on a 64-byte boundary.  The data is sent in
    messages containing 32 bytes.  count must always equal 32.

    Other memory:  Any number of bytes up to te maximum may be sent.

    address: the start address for this portion of pgrogram memory.
    count:   the number of bytes of data (max 32)
    data:    the program data (max 32 bytes)
    """

    if (address >= 0x0 and address <= 0x007aff):
      count = 32
    if count > 32:
      raise ValueError('WriteCode: max count of 32 bytes.')
      return
    self.h.write([self.WRITE_CODE, (address & 0xff), (address>>8 & 0xff), (address>>16 & 0xff), count, data[0:count]])

  def WriteSerial(self, serial):
    """
    This command sends a new serial number to the device.  The serial number consists
    of 8 bytes, typically ASCII numberic or hexadecimal digits (i.e. "00000001").
    Note: The new serial number will be programmed but not used until hardware reset.
    """
    self.h.write(self.WRITE_SERIAL,serial[0:8])

  def volts(self, gain, volt):
    """
    Function to convert volts (or current) to inteter value
    """
    if gain == self.UP_10_00V:
      if volt >= 10.:
        value = 0xffff
      elif volt <= 0.0:
        value = 0
      else:
        value = volt*65536./10.
    elif gain == BP_10_00V:
      if volt >= 10.:
        value = 0xffff
      elif volt <- -10.:
        value = 0
      else:
        value = volt*32728./10 + 32768
    elif gain == I_0_20_mA:
      if volt >= 0.02: # 20 mA
        value = 0xffff
      elif volt <= 0.0:
        value = 0
      else:
        value = volt*249*65535./5.
    else:
      raise ValueError('volts: Unknwon gain level')
      return

    return int(value) & 0xffff

###########################################################

class usb_3101(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x009A              # MCC USB-3101

    usb_3100.__init__(self)

    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3102(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x009B              # MCC USB-3102
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3103(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x009C              # MCC USB-3103
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3104(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x009D              # MCC USB-3104
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3105(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x009E              # MCC USB-3105
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return
    
    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3106(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x009F              # MCC USB-3106
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3110(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x00A2              # MCC USB-3110
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3112(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x00A3              # MCC USB-3112
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

class usb_3114(usb_3100):
  def __init__(self, serial=None):
    self.productID = 0x00A4              # MCC USB-3114
    usb_3100.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      raise FileNotFoundError()
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)
