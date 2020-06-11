#! /usr/bin/python3
#
# Copyright (c) 2020 Warren J. Jasper <wjasper@ncsu.edu>
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

class usb_1608FS_Plus(mccUSB):
  # Gain Ranges
  BP_10_00V   = 0x0         # Differential +/- 10.0 V
  BP_5_00V    = 0x1         # Differential +/- 5.00 V
  BP_2_50V    = 0x2         # Differential +/- 2.50 V
  BP_2_00V    = 0x3         # Differential +/- 2.00 V
  BP_1_25V    = 0x4         # Differential +/- 1.25 V
  BP_1_00V    = 0x5         # Differential +/- 1.00 V
  BP_0_625V   = 0x6         # Differential +/- 0.625 V
  BP_0_3125V  = 0x7         # Differential +/- 0.3125 V

  # Status Bits
  AIN_SCAN_RUNNING = 0x2    # 1 = AIn scan running
  AIN_SCAN_OVERRUN = 0x4    # 1 = AIn scan overrun
  
  NCHAN              = 8    # max number of A/D channels in the device
  NGAIN              = 8    # max number of gain levels (0-7)

  # Option values for AInScan
  IMMEDIATE_TRANSFER_MODE = 0x1
  BLOCK_TRANSFER_MODE     = 0x0
  INTERNAL_PACER_ON       = 0x2
  INTERNAL_PACER_OFF      = 0x0
  AIN_NO_TRIGGER     = 0x0
  TRIG_EDGE_RISING   = (0x1 << 2)
  TRIG_EDGE_FALLING  = (0x2 << 2)
  TRIG_LEVEL_HIGH    = (0x3 << 2)
  TRIG_LEVEL_LOW     = (0x4 << 2)
  DEBUG_MODE         = (0x1 << 5)
  INHIBIT_STALL      = (0x1 << 7)

  # Commands and Codes for USB1608FS_Plus
  # Digital I/O Commands
  DTRISTATE           = 0x00  # Read/write digital port tristate register
  DPORT               = 0x01  # Read digital port pins / write output latch register
  DLATCH              = 0x02  # Read/write digital port output latch register

  # Analog Input Commands
  AIN                 = 0x10  # Read analog input channel
  AIN_SCAN_START      = 0x11  # Start analog input scan
  AIN_SCAN_STOP       = 0x12  # Stop analog input scan
  AIN_SCAN_CONFIG     = 0x14  # Read/Write analog input configuration
  AIN_SCAN_CLEAR_FIFO = 0x15  # Clear the analog input scan FIFO

  # Counter Commands
  COUNTER             = 0x20  # Read/reset event counter

  # Memory Commands
  CAL_MEMORY          = 0x30  # Read/write calibration memory
  USER_MEMORY         = 0x31  # Read/write user memory
  MBD_MEMORY          = 0x32  # Read/write MBD memory

  # Miscellaneous Commands
  BLINK_LED           = 0x41  # Blink the LED
  RESET               = 0x42  # Reset the device
  STATUS              = 0x44  # Read device status
  SERIAL              = 0x48  # Read/write serial number
  DFU                 = 0x50  # Ender device firmware upgrade mode

  # MBD
  MDB_COMMAND         = 0x80  # Text-based MBD command/response
  MDB_RAW             = 0x81  # Raw MBD response

  HS_DELAY = 2000

  def __init__(self, serial=None):
    print("initializing\n")
    self.productID = 0x00ea               # MCC USB-1608FS-Plus
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1608FS-Plus not found")
    if sys.platform.startswith('linux'):
      if self.udev.kernelDriverActive(0):
        self.udev.detachKernelDriver(0)
        self.udev.resetDevice()

    # claim all the needed interfaces for AInScan
    self.udev.claimInterface(0)

    # need to get wMaxPacketSize
    self.wMaxPacketSize = self.getMaxPacketSize()

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #   self.table_AIn[channel][gain]  0 <= chan < 8  0 <= gain < 8
    self.table_AIn = [[table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()], \
                      [table(), table(), table(), table(), table(), table(), table(), table()]]

    address = 0x0
    for gain in range(self.NGAIN):
      for chan in range(self.NCHAN):
        self.table_AIn[chan][gain].slope, = unpack('f', self.CalMemoryR(address, 4))
        address += 4
        self.table_AIn[chan][gain].intercept, = unpack('f', self.CalMemoryR(address, 4))
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

  #############################################
  #        Digital I/O Commands               #
  #############################################

  # Read/Write digital port tristate register
  def DTristateR(self):
    """
    This command reads the digital port tristate register.  The
    tristate register determines if the latch register value is driven
    onto the port pin.  A '1' in the tristate register makes the
    corresponding pin an input, a '0' makes it an output.  The power
    on default is all input.
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= self.udev.controlRead(request_type, self.DTRISTATE, wValue, wIndex, 1, self.HS_DELAY)
    return value

  def DTristateW(self, value):
    """
    This command writes the digital port tristate register.  The
    tristate register determines if the latch register value is driven
    onto the port pin.  A '1' in the tristate register makes the
    corresponding pin an input, a '0' makes it an output.
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.DTRISTATE
    wValue = value & 0xff
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def DPort(self):
    """
    This command reads the current state of the digital pins.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= self.udev.controlRead(request_type, self.DPORT, wValue, wIndex, 1, self.HS_DELAY)
    return value

  def DPortW(self, value):
    """
    This command writes to the latch register.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = value & 0xff
    wIndex = 0
    self.udev.controlWrite(request_type, self.DPORT,  wValue, wIndex, [0x0], self.HS_DELAY)
    return 

  def DLatchR(self):
    """
    This command reads the digital port latch register.  The power on default is all 0.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= self.udev.controlRead(request_type, self.DLATCH, wValue, wIndex, 1, self.HS_DELAY)
    return value

  def DLatchW(self, value):
    """
    This command writes the digital port latch register.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = value & 0xff
    wIndex = 0
    self.udev.controlWrite(request_type, self.DLATCH, wValue, wIndex, [0x0], self.HS_DELAY)
    

  #############################################
  #        Analog Input Commands              #
  #############################################
  def AIn(self, channel, gain):
    """
    This command reads the value of an analog input channel.  This
    command will result in a bus stall if an AInScan is currently
    running.  

     channel: the channel to read (0-7)
     gain:    the input gain for the channel (0-7
     value:   16 bits of data, right justified.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = channel
    wIndex = gain
    value ,= unpack('H',self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 2, timeout = 100))
    data = round(float(value)*self.table_AIn[channel][gain].slope + self.table_AIn[channel][gain].intercept)
    if (data >= 65536):
      value = 65535
    elif (data < 0):
      value = 0
    else:
      value = data
    return value

  def AInScanStart(self, count, frequency, channels, options):
    """
    This command starts an analog input scan. This command will
    result in a bus stall if an AIn scan is currently running. The
    device will not generate an internal pacer faster than 100 kHz;

    The pacer rate is set by an internal 32-bit timer running at a
    base rate of 40 MHz. The timer is controlled by pacer_period. This
    value is the period of the the ADC conversions.  A pulse will be
    output at the SYNC pin at every pacer_period interval if SYNC is
    configured as an output. The equation for calculating pacer_period
    is:
   
          pacer_period = [40 MHz / (sample frequency)] - 1 

    If pacer_period is set to 0 the device does not generate an A/D
    clock. It uses the SYNC pin as an input and the user must
    provide the pacer source. The A/Ds acquire data on every rising
    edge of SYNC; the maximum allowable input frequency is 100 kHz.

    The data will returned in packets utilizing a bulk endpoint  The data will be in the format:

     lowchannel sample 0 : lowchannel + 1 sample 0 : … : hichannel sample 0 
     lowchannel sample 1 : lowchannel + 1 sample 1 : … : hichannel sample 1
      … 
     lowchannel sample n : lowchannel + 1 sample n : … : hichannel sample n 

    The scan will not begin until this command is sent and any
    trigger conditions are met. Data will be sent until reaching the
    specified count or an AInScanStop command is sent.

    The external trigger may be used to start the scan. If enabled,
    the device will wait until the appropriate trigger condition is
    detected then begin sampling data at the specified rate. No
    packets will be sent until the trigger is detected. 

    In block transfer mode the data is sent in 64-byte packets as soon
    as enough data is available from the A/D. In immediate transfer
    mode the data is sent after each scan, resulting in packets that
    are 1-8 samples (2-16 bytes) long depending on the number of
    channels in the scan.  This mode should only be used for low pacer
    rates, typically under 100 Hz, because overruns will occur if the
    rate is too high.

    There is a 32,768 sample FIFO and scan under 32kS can be performed at up to 100 kHz * 8
    channels without overrun.

    Overruns are indicated by the device stalling the bulk in
    endpoint during the scan. The host may read the status to verify
    and must clear the stall condition before further scans can be
    performed.

     count:         the total number of scans to perform, 0 for continuous scan
     pacer_period:  the A/D timer period (0 for external clock)
     channels:      bit field that selects the channels in the scan
        ---------------------------------------------------------
        | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
        ---------------------------------------------------------
        | Ch7  |  Ch6 | Ch5  | Ch4  | Ch3  | Ch2  | Ch1  | Ch 0 |
        ---------------------------------------------------------

     options:  bit field that controls scan options
       bit 0: 1 = immediate transfer mode, 0 = block transfer mode
       bit 1: 1 = output internal pacer on SYNC, 0 = do not output internal pacer (ignored wien using external clock for pacing)
       bits 2-4: Trigger setting
                   0: no trigger
                   1: Edge/rising
                   2: Level/falling
                   3: Level/high
                   4: Level/low
       bit 5:  1 = debug mode (data returned is incrementing counter), 0 = normal A/D data
       bit 6:  reserved
       bit 7:  0 = stall bulk pipe on overrun, 1 = inhibit bulk pipe stall

    """

    if frequency > 100000.:
      frequency = 100000
    if frequency > 0:
      pacer_period = round((40.E6 / frequency) - 1)
    else:
      pacer_period = 0

    self.frequency = frequency
    self.options = options & 0xff
    if count == 0:
      self.continuous_mode = True
    else:
      self.continuous_mode = False

    self.nChan = 0
    for i in range(self.NCHAN):
      if (channels & (0x1 << i)) != 0x0:
        self.nChan += 1

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    scanPacket = pack('IIBB', count, pacer_period, channels, options)
    result = self.udev.controlWrite(request_type, self.AIN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

  def AInScanRead(self, nScan):
    nSamples = int(nScan * self.nChan)
    if self.options & self.IMMEDIATE_TRANSFER_MODE:
      for i in range(self.nSamples):
        try:
          timeout = int(1000*self.nChan/self.frequency + 100)
          data.extend(unpack('H',self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2*self.nChan, timeout)))
        except:
          print('AInScanRead: error in bulk transfer in immmediate transfer mode.')
          return
    else:
      try:
        timeout = int(1000*self.nChan*nScan/self.frequency + 1000)
        data =  unpack('H'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, int(2*nSamples), timeout))
      except:
        print('AInScanRead: error in bulk transfer!', len(data), nSamples)
        return

    status = self.Status()
    try:
      if status & self.AIN_SCAN_OVERRUN:
        raise OverrunERROR
    except:
      print('AInScanRead: Overrun Error')
      return
    
    if self.continuous_mode:
      return list(data)

    # if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
    if ((int(nSamples*2) % self.wMaxPacketSize) == 0 and  not(status & self.AIN_SCAN_RUNNING)):
      data2 = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2, 100)
    
    self.AInScanStop()
    self.AInScanClearFIFO()
    return list(data)

  def AInScanStop(self):
    """
    This command stops the analog input scan (if running).
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_STOP
    wValue = 0x0
    wIndex = 0x0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def AInConfigR(self):
    """
    This command reads or writes the analog input configuration.  This
    command will result in a bus stall if an AIn scan is currently
    running.

    ranges  8 bytes channel ranges, each byte corrsponds to an input channel
            Range #     Input range
             0            +/- 10 V
             1            +/- 5 V
             2            +/- 2.5 V
             3            +/- 2 V
             4            +/- 1.25 V
             5            +/- 1 V
             6            +/- 0.625 V
             7            +/- 0.3125 V
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CONFIG
    wValue = 0x0
    wIndex = 0x0
    ranges = unpack('BBBBBBBB',self.udev.controlRead(request_type, request, wValue, wIndex, 8, timeout = 100))
    return ranges

  def AInConfigW(self, ranges):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CONFIG
    wValue = 0x0
    wIndex = 0x0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, ranges, timeout = 100)

  def AInScanClearFIFO(self):
    """
    This command clears the internal scan endpoint FIFOs
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CLEAR_FIFO
    wValue = 0x0
    wIndex = 0x0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)
    try:
      data = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 64, 100)
    except:
      pass
    
  #############################################
  #           Counter Commands                #
  #############################################

  def Counter(self):
    """
    The command reads the event counter.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    count ,= unpack('I',self.udev.controlRead(request_type, self.COUNTER, wValue, wIndex, 4, timeout = 100))
    return count
    
  def ResetCounter(self):
    """
    This command resets the event counter to 0.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)
    
  #############################################
  #        Memory Commands                    #
  #############################################

  def CalMemoryR(self, address, count):
    """
    This command allows for reading the nonvolatile calibration
    memory.  The cal memory is 768 bytes (address 0-0x2FF).  The cal
    memory is write protected and must be unlocked in order to write
    the memory.  The unlock procedure is to write the unlock code
    0xAA55 to address 0x300.  Writes to the entire memory range is
    then possible.  Write any other value to address 0x300 to lock
    the memory after writing.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)

    if count > 768:
      raise ValueError('CalMemoryR: max bytes that can be read is 768.')
      return

    if address > 0x2ff:
      raise ValueError('CalMemoryR: address must be in the range 0 - 0x2FF.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0
    try:
      result = self.udev.controlRead(request_type, self.CAL_MEMORY, wValue, wIndex, count, timeout = 100)
    except:
      print("CalMemoryR: controlRead error")
    return result

  def UserMemoryR(self, address, count):
    """
    This command allow for reading and writing the nonvolatile user
    memory. wLength specifies the number of bytes to read or write.
    The user memory is 256 bytes (address 0-0xFF)
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 256:
      raise ValueError('UserMemoryR: max bytes that can be read is 256.')
      return

    if address > 0xff:
      raise ValueError('UserMemoryR: address must be in the range 0 - 0xff.')
      return
    
    try:
      result = self.udev.controlRead(request_type, self.USER_MEMORY, wValue, wIndex, count, timeout = 100)
    except:
      print("UserMemoryR: controlRead error")

    return result

  def UserMemoyrW(self, address, data):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 256:
      raise ValueError('UserMemoryW: max bytes that can be read is 256.')
      return

    if address > 0xff:
      raise ValueError('UserMemoryW: address must be in the range 0 - 0xff.')
      return

    try:
      result = self.udev.controlWrite(request_type, self.USER_MEMORY, wValue, wIndex, data, timeout = 100)
    except:
      print("UserMemoryW: controlWrite error")

  def MBDMemoryR(self, address, count):
    """
    This command allows for reading and writing the nonvolite
    MBD memory.  wLength specifies the number of bytes to read
    or write.  The MBD memory is 1024 bytes (address 0 - 0x3FF).
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 1023:
      raise ValueError('MBDMemoryR: max bytes that can be read is 512.')
      return

    if address > 0x3ff:
      raise ValueError('MBDMemoryR: address must be in the range 0 - 0x3ff.')
      return
    
    try:
      result = self.udev.controlRead(request_type, self.SETTINGS_MEMORY, wValue, wIndex, count, timeout = 100)
    except:
      print("MBDMemoryR: controlRead error")

    return result

  def MBDMemoyrW(self, address, data):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 1023:
      raise ValueError('MBDMemoryW: max bytes that can be read is 1023.')
      return

    if address > 0x3ff:
      raise ValueError('MBDMemoryW: address must be in the range 0 - 0x3ff.')
      return

    try:
      result = self.udev.controlWrite(request_type, self.SETTINGS_MEMORY, wValue, wIndex, data, timeout = 100)
    except:
      print("MBDMemoryW: controlWrite error")

  #############################################
  #        Miscellaneous Commands             #
  #############################################
  def BlinkLED(self, count):
    """
    This command will blink the device LED
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.BLINK_LED
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [count], timeout = 100)
    
  def Reset(self):
    """
    This command resets the device
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.RESET
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def Status(self):
    """
    This command retrieves the status of the device.  Writing the command will clear
    the error indicators.

    status:      bit 0:      Reserved
                 bit 1:      1 = AIn scan running
                 bit 2:      1 = AIn scan overrun
                 bits 3-15:   Reserved
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    status ,= unpack('H',self.udev.controlRead(request_type, self.STATUS, wValue, wIndex, 2, timeout = 100))
    return status

  def GetSerialNumber(self):
    """
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal
    digits (i.e. "00000001"). The new serial number will be wtored but
    not used until the device is reset.
    """
    
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0x0
    wIndex = 0x0
    value = self.udev.controlRead(request_type, self.SERIAL, wValue, wIndex, 8, self.HS_DELAY)
    return value.decode()

  def WriteSerialNumber(self, serial):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SERIAL
    wValue = 0x0
    wIndex = 0x0
    barray = bytearray(8)
    for i in range(8):
      barray[i] = ord(serial[i])
    self.udev.controlWrite(request_type, request, wValue, wIndex, barray, self.HS_DELAY)

  def DFU(self):
    """
    This command places the device in firmware upgrade mode by erasing
    a portion of the program memory.  The next time the device is
    reset, it will enumerate in the bootloader and is unusable as a
    DAQ device until new firmware is loaded.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0xADAD
    wIndex = 0
    self.udev.controlWrite(request_type, self.DFU, wValue, wIndex, [0x0], timeout = 100)

  def MBDCommandR(self, nBytes):
    """
    This command is the interface for text-based MBD commands and
    responses.  the length of the string must be passed in wLength for
    an OUT transfer.
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    
    try:
      result = self.udev.controlRead(request_type, self.MBD_COMMAND, wValue, wIndex, nBytes, timeout = 100)
    except:
      print("MBDCommandR: controlRead error")

    return result

  def MBDCommandW(self, command):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    try:
      result = self.udev.controlWrite(request_type, self.MBD_COMMAND, wValue, wIndex, command, timeout = 100)
    except:
      print("MBDCommandW: controlWrite error")

  def MBDRaw(self, nBytes):
    """
    This command is the interface for binary responses to certain MBD commands
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    
    try:
      result = self.udev.controlRead(request_type, self.MBD_RAW, wValue, wIndex, nBytes, timeout = 100)
    except:
      print("MBDRaw: controlRead error")

    return result
    

  def printStatus(self):
    status = self.Status()
    print('**** USB-1608FS-Plus Status ****')
    if status & self.AIN_SCAN_RUNNING:
      print('    Analog Input scan running')
    if status & self.AIN_SCAN_OVERRUN:
      print('    Analong Input scan overrun')

  def volts(self, gain, value):
    # converts unsigned short value to volts
    if gain == self.BP_10_00V:
      volt = (value - 0x8000) * 10. / 32768
    elif gain == self.BP_5_00V:
      volt = (value - 0x8000) * 5. / 32768
    elif gain == self.BP_2_50V:
      volt = (value - 0x8000) * 2.5 / 32768
    elif gain == self.BP_2_00V:
      volt = (value - 0x8000) * 2. / 32768
    elif gain == self.BP_1_25V:
      volt = (value - 0x8000) * 1.25 / 32768
    elif gain == self.BP_1_00V:
      volt = (value - 0x8000) * 1. / 32768
    elif gain == self.BP_0_625V:
      volt = (value - 0x8000) * 0.625 / 32768
    elif gain == self.BP_0_3125V:
      volt = (value - 0x8000) * 0.325 / 32768
    else:
      raise ValueError('volts: Unknown range.')

    return volt
