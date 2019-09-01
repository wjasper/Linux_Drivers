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
  USB_201_PID = 0x0113
  USB_202_PID = 0x12b
  USB_204_PID = 0x0114
  USB_205_PID = 0x12c
    
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
  MBD_RAW                 = 0x81  # Raw MBD response

  MAX_PACKET_SIZE         = 64    # max packet size for FS device
  NCHAN                   = 8     # max number of A/D channels in the device

  # Status bit values
  AIN_SCAN_RUNNING        = 0x1 << 1
  AIN_SCAN_OVERRUN        = 0x1 << 2

  # Analog Input Scan Options
  IMMEDIATE_TRANSFER_MODE = 0x1
  BLOCK_TRANSFER_MODE     = 0x0
  STALL_ON_OVERRUN        = 0x0
  CONTINUOUS              = 0x1 << 6
  INHIBIT_STALL           = 0x1 << 7
  NO_TRIGGER              = 0x0
  TRIGGER                 = 0x1
  EDGE_RISING             = 0x0
  EDGE_FALLING            = 0x1
  LEVEL_HIGH              = 0x2
  LEVEL_LOW               = 0x3
  CHAN0                   = 0x1
  CHAN1                   = 0x2
  CHAN2                   = 0x4
  CHAN3                   = 0x8
  CHAN4                   = 0x10
  CHAN5                   = 0x20
  CHAN6                   = 0x40
  CHAN7                   = 0x80

  HS_DELAY = 2000

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
    self.table_AIn  = [table(), table(), table(), table(), table(), table(), table(), table()]
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
    hour += 1              # seems off by one hour
    
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

  # Read/Write digital port tristate register
  def DTristateR(self):
    """
    This command reads the digital port tristate register.  The
    tristate register determines if the latch register value is driven
    onto the port pin.  A '1' in the tristate register makes the
    corresponding pin an input, a '0' makes it an output.
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
    This command reads the current state of the digital pins or writes to the latch register.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= self.udev.controlRead(request_type, self.DPORT, wValue, wIndex, 1, self.HS_DELAY)
    return value

  def DPortW(self):
    """
    This command reads the current state of the digital pins or writes to the latch register.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    wValue = value & 0xff
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)
    return 

  def DLatchR(self):
    """
    This command reads the digital port latch register
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= self.udev.controlRead(request_type, self.DLATCH, wValue, wIndex, 1, self.HS_DELAY)
    return value

  def DLatchW(self, value):
    """
    This command writes the digital port latch register
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.DLATCH
    wValue = value & 0xff
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)
    

  #############################################
  #        Analog Input Commands              #
  #############################################
  def AIn(self, channel):
    """
    This command reads the value of an analog input channel.  This
    command will result in a bus stall if an AInScan is currently
    running.  

     channel: the channel to read (0-7)
     value:   12 bits of data, right justified.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = channel
    wIndex = 0x0
    value ,= unpack('H',self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 2, timeout = 100))
    data = round(float(value)*self.table_AIn[channel].slope + self.table_AIn[channel].intercept)
    if (data >= 65536):
      value = 65535
    elif (data < 0):
      value = 0
    else:
      value = data
    return value

  def AInScanStart(self, count, frequency, channels, options, trigger_source, trigger_mode):
    """This command starts an analog input scan. This command will
    result in a bus stall if an AIn scan is currently running. The
    device will not generate an internal pacer faster than 100 kHz;
    the USB-205 will not generate an internal pacer faster than 500 kHz

    The ADC is paced such that the pacer controls the ADC conversions.
    The pacer rate is set by an internal 32-bit timer running at a
    base rate of 70 MHz. The timer is controlled by pacer_period. This
    value is the period of the the ADC conversions.  A pulse will be
    output at the PACER_OUT pin at every pacer_period interval if SYNC is
    configured as an output. The equation for calculating pacer_period
    is:
   
          pacer_period = [70 MHz / (sample frequency)] - 1 

    If pacer_period is set to 0 the device does not generate an A/D
    clock. It uses the PACER_IN pin as an input and the user must
    provide the pacer source. Each rising edge of PACER_IN starts a
    conversion. The data will be returned in packets utilizing a bulk
    in endpoint.  The data will be in the format:

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
    are 1-8 samples (2-16 bytes) long.  This mode should only be used
    for low pacer rates, typically under 100 Hz, because overruns will
    occur if the rate is too high.

    Overruns are indicated by the device stalling the bulk in
    endpoint during the scan. The host may read the status to verify
    and must clear the stall condition before further scans can be
    performed.

     count:         the total number of scans to perform, 0 for continuous scan
     pacer_period:  the A/D timer period (0 for external clock)
     channels:      bit field that selects the channels in the scan, upper 4 bits ignored in differential mode. 
        ---------------------------------------------------------
        | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
        ---------------------------------------------------------
        | Ch7  |  Ch6 | Ch5  | Ch4  | Ch3  | Ch2  | Ch1  | Ch 0 |
        ---------------------------------------------------------

     options:  bit field that controls scan options
       bit 0: 1 = immediate transfer mode, 0 = block transfer mode
       bits 1-6:  Reserved
       bit 7:  0 = stall on overrun, 1 = inhibit stall

     trigger_source: the trigger source
                     0 = no trigger,  1 = digital trigger
     trigger_mode:  the trigger mode
                    0: Edge/rising
                    1: Edge/falling
                    2: Level/high
                    3: Level/low
    """

    if frequency > 500000. and self.productID == USB_205_PID:
      frequency = 500000.
    elif frequency > 100000.:
      frequency = 100000
    if frequency > 0:
      pacer_period = round((70.E6 / frequency) - 1)
    else:
      pacer_period = 0

    self.frequency = frequency
    self.options = options & 0xff
    self.trigger_source = trigger_source & 0xff
    self.trigger_mode = trigger_mode & 0xff
    if count == 0:
      self.continuous_mode = True
    else:
      self.continuous_mode = False

    self.nChan = 0
    for i in range(self.NCHAN):
      if (channels & (0x1 << i)) != 0x0:
        self.nChan += 1

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    scanPacket = pack('IIBBBB', count, pacer_period, channels, options, trigger_source, trigger_mode)
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
        print('AInScanRead: error in bulek transfer!', len(data), nSamples)
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

  def AInScanClearFIFO(self):
    """
    The command clears the internal scan endoint FIFOs
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
    request = self.AIN_SCAN_CLEAR_FIFO
    wValue = 0x0
    wIndex = 0x0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)
    
  def BulkFlush(self, count=1):
    """
    This command will send a specified number of empty BULK IN packets
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_BULK_FLUSH
    wValue = count
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex,[0x0], self.HS_DELAY)
    
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
      print("MemoryR: controlRead error")
    return result

  def CalMemoyrW(self, address, count, data):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 768:
      raise ValueError('UserMemoryW: max bytes that can be read is 256.')
      return

    if address > 0xff:
      raise ValueError('UserMemoryW: address must be in the range 0 - 0xff.')
      return

    try:
      result = self.udev.controlWrite(request_type, self.USER_MEMORY, wValue, wIndex, data, timeout = 100)
    except:
      print("UserMemoryW: controlWrite error")

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

  def volts(self, value):
    """
    Convert 12 bit raw value to volts.
    All values single ended +/- 10V.
    """
    volt = ((value - 2048)*10.)/2048.
    return volt

####################################################################################

class usb_201(usb_20x):

  def __init__(self, serial=None):
    self.productID = self.USB_201_PID    # usb-201
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-201 not found")
      return
    usb_20x.__init__(self)

class usb_202(usb_20x):

  def __init__(self, serial=None):
    self.productID = self.USB_202_PID    # usb-202
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-202 not found")
      return
    usb_20x.__init__(self)

  #############################################
  #        Analog Output Commands             #
  #############################################

  def AOut(self, channel, value):
    """
    This command writes the values of an analog output channel

      channel: the channel to write (0-1)
      value:   the value to write (0-4095)
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AOUT
    wValue = value & 0xffff
    wIndex = channel & 0xff

    if channel > 2 or channel < 0:
      raise ValueError('AOut: channel must be 0 or 1.')
      return
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def AOutR(self, channel=0):
    """
    This command reads the value of  both  analog output channels
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AOUT
    wValue = 0
    wIndex = 0
    channels = unpack('HH',self.udev.controlRead(request_type, request, wValue, wIndex, 4, timeout = 100))
    if channel == 0:
      return channels[0]
    else:
      return channels[1]

class usb_204(usb_20x):

  def __init__(self, serial=None):
    self.productID = self.USB_204_PID    # usb-204
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-204 not found")
      return
    usb_20x.__init__(self)

class usb_205(usb_20x):

  def __init__(self, serial=None):
    self.productID = self.USB_205_PID    # usb-205
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-205 not found")
      return
    usb_20x.__init__(self)

  
  #############################################
  #        Analog Output Commands             #
  #############################################

  def AOut(self, channel, value):
    """
    This command writes the values of an analog output channel

      channel: the channel to write (0-1)
      value:   the value to write (0-4095)
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AOUT
    wValue = value & 0xffff
    wIndex = channel & 0xff

    if channel > 2 or channel < 0:
      raise ValueError('AOut: channel must be 0 or 1.')
      return
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def AOutR(self, channel=0):
    """
    This command reads the value of  both  analog output channels
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AOUT
    wValue = 0
    wIndex = 0
    channels = unpack('HH',self.udev.controlRead(request_type, request, wValue, wIndex, 4, timeout = 100))
    if channel == 0:
      return channels[0]
    else:
      return channels[1]
    
