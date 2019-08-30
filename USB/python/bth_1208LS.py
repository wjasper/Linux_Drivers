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


class bth_1208LS(mccUSB):
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
  DIN                     = 0x01 # Read the current state of the DIO pins
  DOUT                    = 0x02 # Read/write DIO latch value
  # Analog Input Commands
  AIN                     =  0x10 # Read analog input channel
  AIN_SCAN_START          =  0x11 # Start analog input scan
  AIN_SCAN_STOP           =  0x12 # Stop analog input scan
  AIN_CONFIG              =  0x14 # Read/write analog input configuration
  AIN_SCAN_CLEAR_FIFO     =  0x15 # Clear the analog input scan FIFO
  # Analog Output Commands
  AOUT                    = 0x18  # Read/write analog output channel
  #Counter Commands
  COUNTER                 = 0x20  # Read/reset event counter
  # Memory Commands
  CAL_MEMORY              = 0x30  # Read/write calibration memory
  USER_MEMORY             = 0x31  # Read/write user memory
  SETTINGS_MEMORY         = 0x32  # Read/write settings memory
  # Miscellaneous Commands
  BLINK_LED               = 0x41  # Blink the LED
  RESET                   = 0x42  # Reset the device
  STATUS                  = 0x44  # Read device status
  INIT_RADIO              = 0x45  # Reset radio to default settings
  BLUETOOTH_PIN           = 0x46  # Read/write Bluetooth PIN
  BATTERY_VOLTAGE         = 0x47  # Read battery voltage
  SERIAL                  = 0x48  # Read/write serial number
  RADIO_FIRMWARE_VERSION  = 0x49  # Read radio firmware version
  DFU                     = 0x50  # Enter device firmware upgrade mode

  MAX_PACKET_SIZE         = 64    # max packet size for FS device
  NGAINS                  =  8    # max number of input gain levels (differential mode only)
  NCHAN_DE                =  4    # max number of A/D differential channels
  NCHAN_SE                =  8    # max number of A/D single-ended channels
  NCHAN_AOUT              =  2    # max number of D/A 12 bit 0-2.5V output channels
  
  # Aanalog Input
  SINGLE_ENDED =  0
  DIFFERENTIAL =  1

  # Analog Input Scan Options 
  IMMEDIATE_TRANSFER_MODE = 0x1
  BLOCK_TRANSFER_MODE     = 0x0
  DIFFERENTIAL_MODE       = 0x2
  SINGLE_ENDED_MODE       = 0x0
  NO_TRIGGER              = 0x0
  TRIG_EDGE_RISING        = 0x1 << 2
  TRIG_EDGE_FALLING       = 0x2 << 2
  TRIG_LEVEL_HIGH         = 0x3 << 2
  TRIG_LEVEL_LOW          = 0x4 << 2
  RETRIGGER_MODE          = 0x1 << 5
  STALL_ON_OVERRUN        = 0x0
  INHIBIT_STALL           = 0x1 << 7

  continuous_mode         = False

  # Ranges
  BP_20V   = 0x0   # +/- 20 V
  BP_10V   = 0x1   # +/- 10V
  BP_5V    = 0x2   # +/- 5V
  BP_4V    = 0x3   # +/- 4V
  BP_2_5V  = 0x4   # +/- 2.5V
  BP_2V    = 0x5   # +/- 2V
  BP_1_25V = 0x6   # +/- 1.25V
  BP_1V    = 0x7   # +/- 1V
  UP_2_5V  = 0x8   # 0-2.5V

  # Status bit values
  AIN_SCAN_RUNNING   = 0x1 << 1
  AIN_SCAN_OVERRUN   = 0x1 << 2
  NO_BATTERY         = 0x0
  FAST_CHARGE        = 0x1 << 8
  MAINTENANCE_CHARGE = 0x2 << 8
  FAULT_CHARGING     = 0x3 << 8
  DISABLE_CHARGING   = 0x4 << 8


  def __init__(self, serial=None):
    self.productID = 0x011a
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC BTH-1208LS not found")
      
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
    Builds a lookup table of single ended multiplexed calibration
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
      raise ValueError('Unknown range.')

    return volt

  #############################################
  #        Digital I/O Commands               #
  #############################################

  def DIn(self):
    """
    This command reads the current state of the DIO pins.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= unpack('B',self.udev.controlRead(request_type, self.DIN, wValue, wIndex, 1, timeout = 100))
    return value

  def DOut(self, value):
    """
    This command writes the DIO output latch values.  The factory
    power on default is all 1 (pins are floting).  Since the outputs
    are open drain, writing a 0 turns on the low side transistor and
    drives a 0 to the port pin and writing a 1 turns off the
    transistor and allows the pin to float.  The bits are mapped to
    the individual port pins.
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.DOUT
    wValue = value
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def DOutR(self):
    """
    This command reads the DIO output latch values.  The factory
    power on default is all 1 (pins are floting).  Since the outputs
    are open drain, writing a 0 turns on the low side transistor and
    drives a 0 to the port pin and writing a 1 turns off the
    transistor and allows the pin to float.  The bits are mapped to
    the individual port pins.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= unpack('B',self.udev.controlRead(request_type, self.DOUT, wValue, wIndex, 1, timeout = 100))
    return value

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
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = (channel | (mode << 0x8))
    wIndex = gain
    value ,= unpack('H',self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 2, timeout = 100))

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
    result in a bus stall if an AIn scan is currently running. The
    device will not generate an internal pacer faster than 50 kHz.

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
    SYNC; the maximum allowable input frequency is 50 kHz.  The data
    will be returned in packets utilizing a bulk in endpoint. The
    data will be in the format: 

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
    packets will be sent until the trigger is detected. In retrigger
    mode the trigger will be automatically rearmed and the scan will
    restart after retrig_count samples have been acquired. The count
    parameter specifies the total number of samples to acquire and
    should be >= retrig_count. Specifying 0 for count causes
    continuous retrigger scans. The data is still sent as a
    continuous stream during retrigger scan so the last data from a
    previous scan will not be transferred until the beginning of the
    next retrigger scan if it does not end on a packet boundary.

    In block transfer mode the data is sent in 64-byte packets as
    soon as enough data is available from the A/D. In immediate
    transfer mode the data is sent after each sample period,
    resulting in packets that are always 2 bytes (1 sample.) This
    mode should only be used for low pacer rates, typically under 100
    Hz, because it will overrun much easier.  

    Overruns are indicated by the device stalling the bulk in
    endpoint during the scan. The host may read the status to verify
    and must clear the stall condition before further scans can be
    performed.

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

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    if frequency > 50000.:
      frequency = 50000
    if frequency > 0:
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
    if options & self.DIFFERENTIAL_MODE:
      for i in range(self.NCHAN_DE):
        if (channels & (0x1 << i)) != 0x0:
          self.nChan += 1
    else:
      for i in range(self.NCHAN_SE):
        if (channels & (0x1 << i)) != 0x0:
          self.nChan += 1

    scanPacket = list(unpack('B'*14, pack('IIIBB', count, retrig_count, pacer_period, channels, options)))
    result = self.udev.controlWrite(request_type, self.AIN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

  def AInScanRead(self, nScan):
    nSamples = int(nScan * self.nChan)
    
    if self.options & self.IMMEDIATE_TRANSFER_MODE:
      for i in range(self.nSamples):
        try: 
          data.extend(unpack('H',self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2, 2000)))
        except:
          print('AInScanRead: error in bulk transfer in immmediate transfer mode.')
          return
    else:
      try:
        data =  unpack('H'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, int(2*nSamples), 2000))
      except:
        print('AInScanRead: error in bulk transfer!')
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
    This command reads or writes the analog input ranges used for
    AInScan in differential mode.  The command will result in a bus
    stall if an AInScan is currently running.  
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT);
    request = self.AIN_CONFIG
    wValue = 0x0
    wIndex = 0x0
    ranges = unpack('BBBB',self.udev.controlRead(request_type, request, wValue, wIndex, 4, timeout = 100))
    return ranges

  def AInConfigW(self, ranges):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
    request = self.AIN_CONFIG
    wValue = 0x0
    wIndex = 0x0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, ranges, timeout = 100)

  def AInScanClearFIFO(self):
    """
    The command clears the internal scan endoint FIFOs
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT);
    request = self.AIN_SCAN_CLEAR_FIFO
    wValue = 0x0
    wIndex = 0x0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)
    
    
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

  def SettingsMemoryR(self, address, count):
    """
    This command allows for reading and writing the nonvolite
    settings memory.  wLength specifies the number of bytes to read
    or write.  The settings memory is 1024 bytes (address 0 - 0x3FF).  If
    the settings are written they will be implemented immediately.

    This command allow for reading and writing the nonvolatile user
    memory. wLength specifies the number of bytes to read or write.
    The user memory is 256 bytes (address 0-0xFF)
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 512:
      raise ValueError('SettingsMemoryR: max bytes that can be read is 512.')
      return

    if address > 0x3ff:
      raise ValueError('SettingsMemoryR: address must be in the range 0 - 0x3ff.')
      return
    
    try:
      result = self.udev.controlRead(request_type, self.SETTINGS_MEMORY, wValue, wIndex, count, timeout = 100)
    except:
      print("SettingsMemoryR: controlRead error")

    return result


  def SettingsMemoyrW(self, address, data):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = address & 0xffff   # force to be 16 bits
    wIndex = 0

    if count > 512:
      raise ValueError('SettingsMemoryW: max bytes that can be read is 512.')
      return

    if address > 0x3ff:
      raise ValueError('SettingsMemoryW: address must be in the range 0 - 0x3ff.')
      return

    try:
      result = self.udev.controlWrite(request_type, self.SETTINGS_MEMORY, wValue, wIndex, data, timeout = 100)
    except:
      print("SettingsMemoryW: controlWrite error")

    
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
                 bits 3-7:   Reserved
                 bits 8-10:  Charger status:
                              0 = no battery
                              1 = fast charge
                              2 = maintenance charge
                              3 = fault (not charging)
                              4 = disabled
                 bits 11-15   Reserved
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    status ,= unpack('H',self.udev.controlRead(request_type, self.STATUS, wValue, wIndex, 2, timeout = 100))
    return status

  def InitRadio(self):
    """
    This command sets the radio module to factory default settings.
    This should be used during manufacturing to initialize the radio.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.INIT_RADIO
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def BluetoothPinR(self):
    """
    This command reads the Bluetooth PIN code.  The PIN code can be up
    to 16 ASCII characters.  The length of the code is specified in
    wLength.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT) 
    wValue = 0
    wIndex = 0
    pin = self.udev.controlRead(request_type, self.BLUETOOTH_PIN, wValue, wIndex, 16, timeout = 100)
    return  pin.decode()

  def BluetoothPinW(self, pin):
    """
    This command writes the Bluetooth PIN code.  The PIN code can be up
    to 16 ASCII characters.  The length of the code is specified in
    wLength.
    """
    if len(pin) > 16:
      raise ValueError('BluetoothPinW: length of pin greater than 16.')
      return
    
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT) 
    wValue = 0
    wIndex = 0
    self.udev.controlWrite(request_type, self.BLUETOOTH_PIN, wValue, wIndex, [pin], timeout = 100)

  def BatteryVoltage(self):
    """
    This command reads the voltage of the battery.  The voltage is
    returned in millivolts.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    voltage ,= unpack('H',self.udev.controlRead(request_type, self.BATTERY_VOLTAGE, wValue, wIndex, 2, timeout = 100))
    return voltage

  def RadioFirmwareVersion(self):
    """
    This command reads the radio firmware version.  The version
    consists of 16 bits in hexadecimal BCD notation.  i.e. version
    6.15 would be 0x0615.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    version ,= unpack('H',self.udev.controlRead(request_type, self.RADIO_FIRMWARE_VERSION, wValue, wIndex, 2, timeout = 100))
    return version
    
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
    
