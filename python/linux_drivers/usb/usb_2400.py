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
from datetime import datetime
from thermocouple import *
from mccUSB import *


# Base class for AIn scan queue
class queue:
  def __init__(self):
    self.channel = 0x0  # the analog input channel
    self.mode    = 0x0  # the input mode (see AIn)
    self.gain    = 0x0  # the input range (see AIn)
    self.rate    = 0x0  # the A/D data rate (see AIn)  

class usb_2400(mccUSB):
  # Gain Ranges
  BP_10_00V  = 0x1         # +/- 10.0 V
  BP_5_00V   = 0x2         # +/- 5.00 V
  BP_2_50V   = 0x3         # +/- 2.50 V
  BP_1_25V   = 0x4         # +/- 1.25 V
  BP_0_625V  = 0x5         # +/- 0.625 V
  BP_0_312V  = 0x6         # +/- 0.3125 V
  BP_0_156V  = 0x7         # +/- 0.15625 V
  BP_0_078V  = 0x8         # +/- 0.078125 V (Voltage and Thermocouple)

  # Modes
  DIFFERENTIAL = 0x0        # Voltage differential
  SINGLE_ENDED = 0xb        # Voltage Single Ended
  SE_HIGH      = 0x1        # Voltage Single Ended high channel
  SE_LOW       = 0x2        # Voltage Single Ended low channel
  DAC_READBACK = 0x3        # Voltage - D/A readback
  THERMOCOUPLE = 0x4        # Thermocouple
  AIN_OFFSET   = 0x5        # AIn Offset Calibration
  AIN_GAIN     = 0x6        # AIn Gain Calibration
  TC_OFFSET    = 0x7        # TC Offset Calibration
  TC_GAIN_POS  = 0x8        # TC Gain Calibration Positive
  TC_GAIN_NEG  = 0x9        # TC Gain Calibration Negative
  TC_BURNOUT   = 0xa        # Thermocouple without burnout detect
  CONTINUOUS   = 0x1        # Continuous Mode

  # Rate
  HZ30000  =  0    # 30,000 S/s
  HZ15000  =  1    # 15,000 S/s
  HZ7500   =  2    #  7,500 S/s
  HZ3750   =  3    #  3,750 S/s
  HZ2000   =  4    #  2,000 S/s
  HZ1000   =  5    #  1,000 S/s
  HZ500    =  6    #    500 S/s
  HZ100    =  7    #    100 S/s
  HZ60     =  8    #     60 S/s
  HZ50     =  9    #     50 S/s
  HZ30     =  10   #     30 S/s
  HZ25     =  11   #     25 S/s
  HZ15     =  12   #     15 S/s
  HZ10     =  13   #     10 S/s
  HZ5      =  14   #      5 S/s
  HZ2_5    =  15   #    2.5 S/s

  COUNTER0       = 0x0      # Counter 0
  COUNTER1       = 0x1      # Counter 1
  nCJCGRAD       = 8        # max number of CJC Gradient array elements
  NGAINS         = 9        # max number of gain levels (analog input)
  MAX_QUEUE_SIZE = 64       # max number of entries in the AIN scan queue

  INPUT_SCAN_RUNNING   = 0x1
  INPUT_FIFO_FULL      = 0x2
  INPUT_PACER_SHORT    = 0x4
  OUTPUT_SCAN_RUNNING  = 0x1
  OUTPUT_SCAN_UNDERRUN = 0x2
  
  # Commands and Codes for USB-2408 reports
  # Digital I/O Commands
  DIN              = 0x00   # Read digital port value
  DOUT             = 0x01   # Read/Write digital port drive register

  # Analog Input Commands
  AIN              = 0x10   # Read analog input channel
  AIN_SCAN_START   = 0x11   # Start analog input scan
  AIN_SCAN_STOP    = 0x12   # Stop input scan
  AIN_SCAN_STATUS  = 0x13   # Read analog input scan status
  AIN_SCAN_QUEUE   = 0x14   # Read/Write analog input channel gain queue
  AIN_SETTING      = 0x16   # Front end setting test

  # Analog output Commands (USB-2408-2AO oly)
  AOUT             = 0x18   # Write analog output channel
  AOUT_SCAN_START  = 0x19   # Start analog output scan
  AOUT_SCAN_STOP   = 0x1A   # Stop analog output scan
  AOUT_SCAN_STATUS = 0x1B   # Read analog output scan status

  # Counter Commands
  COUNTER          = 0x20   # Read/Reset event counter

  # Memory Commands
  MEMORY           = 0x30   # Read/Write Memory (EEPROM)

  # Miscellaneous Commands
  RESET            = 0x40   # Reset USB interface
  BLINK_LED        = 0x41   # Causes LED to blink
  CJC_SENSOR       = 0x42   # Read CJC sensor values
  CAL_CONFIG       = 0x43   # Configure calibration source
  GET_STATUS       = 0x44   # Read device status
  AD_CAL           = 0x45   # Perform A/D self calibration
  TC_CAL           = 0x46   # Measure TC calibration source
  SERIAL           = 0x48   # Write USB serial number
  VERSION          = 0x49   # Read micro firmware versions
  
  # FIRMWARE Update Commands
  UPDATE_MODE      = 0x50   # Put device into firmware update mode
  UPDATE_ADDR      = 0x51   # Read/Write the firmware address
  UPDATE_DATA      = 0x52   # Read/Write the firmware data
  UPDATE_VERSION   = 0x53   # Read the firmware code version
  UPDATE_ERASE     = 0x54   # Erase the firmware
  MBD_COMMAND      = 0x80   # Text-based MBD command
  MBD_RAW          = 0x81   # Raw MBD response

  productID        = 0
  NCHAN            = 16

  def __init__(self):
    # need to get wMaxPacketSize
    self.wMaxPacketSize = self.getMaxPacketSize()

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 8-byte double.
    #
    #   calibrated code = code*slope + intercept
    #   self.Cal[gain]    0 <= gain <= 8  gain = 9 for Thermocouple

    self.Cal = [table(), table(), table(), table(), table(), table(), table(), table(), table(), table()]

    address = 0x00B0
    for i in range(1,10):
       self.Cal[i].slope ,= unpack('d', self.MemoryR(address, 8))
       address += 8
       self.Cal[i].intercept ,= unpack('d', self.MemoryR(address, 8))
       address += 8

    self.Cal[0].slope = self.Cal[1].slope
    self.Cal[0].intercept = self.Cal[1].intercept

    self.Queue = [0x1, queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue(), \
                       queue(), queue(), queue(), queue(), queue(), queue(), queue(), queue() ]

    
  #################################
  #     Digital I/O  Commands     #
  #################################

  def DIn(self, port=0):
    '''
    This command reads the current state of the digital port pins.
    port: the port to read (there is only one)  0: onboard (pins 0-7)
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    data = self.udev.controlRead(request_type, self.DIN, port, wIndex, 2, timeout = 100)
    return data[0]

  def DOut(self, value, port=0):
    '''
      This command writes the DOut port latch.
      port:  the port latch to write:
           0  onboard (pins 0-7)

	   NOTE: The DIO are open-drain, which when used as an output is capable of sinking up to 150 mA.
	   Writing a "1" to a bit will cause its voltage to go LOW (0V), and writing a "0" to
	   the bit will cause the voltage to go HIGH (5V) by the 47k Ohm pullup resister.
	   See page 23 of the users manual.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.DOUT, wValue, wIndex, [port, value], timeout = 100)

  def DOutR(self, port=0):
    '''
      This command reads the DOut port latch
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    data = self.udev.controlRead(request_type, self.DOUT, port, wIndex, 1, timeout = 100)
    return data[0]
    

  #################################
  #    Analog Input  Commands     #
  #################################

  def AIn(self, channel, mode, gain, rate):
    '''
     The command returns the value from the specified analog input channel.  The channel
     configuration is specified in the command.  The command will result in a bus stall
     if an AInScan is currently running.

     channel: 0-15 or 31  the input channel to read
     mode:    0: Differential voltage
              1: Single-ended voltage, high pin
              2: Single-ended voltage, low pin
              3: D/A readback voltage
              4: Thermocouple
              5: AIn offset calibration
              6: AIn gain calibration
              7: TC Offset Calibration
              8: TC Gain Calibration Positive
              9: TC Gain Calibration Negative
             10: Thermocouple no burnout detect
     range:   0-8  
     rate:    0-15 

     value: signed 24 bit value frad from the analog input channel
     flags: bits  0-6: reserved
            bit7: 1 = TC open detected,  0 = normal reading
    '''
    if self.productID == 0xfd or self.productID == 0xfe: # USB-2408 or USB-2408-2AO
      if mode == self.DIFFERENTIAL or mode == self.THERMOCOUPLE:
        self.NCHAN = 8
      elif mode == self.SINGLE_ENDED: # Single Ended
        self.NCHAN = 16
        if channel < 8:
          mode = self.SE_HIGH
        else:
          mode = self.SE_LOW
    else: # USB-2416 or USB-2416-4AO
      if (self.Status() & 0x2) == 0:  # EXP not detected
        if mode == self.DIFFERENTIAL or mode == self.THERMOCOUPLE:
          self.NCHAN = 16
        elif mode == self.SINGLE_ENDED: # Single Ended
          self.NCHAN = 32
          if channel < 16:
            mode = self.SE_HIGH
          else:
            mode = self.SE_LOW
      else: # EXP detected
        if mode == self.DIFFERENTIAL or mode == self.THERMOCOUPLE:
          self.NCHAN = 32
        elif mode == self.SINGLE_ENDED: # Single Ended
          self.NCHAN = 64
          if channel < 32:
            mode = self.SE_HIGH
          else:
            mode = self.SE_LOW

    if channel > self.NCHAN:
      raise ValueError('AIN: value of channel out of range')
      return
    wValue = (mode << 8) | channel
    wIndex = (rate << 8) | gain
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    data ,= unpack('I',self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 4, timeout = 200))
    flags = (data >> 24)
    data = self.int24ToInt(data)
    return (data, flags)

  def AInScanStop(self):
    '''
      This command stops the analog input scan (if running)
    '''    
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.AIN_SCAN_STOP, wValue, wIndex, [0], timeout = 100)

  def AInScanStatus(self):
    '''
    This command reads the status of the analog input scan

      depth:  the number of samples currently in the FIFO (max 512)
      status:  bit 0: 1 = scan running
               bit 1: 1 = scan overrun due to FIFO full
               bit 2: 1 = scan overrun due to pacer period too short for queue
               bits 3-7: reserved
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    data = list(unpack('HB',self.udev.controlRead(request_type, self.AIN_SCAN_STATUS, wValue, wIndex, 3, timeout = 100)))
    return(data[0], data[1])

  def AInScanQueue(self):
    '''
    This command reads or writes the analog input scan channel queue.  The
    queue may have a maximum of 64 entries.  The queue can not be mondified
    during an AInScan.

    The minimum pacer period can be calculated from the queue data.  The formula is:
                  
                ---
                \      1
       period = /     ---  + 640 us
		---   rate

      i.e. If you have a queue with the following elements:
           channel 0: 100 SPS
           channel 1: 500 SPS
           channel 2: 60 SPS

     Then the minimum allowable pacer period for this queue would be:
        (1/100 + 640us) + (1/500 + 640us) + (1/60 + 640us) = 30.59 ms

    This results in a maximum rate of 32.69 Hz.

    For each queue entry:
      channel:  the analog input channel
      mode:     the input mode (see AIn)
      range:    the input range (see AIn)
      rate:     the A/D data rate (see AIn)
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    buf = [0]*(1+64*4)
    buf[0] = self.Queue[0]
    for i in range(self.Queue[0]):
      if self.productID == 0xfd or self.productID == 0xfe: # USB-2408 or USB-2408-2AO
        if self.Queue[i+1].mode == self.SINGLE_ENDED and self.Queue[i+1].channel < 8:
          self.Queue[i+1].mode = self.SE_HIGH
        if self.Queue[i+1].mode == self.SINGLE_ENDED and self.Queue[i+1].channel >= 8:
          self.Queue[i+1].mode = self.SE_LOW
      else: # USB-2416 or USB-2416-4AO
        if (self.Status() & 0x2) == 0:  # EXP not detected
          if self.Queue[i+1].mode == self.SINGLE_ENDED and self.Queue[i+1].channel < 16:
            self.Queue[i+1].mode = self.SE_HIGH
          if self.Queue[i+1].mode == self.SINGLE_ENDED and self.Queue[i+1].channel >= 16:
            self.Queue[i+1].mode = self.SE_LOW
        else: # EXP detected
          if self.Queue[i+1].mode == self.SINGLE_ENDED and self.Queue[i+1].channel < 32:
            self.Queue[i+1].mode = self.SE_HIGH
          if self.Queue[i+1].mode == self.SINGLE_ENDED and self.Queue[i+1].channel >= 32:
            self.Queue[i+1].mode = self.SE_LOW
            
    for i in range(self.Queue[0]):
      buf[4*i+1] = int(self.Queue[i+1].channel)
      buf[4*i+2] = int(self.Queue[i+1].mode)
      buf[4*i+3] = int(self.Queue[i+1].gain)
      buf[4*i+4] = int(self.Queue[i+1].rate)
    try:
     result = self.udev.controlWrite(request_type, self.AIN_SCAN_QUEUE, wValue, wIndex, buf, timeout = 100)
    except:
      print('AInScanQueue: error loading Gain Queue.')
    return

  def AInScanQueueR(self):
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    try:
      result = self.udev.controlRead(request_type, self.AIN_SCAN_QUEUE, wValue, wIndex, 257, timeout = 100)
    except:
      print('AInScanQueueR: error in reading queue')
    return result
  
  def AInMinPacerPeriod(self):
    '''
    Calculate the minimum allowable pacer period.
    '''
    period = 0.0
    for i in range(1,self.Queue[0]+1):
      if self.Queue[i].rate == self.HZ30000:
        period += 1./30000. + 640.E-6
      elif self.Queue[i].rate == self.HZ15000:
        period += 1./15000. + 640.E-6
      elif self.Queue[i].rate == self.HZ7500:
        period += 1./7500. + 640.E-6
      elif self.Queue[i].rate == self.HZ3750:
        period += 1./3750. + 640.E-6
      elif self.Queue[i].rate == self.HZ2000:
        period += 1./2000. + 640.E-6
      elif self.Queue[i].rate == self.HZ1000:
        period += 1./1000. + 640.E-6
      elif self.Queue[i].rate == self.HZ500:
        period += 1./500. + 640.E-6
      elif self.Queue[i].rate == self.HZ100:
        period += 1./100. + 640.E-6
      elif self.Queue[i].rate == self.HZ60:
        period += 1./60. + 640.E-6
      elif self.Queue[i].rate == self.HZ50:
        period += 1./50. + 640.E-6
      elif self.Queue[i].rate == self.HZ30:
        period += 1./30. + 640.E-6
      elif self.Queue[i].rate == self.HZ25:
        period += 1./25. + 640.E-6
      elif self.Queue[i].rate == self.HZ15:
        period += 1./15. + 640.E-6
      elif self.Queue[i].rate == self.HZ10:
        period += 1./10. + 640.E-6
      elif self.Queue[i].rate == self.HZ5:
        period += 1./5. + 640.E-6
      elif self.Queue[i].rate == self.HZ2_5:
        period += 1./2.5 + 640.E-6
      else:
        raise ValueError('AInMinPacerPeriod: Unknown rate')
        return
    return period

  def AInScanFlush(self):
    while True:
      try:
        result = self.udev.bulkRead(1, 64, timeout = 100)
        if len(result) <= 64:
          return
      except:
        return

  def AInScanStart(self, frequency, count, packet_size=15):
    '''
    count: The total number of scans to perform. (0 causes continuous scan)
    packet_size: the number of samples per bulk transfer (0-15)

    This command starts an analog input channel scan.  The channel
    configuration for the scan is set with AInScanQueue.  This command
    will result in a bus stall if AInScan is currently running.

    The sample rate is set by an internal incrementing timer running
    at a base rate of 50 kHz.  The timer is controllered by
    pacer_period.  The timer will be reset and samples acquired when
    its value equals pacer_period.  The equation for calculating
    paercer_period is:

        pacer_period = 50,000 / (sample frequency)

    Every time the pacer timer expires the firmware will acquire all
    of the samples specified in the channel gain queue, using the A/D
    data rates specified in the queue.  Therefore, the pacer period
    specified here is the per-channel scan rate.  The time to acquire
    all of the samples will depend on the number of entries in the
    queue and the data rate for each entry.

    The data will be returned in packets utilizing a bulk IN endpoint.
    Each sample will consist of 4 bytes: the MSB will be the scan
    queue index for the samples (0-63) followed by the 24-bit signed
    data value.  Each packet will have packet_size + 1 samples; use a
    low value of packet_size for slow scan rates to improve latency
    and a high value for fast rates for better performance.

    If a queue item is a thermocouple measurement, the MSB of the data
    will also reflect the burnout detection status in the high bit
    (i.e. If burnout is detected the high bit will be 1).  The next
    highest bit (bit 6 of the MSB) will indicate overrun if set.

    Data will be sent until reaching the specified count or an
    AInScanStop() command is sent.
    '''

    request_type = libusb1.LIBUSB_TYPE_VENDOR
    if packet_size > 15:
      packet_size = 15
    if packet_size < 0:
      packet_size = 0
      
    period = self.AInMinPacerPeriod()
    if (period > 1./frequency):
      pacer_period = int(round(period*50000.))
    else:
      pacer_period = int(round(50000./frequency))

    (depth, status) = self.AInScanStatus()
    if status & self.INPUT_SCAN_RUNNING:
      print('There are currently',depth,'samples in the FIFO buffer')
      return
    scanPacket = list(unpack('B'*7, pack('IHB',pacer_period, count, packet_size)))
    result = self.udev.controlWrite(request_type, self.AIN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

  def AInScanRead(self, count, options):
    # Each scan consists of 4 bytes times the number of active channels in the scan queue
    # each sample consisits of [ scan queue index | 24 bit signed value ]
    nBytes = count*self.Queue[0]*4  
    data = [0]*nBytes
    try:
      data = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, nBytes, 0)
    except:
      print('AInScanRead: error in bulk transfer')
      return
    data = list(unpack('I'*count*self.Queue[0], data))
      
    if (options & self.CONTINUOUS):
      return data
    self.AInScanStop()
    self.AInScanFlush()
    return data
    

  #################################
  #     Counter  Commands         #
  #################################
  def CounterInit(self, counter):
    '''
    This command initializes the 32-bit event counter.  On a write, the
    counter will be initialized to zero.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.COUNTER, wValue, wIndex, [counter], timeout = 100)

  def Counter(self, counter):
    '''
    This command reads the 32-bit event counter.  
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = 0
    wIndex = 0
    result = unpack('II',self.udev.controlRead(request_type, self.COUNTER, wValue, wIndex, 8, timeout = 100))
    if counter == self.COUNTER0:
      return result[0]
    else:
      return result[1]

  #################################
  #     Memory  Commands          #
  #################################
  def MemoryR(self, address, length):
    '''
    This command reads data from the available data EEPROM memory.
    The number of bytes to read is specified in the wLength (for
    writes it is wLength - sizeof(address)).  The first 2 byes of data is
    the address.

    Note: this function is not reentrant
  */
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = address
    wIndex = 0
    try:
      result = self.udev.controlRead(request_type, self.MEMORY, wValue, wIndex, length, timeout = 100)
    except:
      print('MemoryR: error')
    return result

  def MemoryW(self, address, data):
    address = address & 0xffff    # force to be 16 bits
    request_type = libusb1.LIBUSB_TYPE_VENDOR 
    wValue = address
    wIndex = 0
    self.udev.controlWrite(request_type, self.MEMORY, wValue, wIndex, [address, data[0:data.len()]], timeout = 100)
    return result
    

  #################################
  #     Miscellaneous Commands    #
  #################################

  def Reset(self):
    '''
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.RESET, wValue, wIndex, [0], timeout = 100)

  def Blink(self, count=1):
    '''
    This command causes the LED to blink "count" times.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.BLINK_LED, wValue, wIndex, [count], timeout = 100)

  def CJC(self):
    '''
    This command reads the CJC sensors.  The value returned is the raw
    CJC sensor reading, not compensated with the stored gradient
    value.  The temperature in degrees Celsius is calculated as:

     T = 128.(value/2^15)
    '''    
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0

    if self.productID == 0x00fd or self.productID == 0x00fe:
      value = unpack('hh',self.udev.controlRead(request_type, self.CJC_SENSOR, wValue, wIndex, 4, timeout = 100))
      temp = [0.0, 0.0]
      for i in range(2):
        temp[i] = value[i]/256.0
      return temp
    else:
      value = unpack('h'*8,self.udev.controlRead(request_type, self.CJC_SENSOR, wValue, wIndex, 16, timeout = 100))
      temp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      for i in range(8):
        temp[i] = value[i]/256.0
      return temp

  def CalConfig(self, value):
    '''
    This command will configure the calibration source.
    value =  0:  +0.078125V
             1:  -0.078125V
             2:  +0.15625V
	     3:  -0.15625V
	     4:  +0.3125V
     	     5:  -0.3125V
	     6:  +0.625V
     	     7:  -0.625V
	     8:  +1.25V
     	     9:  -1.25V
	    10:  +2.50V
     	    11:  -2.50V
	    12:  +5.00V
     	    13:  -5.00V
	    14:  +10.0V
     	    15:  -10.0V
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.CAL_CONFIG, wValue, wIndex, [value], timeout = 100)
    
  def Status(self):
    '''
    This command retrieves the status of the device.
    status:   bit 0:    1 = isolated micro ready for commands
              bit 1:    1 = EXP detected (USB-2416 and USB-2416-4AO only)
              bits 2-7: reserved
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    status ,= unpack('B',self.udev.controlRead(request_type, self.GET_STATUS, wValue, wIndex, 1, timeout = 100))
    return status

  def ADCal(self):
    '''
    The command will start the A/D self-calibration process and return
    the status of the calibration.
      status: 0 = calibration complete, 1 = calibration running
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.AD_CAL, wValue, wIndex, [0], timeout = 1000)
    status = 1
    while status == 1:
      status = self.udev.controlRead(request_type, self.AD_CAL, wValue, wIndex, 1, timeout = 1000)
      time.sleep(1)
    return status

  def TCCalMeasure(self, value):
    '''
      The command will enable measurement of the TC cal source
      value: 0: normal operation
             1: TC cal source measurment mode (JP3)
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.TC_CAL, wValue, wIndex, [value], timeout = 100)

  def SetSerial(self, serial):
    '''
    This command writes the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001"). The new serial number will be programmed but not used until
    hardware reset.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.SERIAL, wValue, wIndex, [serial], timeout = 100)
    
  def Version(self):
    '''
    This command reads the microcontroller firmware versions.  The
    firmware versions are returned as packed hexadecmal BCD values,
    i.e. if version = 0x0132, then the firmware version is 1.32.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    version = list(unpack('HHHH',self.udev.controlRead(request_type, self.VERSION, wValue, wIndex, 8, timeout = 100)))
    version[0] = str(int(version[0]/0x100)) + '.' + str(version[0]%0x100)
    version[1] = str(int(version[1]/0x100))+ '.' + str(version[1]%0x100)
    version[2] = str(int(version[2]/0x100)) + '.' + str(version[2]%0x100)
    version[3] = str(int(version[3]/0x100)) + '.' + str(version[3]%0x100)
    return version
                     
    
  #################################
  #       Firmware Update         #
  #################################

  def UpdateMode(self, micro):
    '''
    The command puts the device into firmware update mode which
    allows updating the code for either microcontroller.  The unlock
    code must be correct as a safety mechanism to prevent inadvertent
    code corruption.  If the device is not in update ode, all of the
    other firmware update commands will result in a control pipe
    stall.  A reset command must be issued at the end of the code
    update in order to return the device to operation with the new
    code.

    micro; which device to update:  0 - USB micro,  1 - isolated micro
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.UPDATE_MODE, wValue, wIndex, [0xad, micro], timeout = 100)

  def UpdateAddress(self, address):
    '''
    This command reads or writes the address used for code
    downloading.  The destination microcontroller for the download is
    selected when calling UpdateMode.

    The USB microcontroller memory map is:
      ---------------------------------------
      |  Address   |       Description      |
      ---------------------------------------
      | 0x000000   |  FLASH program memory  |
      | 0x0075FF   |                        |
      ---------------------------------------
  
    The isolated microcontroller memory map is:
      ---------------------------------------
      |  Address   |       Description      |
      ---------------------------------------
      | 0x000000   |  FLASH program memory  |
      | 0x014FFF   |                        |
      ---------------------------------------
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    value = [address & 0xff, (address >>8) & 0xff, (address >>16) & 0xff] 
    result = self.udev.controlWrite(request_type, self.UPDATE_ADDRESS, wValue, wIndex, value, timeout = 100)

  def UpdateAddressR(self):
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    value ,= unpack('BBB',self.udev.controlRead(request_type, self.UPDATE_ADDRESS, wValue, wIndex, 3, timeout = 100))
    address = value[0] | (value[1]<<8) | (value[2]<<16)
    return address

  def UpdateData(self, data):
    '''This command reads or writes the program memory in the
    microcontroller.  This command is not accepted unless the device
    is in update mode.  This command will normally be used when
    downloading a new hex file, so it supports the momory ranges that
    may be found in the hex file.  The number o f bytes to be read or
    written much be specified in wLength.

    USB microcontroller: The device must receive data in 64-byte
    transfers that originate on a 64-byte boundary.

    Isolated microcontroller: The device must receive data in 64-byte
    transfers that orginate on a 256-byte boundary.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    if len(data) % 64 != 0:
      raise ValueError('UpdateData: Data must be divisible by 64')
      return

    value = [address & 0xff, (address >>8) & 0xff, (address >>16) & 0xff] 
    result = self.udev.controlWrite(request_type, self.UPDATE_DATA, wValue, wIndex, data, timeout = 100)

  def UpdateDataR(self, length):
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    data = unpack('B'*length, self.udev.controlRead(request_type, self.UPDATE_DATA, wValue, wIndex, length, timeout = 100))

  def UpdateVersion(self):
    '''
    This command retrieves the firmware version of the update code.
    This command is not accepted unless the device is in update mode
    and the microcontroller is selected.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    wValue = 0
    wIndex = 0
    version = unpack('H', self.udev.controlRead(request_type, self.UPDATE_VERSION, wValue, wIndex, 2, timeout = 100))
    return version


  def getMFGCAL(self):
    '''
      get the manufacturers calibration data (timestamp) from EEPROM
    '''
    # get the year (since 2000)
    address = 0x0640
    data ,= unpack('B', self.MemoryR(address, 1))
    year = 2000+data

    # get the month
    address = 0x641
    month ,= unpack('B', self.MemoryR(address, 1))

    # get the day
    address = 0x642
    day ,= unpack('B', self.MemoryR(address, 1))

    # get the hour
    address = 0x643
    hour ,= unpack('B', self.MemoryR(address, 1))

    # get the minute
    address = 0x644
    minute ,= unpack('B', self.MemoryR(address, 1))

    # get the second
    address = 0x645
    second ,= unpack('B', self.MemoryR(address, 1))

    mdate = datetime(year, month, day, hour, minute, second)
    return mdate

  def int24ToInt(self, int24val):
    # convers a 2's complement signed 24 bit number to an int (32 or 64 bit)
    SIGN_BITMASK = (1 << 23)
    FULL_SCALE24_BITMASK = ((1<<24) - 1)
    SIGN_EXT_BITMASK = (~FULL_SCALE24_BITMASK)
    int24val &= 0xffffff
    if int24val & SIGN_BITMASK:
      int24val |= SIGN_EXT_BITMASK
    else:
      int24val &= FULL_SCALE24_BITMASK;
    return int24val

  def intToInt24(self, int32):
    # converts an int to a 2's complement signed 24 bit number
    SIGN_BITMASK (1 << 23)
    FULL_SCALE24_BITMASK ((1<<24) - 1)
    SIGN_EXT_BITMASK (~FULL_SCALE24_BITMASK)

    if int32 < 0:
      int32 &= SIGN_EXT_BITMASK
      int32 |= SIGN_BITMASK
    else:
      int32 &= FULL_SCALE24_BITMASK
    return int32

  def volts(self, gain, value):
    '''
    converts 24 bit signed value to volts
    '''
    if gain == self.BP_10_00V:
      volt = value * 10.0 / 0x7fffff;
    elif gain == self.BP_5_00V:
      volt = value * 5.0 / 0x7fffff;
    elif gain == self.BP_2_50V:
      volt = value * 2.5 / 0x7fffff;
    elif gain == self.BP_1_25V:
      volt = value * 1.25 / 0x7fffff;
    elif gain == self.BP_0_625V:
      volt = value * 0.625 / 0x7fffff;
    elif gain == self.BP_0_312V:
      volt = value * 0.3125 / 0x7fffff;
    elif gain == self.BP_0_156V:
      volt = value * 0.15625 / 0x7fffff;
    elif gain == self.BP_0_078V:
      volt = value * 0.078125 / 0x7fffff;
    elif gain == 9:
      volt = value * 0.078125 / 0x7fffff;
    else:
      raise ValueError('volts: Unknown voltage range.')
      return 
    return volt

  def Temperature(self, tc_type, channel):
    tc = Thermocouple()
    # Read the raw voltage (Mode = 4, Range = +/- .078V, Rate = 1kS/s)
    (value, flag) = self.AIn(channel, 4, 8, 5)
    if flag & 0x80:
      print('TC open detected.  Check wiring on channel', channel)
      return -1
    # Apply calibration offset from Gain Table (EEPROM) address 0x0130 (slope) and 0x0138 (offset)
    value = value*self.Cal[9].slope + self.Cal[9].intercept
    # Convert the corrected valued to a voltage (Volts)
    tc_voltage = (value * 2. * 0.078125) / 16777216.
    # Correct the CJC Temperature by the CJCGradiant for the appropriate channel
    cjc = self.CJC()
    CJC_temp = cjc[channel//4] - self.CJCGradient[channel]
    # Calculate the CJC voltage using the NIST polynomials and add to tc_voltage in millivolts
    tc_mv = tc.temp_to_mv(tc_type, CJC_temp) + tc_voltage*1000.
    # Calcualate actual temperature using reverse NIST polynomial.
    return tc.mv_to_temp(tc_type, tc_mv)

################################################################################################
 
class usb_2408(usb_2400):
  def __init__(self, serial=None):
    self.productID = 0x00fd    # usb-2408
#   self.context = usb1.USBContext()
#   self.udev = self.context.openByVendorIDAndProductID(0x9db, self.productID)
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-2408 not found")
      return
    usb_2400.__init__(self)

    self.CJCGradient = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # CJC Gradients
    address = 0x0140
    for chan in range(8):
      self.CJCGradient[chan] ,= unpack('f', self.MemoryR(address, 4))
      address += 4

class usb_2408_2AO(usb_2400):
  def __init__(self, serial=None):
    self.productID = 0x00fe    # usb-2408-2AO
#   self.context = usb1.USBContext()
#   self.udev = self.context.openByVendorIDAndProductID(0x9db, self.productID)
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-2408-2AO not found")
    usb_2400.__init__(self)

    NCHAN_AO = 2        # number of analog output channels (USB-2408-2AO only)

    self.CJCGradient = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # CJC Gradients
    address = 0x0140
    for chan in range(8):
      self.CJCGradient[chan] ,= unpack('f', self.MemoryR(address, 4))
      address += 4

    # Build a lookup table of calibration coefficients to translate values to voltages:
    # corrected value = value * Cal_AO[DAC#].slope + Cal_AO[DAC#].intercept
    self.Cal_AO = [table(), table()]
    address = 0x0180
    for i in range(0,2): # two analog output channels
      self.Cal_AO[i].slope ,= unpack('d', self.MemoryR(address, 8))
      address += 8
      self.Cal_AO[i].intercept ,= unpack('d', self.MemoryR(address, 8))
      address += 8

  ###################################################
  #    Analog Output  Commands  USB-2408-2AO only   #
  ###################################################

  def AOut(self, channel, voltage, command=-1):
    '''
    This command writes the values for the analog output channels.  The
    values are 16-bit unsigned numbers.  This command will result in a control
    pipe stall if an output scan is running.  The equation for the output voltage is:

          V_out = (value - 32768 / 32768)* V_ref

    where "value" is the value written to the channel and V_ref = 10V.  

    command values:
                     0x00  -  write value to buffer 0
                     0x04  -  write value to buffer 1
                     0x10  -  write value to buffer 0 and load DAC 0
                     0x14  -  write value to buffer 1 and load DAC 0
                     0x20  -  write value to buffer 0 and load DAC 1
                     0x24  -  write value to buffer 1 and load DAC 1
                     0x30  -  write value to buffer 0 and load DAC 0 & DAC 1
                     0x34  -  write value to buffer 1 and load DAC 0 & DAC 1
    '''
    value = voltage*32768./10. + 32768.
    value = int(value*self.Cal_AO[channel].slope + self.Cal_AO[channel].intercept)

    if value >= 0xffff:
      value = 0xffff
    elif value <= 0:
      value = 0x0
    else:
      value = value & 0xffff

    if command == -1:  # use default values
      if channel == 0:
        command = 0x10
      else:
        command = 0x24

    (status, depth) = self.AOutScanStatus()
    if status & self.OUTPUT_SCAN_RUNNING:
      print('There are currently', depth, 'samples in the Output FIFO buffer.')
      return

    request_type = libusb1.LIBUSB_TYPE_VENDOR     
    result = self.udev.controlWrite(request_type, self.AOUT, 0x0, 0x0, \
                      [value & 0xff, (value >> 8) & 0xff, command], timeout = 100)

  def AOutScanStop(self):
    '''
    This command stops the analog output scan (if running) and
    clears the output FIFO data.  Any data in the endpoint buffers will
    be flushed, so this command is useful to issue prior to the
    beginning of an output scan.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR     
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_STOP, 0x0, 0x0, [0x0], timeout = 100)

  def AOutScanStatus(self):
    '''
    This comamnd reads the status of the analog output scan:
    depth: the number of samples currently in the FIFO (max 1024)
    status: bit 0: 1 = scan running
            bit 1: 1 = scan underrun
            bits 2-7: reserved
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR     
    result = unpack('BBB',self.udev.controlRead(request_type, self.AOUT_SCAN_STATUS, 0x0, 0x0, 3, timeout = 100))
    depth = (result[0] | result[1] << 8)
    status = result[2]
    return (depth, status)

  def AOutScanStart(self, frequency, scans, options):
    '''
     This command configures the analog output channel scan.
     This command will result in a bus stall if an AOUT_SCAN is
     currently running.

     Notes:
     The output scan operates with the host continuously transferring data for the
     outputs until the end of the scan.  If the "scans" parameter is 0, the scan will run
     until the AOutScanStop command is issued by the host; if it is nonzero, the scan
     will stop automatically after the specified number of scans have been output.
     The channels in the scan are selected in the options bit field.  "Scans" refers to
     the number of updates to the channels (if all channels are used, one scan is an
     update to all 2 channels).

     period = 50kHz / frequency

     Multiple channels are updated simultaneously using the same time base.

     The output data is sent using the bulk out endpoint.  The data format is:
     low channel sample 0 : ... : [high channel sample 0]
     low channel sample 1 : ... : [high channel sample 1]
     .
     .
     .
     low channel sample n : ... : [high channel sample n]

     The output data is written to a 512-sample FIFO in the device.  The bulk endpoint
     data is only accepted if there is room in the FIFO.  Output data may be sent to the
     FIFO before the start of the scan, and the FIFO is cleared when the AOutScanStop command
     is received.  The scan will not begin until the command is sent (and output data is in
     the FIFO).  Data will be output until reaching the specified number of scans (in single
     execution mode)or an AOutScanStop command is sent.
    '''
    if frequency <= 0.:
      raise ValueError('AOutScanStart: frequency must be positive')
      return
    elif frequency > 50000:
      raise ValueError('AOutScanStart: frequency must be less than 50 KHz')
      return
    else:
      pacer_period = int(round(50000./frequency)) & 0xffffffff
    scans &= 0xffff                                      # scans = 0 for continuous
    options &= 0xff
    (status, depth) = self.AOutScanStatus()
    if status & self.OUTPUT_SCAN_RUNNING:
      print('There are currently', depth, 'samples in the Output FIFO buffer.')
      return
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    buf = bytearray(7)
    buf[0] = pacer_period         & 0xff
    buf[1] = (pacer_period >> 8)  & 0xff
    buf[2] = (pacer_period >> 16) & 0xff
    buf[3] = (pacer_period >> 24) & 0xff
    buf[4] = scans                & 0xff
    buf[5] = (scans >> 8)         & 0xff
    buf[6] = options
              
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_START, 0x0, 0x0, buf, timeout = 1000)

  def AOutScanWrite(self, data):
    # data is a list of unsigned 16 bit numbers
    value = [0]*len(data)*2
    for i in range(len(data)):
      value[2*i] = data[i] & 0xff
      value[2*i+1] = (data[i] >> 8) & 0xff
    try:
      result = self.udev.bulkWrite(1, value, timeout = 10000)
    except:
      print('AOutScanWrite: error in bulkWrite')
      return
    
class usb_2416(usb_2400):
  BP_20_00V = 0x0      # +/- 20.0 V

  def __init__(self, serial=None):
    self.productID = 0x00d0    # usb-2416
#   self.context = usb1.USBContext()
#   self.udev = self.context.openByVendorIDAndProductID(0x9db, self.productID)
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-2416 not found")
    usb_2400.__init__(self)

    address = 0x00A0
    self.Cal[0].slope ,= unpack('d', self.MemoryR(address, 8))
    address += 8
    self.Cal[0].intercept ,= unpack('d', self.MemoryR(address, 8))

    self.CJCGradient = [1.2004, 0.9439, 0.6703, 0.3711, 1.2506, 1.1418, 1.0145, 0.9315,\
                        0.8663, 1.2399, 1.5692, 1.8611, 0.1736, 0.6306, 1.0432, 1.4027,\
                        1.2004, 0.9439, 0.6703, 0.3711, 1.2506, 1.1418, 1.0145, 0.9315,\
                        1.2004, 0.9439, 0.6703, 0.3711, 1.2506, 1.1418, 1.0145, 0.9315]
                        
class usb_2416_4AO(usb_2400):
  BP_20_00V = 0x0      # +/- 20.0 V
  NCHAN_AO  = 4        # number of analog output channels (USB-2416-4AO only)

  def __init__(self, serial=None):
    self.productID = 0x00d1    # usb-2416-4AO
#   self.context = usb1.USBContext()
#   self.udev = self.context.openByVendorIDAndProductID(0x9db, self.productID)
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-2416-4AO not found")
      return
    usb_2400.__init__(self)

    address = 0x00A0
    self.Cal[0].slope ,= unpack('d', self.MemoryR(address, 8))
    address += 8
    self.Cal[0].intercept ,= unpack('d', self.MemoryR(address, 8))

    self.CJCGradient = [1.2004, 0.9439, 0.6703, 0.3711, 1.2506, 1.1418, 1.0145, 0.9315,\
                        0.8663, 1.2399, 1.5692, 1.8611, 0.1736, 0.6306, 1.0432, 1.4027,\
                        1.2004, 0.9439, 0.6703, 0.3711, 1.2506, 1.1418, 1.0145, 0.9315,\
                        1.2004, 0.9439, 0.6703, 0.3711, 1.2506, 1.1418, 1.0145, 0.9315]

    # Build a lookup table of calibration coefficients to translate values to voltages:
    # corrected value = value * Cal_AO[DAC#].slope + Cal_AO[DAC#].intercept
    self.Cal_AO = [table(), table(), table(), table()]
    address = 0x0180
    for i in range(0,4): # four analog output channels
      self.Cal_AO[i].slope ,= unpack('d', self.MemoryR(address, 8))
      address += 8
      self.Cal_AO[i].intercept ,= unpack('d', self.MemoryR(address, 8))
      address += 8

  ###################################################
  #    Analog Output  Commands  USB-2416-4AO only   #
  ###################################################
  def AOut(self, channel, voltage, command=-1):
    '''
    This command writes the values for the analog output channels.  The
    values are 16-bit signed numbers.  This command will result in a control
    pipe stall if an output scan is running.  The equation for the output voltage is:

          V_out = (value / 32768)* V_ref

    where "value" is the value written to the channel and V_ref = 10V.  

    command values:
                     0x00  -  write value to buffer 0
                     0x02  -  write value to buffer 1
                     0x04  -  write value to buffer 2
                     0x06  -  write value to buffer 3
                     0x10  -  write value to buffer 0 and load DAC 
                     0x12  -  write value to buffer 1 and load DAC 
                     0x14  -  write value to buffer 2 and load DAC 
                     0x16  -  write value to buffer 3 and load DAC 
                     0x20  -  write value to buffer 0 and load DAC all DACs simultaneously
                     0x22  -  write value to buffer 1 and load DAC all DACs simultaneously
                     0x24  -  write value to buffer 2 and load DAC all DACs simultaneously
                     0x26  -  write value to buffer 3 and load DAC all DACs simultaneously
                     0x30  -  simultaneously load all DACs (ignore value)
                     0x34  -  write value to all buffers and simultaneously load all DACs
    '''
    value = voltage*32768./10.
    value = int(value*self.Cal_AO[channel].slope + self.Cal_AO[channel].intercept)

    if value >= 32768:
      value = 32767
    elif value <= -32768:
      value = -32768
    else:
      value = value & 0xffff

    if command == -1:  # use default values
      if channel == 0:
        command = 0x10
      elif channel == 1:
        command = 0x12
      elif channel == 2:
        command = 0x14
      elif channel == 3:
        command = 0x16

    (status, depth) = self.AOutScanStatus()
    if status & self.OUTPUT_SCAN_RUNNING:
      print('There are currently', depth, 'samples in the Output FIFO buffer.')
      return

    request_type = libusb1.LIBUSB_TYPE_VENDOR     
    result = self.udev.controlWrite(request_type, self.AOUT, 0x0, 0x0, \
                      [value & 0xff, (value >> 8) & 0xff, command], timeout = 100)
  def AOutScanStop(self):
    '''
    This command stops the analog output scan (if running) and
    clears the output FIFO data.  Any data in the endpoint buffers will
    be flushed, so this command is useful to issue prior to the
    beginning of an output scan.
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR     
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_STOP, 0x0, 0x0, [0x0], timeout = 100)

  def AOutScanStatus(self):
    '''
    This comamnd reads the status of the analog output scan:
    depth: the number of samples currently in the FIFO (max 1024)
    status: bit 0: 1 = scan running
            bit 1: 1 = scan underrun
            bits 2-7: reserved
    '''
    request_type = libusb1.LIBUSB_TYPE_VENDOR     
    result = unpack('BBB',self.udev.controlRead(request_type, self.AOUT_SCAN_STATUS, 0x0, 0x0, 3, timeout = 100))
    depth = (result[0] | result[1] << 8)
    status = result[2]
    return (depth, status)

  def AOutScanStart(self, frequency, scans, options):
    '''
    This command configures the analog output channel scan.
    This command will result in a bus stall if an AOUT_SCAN is
    currently running.

    Notes:
    The output scan operates with the host continuously transferring data for the
    outputs until the end of the scan.  If the "scans" parameter is 0, the scan will run
    until the AOutScanStop command is issued by the host; if it is nonzero, the scan
    will stop automatically after the specified number of scans have been output.
    The channels in the scan are selected in the options bit field.  "Scans" refers to
    the number of updates to the channels (if all channels are used, one scan is an
    update to all 2 channels).

       period = 50kHz / frequency 

    Multiple channels are updated simultaneously using the same time base.

    The output data is sent using the bulk out endpoint.  The data format is:
    low channel sample 0 : ... : [high channel sample 0]
    low channel sample 1 : ... : [high channel sample 1]
    .
    .
    .
    low channel sample n : ... : [high channel sample n]

    The output data is written to a 512-sample FIFO in the device.  The bulk endpoint
    data is only accepted if there is room in the FIFO.  Output data may be sent to the
    FIFO before the start of the scan, and the FIFO is cleared when the AOutScanStop command
    is received.  The scan will not begin until the command is sent (and output data is in
    the FIFO).  Data will be output until reaching the specified number of scans (in single
    execution mode)or an AOutScanStop command is sent.

       pacer_period: pacer timer period value
       scans:        the total number of scans to perform
       options:      bit 0: 1 = include channel 0 in output scan
                     bit 1: 1 = include channel 1 in output scan
                     bit 2: 1 = include channel 2 in output scan
                     bit 3: 1 = include channel 3 in output scan
                     bits 4-7: reserved
    '''
    if frequency <= 0.:
      raise ValueError('AOutScanStart: frequency must be positive')
      return
    elif frequency > 50000:
      raise ValueError('AOutScanStart: frequency must be less than 50 KHz')
      return
    else:
      pacer_period = int(round(50000./frequency)) & 0xffff
    scans &= 0xffff                           # scans = 0 for continuous
    options &= 0xff
    (status, depth) = self.AOutScanStatus()
    if status & self.OUTPUT_SCAN_RUNNING:
      print('There are currently', depth, 'samples in the Output FIFO buffer.')
      return
    request_type = libusb1.LIBUSB_TYPE_VENDOR
    buf = bytearray(5)
    buf[0] = pacer_period         & 0xff
    buf[1] = (pacer_period >> 8)  & 0xff
    buf[2] = scans                & 0xff
    buf[3] = (scans >> 8)         & 0xff
    buf[4] = options
              
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_START, 0x0, 0x0, buf, timeout = 1000)

  def AOutScanWrite(self, data):
    # data is a list of signed 16 bit numbers
    value = [0]*len(data)*2
    for i in range(len(data)):
      value[2*i] = data[i] & 0xff
      value[2*i+1] = (data[i] >> 8) & 0xff
    try:
      result = self.udev.bulkWrite(1, value, timeout = 10000)
    except:
      print('AOutScanWrite: error in bulkWrite')
      return

