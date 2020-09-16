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

class usb1808(mccUSB):

  # USB PIDs for family of devices
  USB_1808_PID  = 0x013d
  USB_1808X_PID = 0x013e

  # Counter Timer
  COUNTER0 = 0x0  # Counter 0
  COUNTER1 = 0x1  # Counter 1
  ENCODER0 = 0x2  # Counter 2
  ENCODER1 = 0x3  # Counter 3
  TIMER0   = 0x0  # Timer 0
  TIMER1   = 0x1  # Timer 1

  # Timer Control
  TIMER_ENABLE      = 0x1   # Enable timer
  TIMER_RUNNING     = 0x2   # Timer running
  TIMER_INVERTED    = 0x4   # Timer inverted output
  TIMER_OTRIG_BEGIN = 0x10  # Timer will begin output when the OTRIG pin has triggered
  TIMER_OTRIG       = 0x40  # Timer will continue to output on every OTRIG it receives

  # Counter Modes
  COUNTER_TOTALIZE   = 0x0   # Total Count (counts total number of pulses)
  COUNTER_PERIOD     = 0x1   # Counter returns Period [100us]
  COUNTER_PULSEWIDTH = 0x2   # Counter returns pulse width [100us]
  COUNTER_TIMING     = 0x3
  PERIOD_MODE_1X     = 0x0   # Period Mode x1
  PERIOD_MODE_10X    = 0x4   #  Period Mode x10
  PERIOD_MODE_100X   = 0x8   # Period Mode x100
  PERIOD_MODE_1000X  = 0xc   # Period Mode x1000
  TICK_SIZE_20NS     = 0x0   #  Tick size 20ns (fundamental unit of time in nano seconds)
  TICK_SIZE_200NS    = 0x10  # 200 ns
  TICK_SIZE_2000NS   = 0x20  # 2000 ns
  TICK_SIZE_20000NS  = 0x30  # 20000 ns

  # Counter Options
  CLEAR_ON_READ     = 0x1   # Clear on Read
  NO_RECYCLE        = 0x2   # No recycle mode
  COUNT_DOWN        = 0x4   # Count Down
  RANGE_LIMIT       = 0x8   # Range Limit (use max and min limits)
  FALLING_EDGE      = 0x10  # Count on the falling edge

  # Aanalog Input
  DIFFERENTIAL      = 0
  SINGLE_ENDED      = 1
  GROUNDED          = 3
  PACKET_SIZE       = 512    # max bulk transfer size in bytes
  CONTINUOUS        = 1      # continuous input mode
  EXTERNAL_TRIGGER  = 0x1    # 1 = use external trigger
  PATTERN_DETECTION = 0x2    # 1 = use Pattern Detection trigger
  RETRIGGER_MODE    = 0x4    # 1 = retrigger mode, 0 = normal trigger
  COUNTER_VALUE     = 0x8    # 1 = Maintain counter value on scan start,  0 = Clear counter value on scan start
  SINGLE_IO         = 0x10   # 1 = use SINGLE_IO data transfer,  0 = use BLOCK_IO transfer

  # Ananlog Output Scan Options
  AO_CHAN0       = 0x0   # Include Channel 0 in output scan
  AO_CHAN1       = 0x1   # Include Channel 1 in output scan
  AO_TRIG        = 0x10  # Use Trigger
  AO_RETRIG_MODE = 0x20  # Retrigger Mode
  
  # Ranges 
  BP_10V = 0x0      # +/- 10V
  BP_5V  = 0x1      # +/- 5V
  UP_10V = 0x2      # 0 - 10V
  UP_5V  = 0x3      # 0 - 5V
  
  # Status bit values
  AIN_SCAN_RUNNING   = (0x1 << 1)  # input pacer running
  AIN_SCAN_OVERRUN   = (0x1 << 2)  # input scan overrun
  AOUT_SCAN_RUNNING  = (0x1 << 3)  # output scan running
  AOUT_SCAN_UNDERRUN = (0x1 << 4)  # output scan overrun
  AIN_SCAN_DONE      = (0x1 << 5)  # input scan done
  AOUT_SCAN_DONE     = (0x1 << 6)  # output scan done
  FPGA_CONFIGURED    = (0x1 << 8)  # 1 = FPGA configured
  FPGA_CONFIG_MODE   = (0x1 << 9)  # 1 = FPGA config mode

  NCHAN               =   8  # max number of A/D channels in the device
  NGAIN               =   4  # max number of gain levels
  NCHAN_AO            =   2  # number of analog output channels
  NCOUNTER            =   4  # 2 counters 2 encoders
  NTIMER              =   2  # total number of timers
  MAX_PACKET_SIZE_HS  = 512  # max packet size for HS device
  MAX_PACKET_SIZE_FS  =  64  # max packet size for FS device
  BASE_CLOCK       = 100.E6  # Base clock frequency

  # AIn Scan Modes
  CONTINUOUS_READOUT   = 0x1  # Continuous mode
  SINGLEIO             = 0x2  # Return data after every read (used for low frequency scans)
  FORCE_PACKET_SIZE    = 0x4  # Force packet_size
  VOLTAGE              = 0x8  # return values as voltages

  # Commands and Codes for USB1608G
  # Digital I/O Commands
  DTRISTATE            = 0x00  # Read/write digital port tristate register
  DPORT                = 0x01  # Read digital port pins / write output latch register
  DLATCH               = 0x02  # Read/write digital port output latch register

  # Analog Input Commands
  AIN                  = 0x10  # Read analog input channel
  AIN_ADC_SETUP        = 0x11  # Read/write setup registers on the ADC
  AIN_SCAN_START       = 0x12  # Start analog input scan
  AIN_SCAN_STOP        = 0x13  # Stop analog input scan
  AIN_SCAN_CONFIG      = 0x14  # Read/Write analog input configuration
  AIN_SCAN_CLEAR_FIFO  = 0x15  # Clear the analog input scan FIFO
  AIN_BULK_FLUSH       = 0x16  # Flush the input Bulk pipe

  # Analog Output Commands
  AOUT                 = 0x18  # Read/write analog output channel
  AOUT_SCAN_CONFIG     = 0x19  # Read / write output scan queue
  AOUT_SCAN_START      = 0x1A  # Start analog output scan
  AOUT_SCAN_STOP       = 0x1B  # Stop analog output scan
  AOUT_SCAN_CLEAR_FIFO = 0x1C  # Clear the analog output scan FIFO

  # Counter Commands
  COUNTER              = 0x20  # Read or set the counter
  COUNTER_OPTIONS      = 0x21  # Read or set the counter's options
  COUNTER_LIMITS       = 0x22  # Read or set the counter's range limits
  COUNTER_MODE         = 0x23  # Read or set the counter's mode
  COUNTER_PARAMETERS   = 0x24  # Read or set the counter's mode and options

  # Timer Commandws
  TIMER_CONTROL        = 0x28  # Read/Write timer control register
  TIMER_PARAMETERS     = 0x2D  # Read/Write all timer parameters at once

  # Memory Commands
  MEMORY               = 0x30  # Read/Write EEPROM
  MEM_ADDRESS          = 0x31  # EEPROM read/write address value
  MEM_WRITE_ENABLE     = 0x32  # Enable writes to firmware area

  # Miscellaneous Commands
  STATUS               = 0x40  # Read device status
  BLINK_LED            = 0x41  # Causes the LED to blink
  RESET                = 0x42  # Reset the device
  TRIGGER_CONFIG       = 0x43  # External trigger configuration
  PATTERN_DETECT_CONF  = 0x44  # Pattern detection triger configuration
  SERIAL               = 0x48  # Read/Write USB Serial Number

  # FPGA Configuration Commands
  FPGA_CONFIG          = 0x50 # Start FPGA configuration
  FPGA_DATA            = 0x51 # Write FPGA configuration data
  FPGA_VERSION         = 0x52 # Read FPGA version

  HS_DELAY = 2000

  def __init__(self):
    self.status = 0                    # status of the device
    self.samplesToRead = -1            # number of bytes left to read from a scan
    self.scanQueueAIn = [0]*13         # depth of analog input scan queue is 13
    self.scanQueueAOut= [0]*3          # depth of analog output scan queue is 3
    self.lastElementAIn = 0            # last element of the analog input scan list
    self.lastElementAOut = 0           # last element of the analog output scan list
    self.count = 0
    self.retrig_count = 0
    self.options = 0
    self.frequency = 0.0               # frequency of scan (0 for external clock)
    self.packet_size = 512             # number of samples to return from FIFO
    self.mode = 0                      # mode bits:
                                       # bit 0:   0 = counting mode,  1 = CONTINUOUS_READOUT
                                       # bit 1:   1 = SINGLEIO
                                       # bit 2:   1 = use packet size in self.packet_size
                                       # bit 3:   1 = convert raw readings to voltages

    # Configure the FPGA
    if not (self.Status() & self.FPGA_CONFIGURED) :
      # load the FPGA data into memory
      from usb_1808_rbf import FPGA_data
      print("Configuring FPGA.  This may take a while ...")
      self.FPGAConfig()
      if self.Status() & self.FPGA_CONFIG_MODE:
        for i in range(0, len(FPGA_data) - len(FPGA_data)%64, 64) :
          self.FPGAData(FPGA_data[i:i+64])
        i += 64
        if len(FPGA_data) % 64 :
          self.FPGAData(FPGA_data[i:i+len(FPGA_data)%64])
        if not (self.Status() & self.FPGA_CONFIGURED):
          print("Error: FPGA for the USB-1808 is not configured.  status = ", hex(self.Status()))
          return
      else:
        print("Error: could not put USB-1808 into FPGA Config Mode.  status = ", hex(self.Status()))
        return
    else:
      print("USB-1808 FPGA configured.")

    if sys.platform.startswith('linux'):
      if self.udev.kernelDriverActive(0):
        self.udev.detachKernelDriver(0)
        self.udev.resetDevice()

    # claim all the needed interfaces for AInScan
    self.udev.claimInterface(0)

    # Find the maxPacketSize for bulk transfers
    self.wMaxPacketSize = self.getMaxPacketSize(libusb1.LIBUSB_ENDPOINT_IN | 0x6)  #EP IN 6

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #   self.table_AIn[channel][gain]  0 <= chan < 8,  0 <= gain < 4
    #
    self.table_AIn = [[table(), table(), table(), table()], \
                      [table(), table(), table(), table()], \
                      [table(), table(), table(), table()], \
                      [table(), table(), table(), table()], \
                      [table(), table(), table(), table()], \
                      [table(), table(), table(), table()], \
                      [table(), table(), table(), table()], \
                      [table(), table(), table(), table()]]

    address = 0x7000
    for chan in range(self.NCHAN):
      for gain in range(self.NGAIN):
        self.MemAddressW(address)
        self.table_AIn[chan][gain].slope, = unpack('f', self.MemoryR(4))
        address += 4
        self.MemAddressW(address)
        self.table_AIn[chan][gain].intercept, = unpack('f', self.MemoryR(4))
        address += 4

    # Read calibration table for analog out
    self.table_AOut = [table(), table()]
    address = 0x7100
    for chan in range(self.NCHAN_AO):
      self.MemAddressW(address)
      self.table_AOut[chan].slope, = unpack('f', self.MemoryR(4))
      address += 4
      self.MemAddressW(address)
      self.table_AOut[chan].intercept, = unpack('f', self.MemoryR(4))
      address += 4

    # Set up the ADC Configuration
    #  Default to +/- 10V Differential
    self.AInConfig = [0, 0, 0, 0, 0, 0, 0, 0]

    # Set up structure for Counter Parameters
    self.counterParameters = [CounterParameters(), CounterParameters(), CounterParameters(), CounterParameters()]

    # Set up structure and initialize for Timer Parameters
    self.timerParameters = [TimerParameters(), TimerParameters()]
    for timer in range(self.NTIMER):
      self.timerParameters[timer].timer = int(timer)
      self.TimerParametersW(timer, 1000., 0.5, 0, 0)

  def CalDate(self):
    """
    Get the manufacturers calibration data (timestamp) from the
    Calibration memory
    """

    # get the year (since 2000)
    address = 0x7110
    self.MemAddressW(address)
    year ,= unpack('B', self.MemoryR(1))
    year += 2000

    # get the month
    address = 0x7111
    self.MemAddressW(address)
    month ,= unpack('B', self.MemoryR(1))

    # get the day
    address = 0x7112
    self.MemAddressW(address)
    day ,= unpack('B', self.MemoryR(1))

    # get the hour
    address = 0x7113
    self.MemAddressW(address)
    hour ,= unpack('B', self.MemoryR(1))
    
    # get the minute
    address = 0x7114
    self.MemAddressW(address)
    minute ,= unpack('B', self.MemoryR(1))

    # get the second
    address = 0x7115
    self.MemAddressW(address)
    second ,= unpack('B', self.MemoryR(1))

    mdate = datetime(year, month, day, hour, minute, second)
    return mdate
        
  ##############################################
  #           Digital I/O  Commands            #
  ##############################################
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
    wValue = value & 0xffff
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
    wValue = value & 0xffff
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  #############################################
  #        Analog Input Commands              #
  #############################################

  def AIn(self, voltage = False):
    """
    This command performs an asynchronous read of all analog input channels. 
    value[8]: 18-bit data read from that channel
    
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = list(unpack('IIIIIIII', self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 32, self.HS_DELAY)))
    for chan in range(self.NCHAN):
      gain = self.AInConfig[chan] & 0x3
      value[chan] = self.table_AIn[chan][gain].slope * value[chan] + self.table_AIn[chan][gain].intercept
      if value[chan] > 0x3ffff:
        value[chan] = 0x3ffff
      elif value[chan] < 0:
        value[chan] = 0x0
      else:
        value[chan] = round(value[chan])

    if voltage == True:  # return voltage instead of raw readings
      for chan in range(self.NCHAN):
        value[chan] = self.volts(gain, value[chan])
    
    return value
      

  def ADCSetup(self, channel, gain, mode):
    """
     This command reads or writes the range configuration for all
     analog input channels (each channel can have its own unique
     range).  Each bye in this array corresponds to the analog input
     channel and the value determines that channel's range and input
     type.
     Bits 1-0: +/- 10V    = 0     
               +/-  5V    = 1
               0 - 10V    = 2
               0 -  5V    = 3
     Bits 3-2: Differential = 0
               Single Ended = 1
               Grounded     = 3
     Bits 7-4: Reserved 
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_ADC_SETUP
    wValue = 0
    wIndex = 0
    self.AInConfig[channel] = (mode & 0x3) << 2 | (gain & 0x3)
    self.udev.controlWrite(request_type, request, wValue, wIndex, self.AInConfig, self.HS_DELAY)

  def ADCSetupR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = unpack('BBBBBBBB',self.udev.controlRead(request_type, self.AIN_ADC_SETUP, wValue, wIndex, 8, self.HS_DELAY))
    return value

  def AInScanStart(self, count, retrig_count, frequency, options, mode=0x0):
    """
    This command starts the analog input channel scan.  The gain
    ranges that are currently set on the desired channels will be
    used (these may be changed with AInConfig) This command will
    result in a bus stall if an AInScan is currently running.

    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 100 MHz.  The timer is controlled by
    pacer_period. A pulse will be output at the ICLKO pin at
    every pacer_period interval regardless of the mode.

    If pacer_period is set to 0, the device does not generate an A/D
    clock.  It uses the ICLKO pin as the pacer source.  

    The timer will be reset and sample acquired when its value equals
    timer_period.  The equation for calculating timer_period is:

        timer_period = [100MHz / (sample frequency)] - 1

    The data will be returned in packets utilizing a bulk IN endpoint.
    The data will be in the format:

    lowchannel sample 0: lowchannel + 1 sample 0: ... :hichannel sample 0
    lowchannel sample 1: lowchannel + 1 sample 1: ... :hichannel sample 1
    ...
    lowchannel sample n: lowchannel + 1 sample n: ... :hichannel sample n

    Important: Since the analog input data is 18 bits wide, each
    input sample will be 32 bits wide to account for this.  This
    includes the digital channels, and zeros will be padded for
    unused bits.  The scan will not begin until the AInScanStart
    command is sent (and any trigger conditions are met.)  Data will
    be sent until reaching the specified count or an InScanStop
    command is sent.

    The packet_size parameter is used for low sampling rates to avoid
    delays in receiving the sampled data. The buffer will be sent,
    rather than waiting for the buffer to fill.  This mode should
    not be used for high sample rates in order to avoid data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate.  No messages will be sent until the trigger
    is detected.

    Pattern detection is used with the PatternDetectConfig command
    to set up a specified number of bits to watch, and then trigger
    when those bits reach the specified value.

    The retrigger mode option and the retrig_count parameter are only
    used if trigger is used.  This option will cause the trigger to
    be rearmed after retrig_count samples are acquired, with a total
    of count samples being returned from the entire scan.


    count:          the total number of scans to perform (0 for continuous scan)
    retrig_count:   the number of scans to perform fro each trigger in retrigger mode
    pacer_period:   pacer timer period value (0 for external clock)
    packet_size:    number of samples to transfer from defice FIFO at a time
    options:        bit field that controls various options
                bit 0: 1 = use external trigger
                bit 1: 1 = use Pattern Detection trigger
                bit 2: 1 = retrigger mode, 0 = normal trigger
                bit 3: 1 = Maintain counter value on scan start, 0 = clear counter value on scan start
                bit 4: Reserved 
                bit 5: Reserved 
                bit 6: Reserved 
                bit 7: Reserved 

    frequency: sampling frequency in Hz to set the pacer clock (scans per second)
    mode:      Controls various AIn Scan Modes
               bit 0: 1 = Continuous mode  0 = sample for "count" scans
               bit 1: 1 = return data after every read (used for low frequency scans)
               bit 2: 1 = force value of packet_size
               bit 3: 1 = return voltages as floats instead of raw reading
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    bytesPerScan = (self.lastElementAIn+1)*4

    if self.productID == self.USB_1808_PID and frequency > 50000: # 50k S/s throughput max
      frequency = 50000
    elif self.productID == self.USB_1808X_PID and frequency > 200000: # 200k S/s throughput max
      frequency = 200000
    elif frequency > 0 and frequency < .023: # .023 S/s throughput min
      frequency = .023

    self.frequency = frequency
    self.mode = mode

    if frequency == 0.0:
      pacer_period = 0     # use external pacer
    else:
      pacer_period = round((self.BASE_CLOCK / frequency) - 1)

    if count == 0:
      self.mode |= self.CONTINUOUS_READOUT
      self.bytesToRead = -1  # disable and sample forever
    else:
      self.bytesToRead = count*(self.lastElementAIn+1)*4  # total number of bytes to read

    if self.mode & self.FORCE_PACKET_SIZE:
      packet_size = self.packet_size
    elif self.mode & self.SINGLEIO:
      packet_size = self.lastElementAIn + 1
    elif self.mode & self.CONTINUOUS_READOUT:
      packet_size = int(((self.wMaxPacketSize//bytesPerScan) * bytesPerScan) // 4)
    elif self.bytesToRead <= self.wMaxPacketSize:
      packet_size = self.bytesToRead // 4  # causes timeout error for small sample size
    else:
      packet_size = self.wMaxPacketSize // 2
    self.packet_size = packet_size

    if self.mode & self.CONTINUOUS_READOUT:
      self.count = 0
    else:
      self.count = count

    self.retirg_count = retrig_count
    self.mode = mode & 0xff
    self.options = options 

    packet_size -= 1  # force to uint8_t size in range 0-255
    scanPacket = pack('IIIBB', count, retrig_count, pacer_period, packet_size, options)

    try:
      result = self.udev.controlWrite(request_type, self.AIN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 500)
    except:
      print('AInScanStart: Error in control write')

    self.status = self.Status()

  def AInScanRead(self):
    if self.mode & self.CONTINUOUS_READOUT or self.mode & self.SINGLEIO :
      nSamples = self.packet_size
    else:
      nSamples = self.count*(self.lastElementAIn+1)

    timeout = round(500 + 1000*nSamples/self.frequency)

    try:
      data =  list(unpack('I'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, 4*nSamples, timeout)))
    except usb1.USBErrorTimeout as exception:
      print('AInScanRead: Bulk Read error.  Timeout')
      return

    if len(data) != nSamples:
      raise ValueError('AInScanRead: error in number of samples transferred.')
      return len(data)
    
    if self.mode & self.VOLTAGE:
      for i in range(len(data)):
        chan = self.scanQueueAIn[i%(self.lastElementAIn+1)]
        if chan < 8:  # Only convert A/D values
          gain = self.AInConfig[chan] & 0x3
          data[i] = data[i]*self.table_AIn[chan][gain].slope + self.table_AIn[chan][gain].intercept
          if data[i] > 0x3ffff:
            data[i] = 0x3ffff
          elif data[i] < 0.0:
            data[i] = 0x0
          else:
            data[i] = int(round(data[i]))
          data[i] = self.volts(gain, data[i])

    if self.bytesToRead > len(data)*4:
      self.bytesToRead -= len(data)*4
    elif self.bytesToRead > 0 and self.bytesToRead < len(data)*4:  # all done
      self.AInScanStop()
      self.AInScanClearFIFO()
      self.status = self.Status()
      return data

    if self.mode & self.CONTINUOUS_READOUT:
      return data

    # if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
    if nSamples*4%self.wMaxPacketSize == 0:
      try:
       dummy = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, 1, 100)
      except:
        pass
     
    self.status = self.Status()
    if self.status & self.AIN_SCAN_OVERRUN:
      self.AInScanStop()
      self.AInScanFIFO()
      raise ValueError('AInScanRead: Scan overrun.')
      return

    return data

  def AInScanStop(self):
    """
    This command stops the analog input scan (if running).
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_STOP
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def AInScanConfigR(self):
    """
    This command reads/writes the input scan queue to the FPGA.  The max
    queue is 13 elements: 8 AIn, 2 Counters, 2 Encoders, 1 DIO.

    Each element of the scanQueueAIn array corresponds to the element in
    the scan queue of the FPGA.  The data will determine which channel
    is read as follows:

    Analog Inputs:   0 - 7
    DIO:             8
    Counter 0:       9
    Counter 1:       10
    Encoder 0:       11
    Encoder 1:       12

    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = self.udev.controlRead(request_type, self.AIN_SCAN_CONFIG, wValue, wIndex, 13, self.HS_DELAY)
    for i in range(len(value)):
      self.scanQueueAIn[i] = value[i]
    return value

  def AInScanConfigW(self, entry, value, lastElement=False):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CONFIG
    wValue = 0

    if entry < 0 or entry > 12:
      raise ValueError('AInScanConfigW: Exceed depth of queue')
      return

    if value < 0 or value > 12:
      raise ValueError('AInScanConfigW: Unknown entry for ScanQueue')
      return

    if lastElement == True:
      self.lastElementAIn = entry
      wIndex = entry               # This element is the last entry in the queue
    else:
      self.lastElementAIn = 0
      wIndex = 0

    if self.Status() & self.AIN_SCAN_RUNNING:
      self.AInScanStop()

    self.scanQueueAIn[entry] = value

    try:
      result = self.udev.controlWrite(request_type, request, wValue, wIndex, self.scanQueueAIn, self.HS_DELAY)
    except:
      print('AInScanConfigW: error in control write. result =', result)
      return
    
  def AInScanClearFIFO(self):
    """
    This command clears the analog input firmware buffer.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CLEAR_FIFO
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def AInBulkFlush(self, count):
    """
    This command flushes the input Bulk pipe a number of times.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_BULK_FLUSH
    wValue = count
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)


  #############################################
  #           Analog Output                   #
  #############################################

  def AOut(self, channel, voltage):
    """
    This command asynchronously writes out data to the specified DAC
    channels.  The values are 16-bit unsigned numbers.  Both read and
    write will result in a control pipe stall if an output scan is
    running.

    channel: the channel number to update (0-1)
    value:   the value for the analog output channel (0-65535)

    The equation for the output voltage is:

              (value - 2^15)
    V_out =  ---------------- * V_ref
                 2^15

    where value is the value written to the device.  V_ref = 10V
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wIndex = channel & 0x1

    if channel > 2:
      raise ValueError('AOut: channel out of range')
      return

    value = voltage/10.*32768. + 32768.
    value = value*self.table_AOut[channel].slope + self.table_AOut[channel].intercept

    if int(value) > 0xffff:
      wValue = 0xffff
    elif value < 0.0:
      wValue = 0x0
    else:
      wValue = int(round(value))
          
    result = self.udev.controlWrite(request_type, self.AOUT, wValue, wIndex, [0x0], timeout = 100)

  def AOutScanConfig(self, entry, value, lastElement=-1):
    """
    This command writes the output scan queue to the FPGA.  The max
    queue depth is 3 elements: 2 AOUT, 1 DIO

      LastChan:         This is the last channel that will be read in the queue
      ScanQueueAOut[3]: The value in this array determines which channel is read
                        at that point in the queue as follows:
                        AOUT0 = 0
                        AOUT1 = 1
                        DIO   = 2

    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wIndex = 0
    wValue = 0

    self.scanQueueAOut[entry] = value
    if lastElement != -1:
      self.lastElementAOut = lastElement
      wIndex = lastElement
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_CONFIG, wValue, wIndex, self.scanQueueAOut, timeout = 200)

  def AOutScanConfigR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0

    value = unpack('BBB',self.udev.controlRead(request_type, self.AOUT_SCAN_CONFIG, wValue, wIndex, 3, self.HS_DELAY))
    return list(value)

  def AOutScanStart(self, count, retrig_count, frequency, options):
    """
    This command starts the analog output channel scan.  This command
    will result in a bus stall if an AOutScan is currently running.

    count:        the total number of scans to perform (0 = continuous mode)
    retrig_count: the number of scans to perform for each trigger in
                  retrigger mode
    frequency:    pacer frequency (0 for AO_CLK_IN) 0.23 Hz < frequency < 125 kHz
    options:      bit 0: 1 = use external trigger
                  bit 1: 1 = use Pattern Detection trigger
                  bit 2: 1 = retirgger mode, 0 = normal trigger
                  bit 3: reserved
                  bit 4: reserved
                  bit 5: reserved
		  bit 6: reserved
		  bit 7: reserved
    Notes:
		  
    The output scan operates with the host continuously transferring
    data for the outputs until the end of the scan.  If the "count"
    parameter is 0, the scan will run until the AOutScanStop command
    is issued by the host; if it is nonzero, the scan will stop
    automatically after the specified number of scans have been
    output.  The channels in the scan are selected in the options bit
    field.  Scan refer to the number of updates to the channels (if
    both channels are used, one scan is an update to both channels).

    The time base is controlled by an internal 32-bit timer running at
    a base rate of 100 MHz.  The timer is controlled by pacer_period.  
    The equation for calculating pacer_period is:

        pacer_period = (100 MHz / sample_frequency) - 1

    The same time base is used for all channels when the scan involved
    multiple channels.  The output data is to be sent using the bulk
    out endpoint.  The data must be in the format:

      low channel sample 0 : [high channel sample 0]
      low channel sample 1 : [high channel sample 1]
      ...
      low channel sample n : [high channel sample n]

    The output is written to an internal FIFO.  The bulk endpoint data
    is only accepted if there is room in the FIFO.  Output data may be
    sent to the FIFO before the start of the scan, and the FIFO is
    cleared when the AOutScanClearFIFO command is received.  The scan
    will not begin until the AOutScanStart command is sent (and outupt
    data is in the FIFO).  Data will be output until reaching the
    specified number of scans (in single execution mode) or an
    AOutScanStop command is sent.
    """

    if frequency < 0.:
      raise ValueError('AOutScanStart: frequency must be positive')
      return
    elif frequency > 0 and frequency < 0.23:
      raise ValueError('AOutScanStart: frequency must be greater than .23 kHz')
      return
    elif frequency > 125000 and self.productID == self.USB_1808_PID:
      raise ValueError('AOutScanStart: frequency must be less than 125 kHz')
      return
    elif frequency > 500000 and self.productID == self.USB_1808X_PID:
      raise ValueError('AOutScanStart: frequency must be less than 500 kHz')
      return

    if frequency == 0:
      pacer_period = 0  # use external clock pin 22 OCLKI      
    else:
      pacer_period = round((self.BASE_CLOCK / frequency) - 1)

    self.frequency_AOut = frequency
    self.options_AOut = options
    self.count_AOut = count
    if count == 0:
      self.continuous_mode_AOUT = True
    else:
      self.continuous_mode_AOUT = False
    self.retrig_count = retrig_count

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    scanPacket = pack('IIIB', count, retrig_count, pacer_period, options)
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

  def AOutScanWrite(self, data):
    # data is a list of unsigned 16 bit numbers
    value = [0]*len(data)*2
    timeout = int(800 + 1000*len(data)/self.frequency_AOut)

    for i in range(len(data)):
      value[2*i] = data[i] & 0xff
      value[2*i+1] = (data[i] >> 8) & 0xff
    try:
      result = self.udev.bulkWrite(2, value, timeout)
    except:
      print('AOutScanWrite: error in bulkWrite')
      return

    try:
      result = self.udev.bulkWrite(2, [0x0],  200)
    except:
      print('AOutScanWrite: error in bulkWrite 2')

    # if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
    if self.continuous_mode_AOUT == False and len(data) % self.wMaxPacketSize == 0:
      dummy = self.udev.bulkWrite(2, 0x1, timeout)
    
  def AOutScanStop(self):
    """
    This command stops the analog output scan (if running).
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_STOP, wValue, wIndex, [0x0], timeout = 100)

  def AOutScanClearFIFO(self):
    """
    The command clears the output firmware buffer.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_CLEAR_FIFO, wValue, wIndex, [0x0], timeout = 100)

  def CounterSet(self, counter, count):
    """
    This command reads or sets the value of the 32-bit counters.  Counter 0 and 1
    are event coutner, while Counter 2 and 3 are Encoder 0 and 1, respectively.

    counter: the counter to set (0-3)
    count:   the 32 bit value to set the counter
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER
    wValue = 0x0
    wIndex = counter & 0xf

    if counter > self.NCOUNTER:
      raise ValueError('CounterSet: counter out of range.')
      return

    count = pack('I', count)
    self.udev.controlWrite(request_type, request, wValue, wIndex, count, self.HS_DELAY)

  def Counter(self, counter):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter & 0xf

    if counter > self.NCOUNTER:
      raise ValueError('Counter: counter out of range.')
      return

    value, = unpack('I',self.udev.controlRead(request_type, self.COUNTER, wValue, wIndex, 4, self.HS_DELAY))
    return value
  
  def CounterOptionsR(self, counter):
    """
    This command reads or sets the options of the counter.
    counter: the counter to set (0-3)

    options:    The options for this counter's mode and will differ depending on the coutner type.
      Counter:
          bit(0):   1 = Clear on Read,  0 = Read has no effect
          bit(1):   1 = No recycle mode (counter stops at 2^32 or 0, unless Range Limit is enabled)
                    0 = counter rolls over to a minimum (or maximum) and continues counting.
          bit(2):   1 = Count down,  0 = Count up
          bit(3):   1 = Range Limit on (use max and min limits) 0 = 64-bit counter (max = 2^32, min = 0)
          bit(4):   1 = Count on falling edge,  0 = Count on rising edge
          bit(5-7): Reserved

      Encoder:
         bit 0-1: Encoder Type:
                  0 = X1
                  1 = X2
                  2 = X4
         bit 2: Clear on Z: 1 = clear when Z goes high, 0 = do not clear when Z goes high
         bit 3: Latch on Z: 1 = Counter will be latched when Z goes high, 
                            0 = Counter will be latched when asynchronously read or on a pacer clock in a scan.
         bit 4: 1 = No recycle mode (counter stops at 2^32 or 0, unless Range Limit is enabled)
                0 = counter rolls over to minimum (or max) and continue counting.
         bit 5: 1 = Range Limit on (use max and min limits)
                0 = 32-bit counter (max = 2^32, 0 = min)
         bits 6-7: Reserved

    """
    
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOptionsR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter & 0xf
    options ,= unpack('B',self.udev.controlRead(request_type, self.COUNTER_OPTIONS, wValue, wIndex, 1, self.HS_DELAY))
    self.counterParameters[counter].counterOptions = options
    return options
    
  def CounterOptionsW(self, counter, options):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOptionsW: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_OPTIONS
    wValue = options
    wIndex = counter
    self.counterParameters[counter].counterOptions = options
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def CounterLimitR(self, counter, index):
    """
    This command reads or sets the counter's limit values.
      index: the index of the value to set (0-1)
          0 =  Minimum Limit Value, 1 = Maximum Limit Value 
      value: when the counter reaches this value, rolls over or stops depending on the options.
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterLimitValuesR: counter value too large.')
      return
    if index > 2:
      raise ValueError("CounterLimitValuesR: index must be '0' or '1'.")
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = index
    wIndex = counter & 0xf
    value ,= unpack('I',self.udev.controlRead(request_type, self.COUNTER_LIMIT_VALUES, wValue, wIndex, 4, self.HS_DELAY))
    if index == 0:
      self.counterParameters[counter].limitValue0 = value
    else:
      self.counterParameters[counter].limitValue1 = value
    return value
    
  def CounterLimitValuesW(self, counter, index, value):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterLimitValuesW: counter value too large.')
      return
    if index > 2:
      raise ValueError("CounterLimitValuesW: index must be '0' or '1'.")
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_LIMIT_VALUES
    wValue = index
    wIndex = counter & 0xf
    value = pack('I',value)

    self.udev.controlWrite(request_type, request, wValue, wIndex, value, self.HS_DELAY)

  def CounterModeR(self, counter):
    """
    This command reads or sets the mode of the counter.  This does not function on
    the Encoder counters.

    Mode:
      bits(0-1): type of mode
                Totalize   = 0
                Period     = 1
                Pulsewidth = 2
                Timing     = 3
      bits(2-3): resolution of Period Mode
                Period Mode x1    = 0
                Period Mode x10   = 1
                Period Mode x100  = 2
                Period Mode x1000 = 3
      bits(4-5): Tick size (fundamental unit of time for period, pulsewidth and timing modes)
		0 = 20.00 ns
                1 = 200 ns
		2 = 2000 ns
		3 = 20000 ns
      bits(6-7): Reserved 		
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterModeR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter & 0xf
    mode ,= unpack('B',self.udev.controlRead(request_type, self.COUNTER_MODE, wValue, wIndex, 2, self.HS_DELAY))
    self.counterParameters[counter].modeOpitons = mode
    return mode

  def CounterModeW(self, counter, mode):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterModeW: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_MODE
    wValue = mode
    wIndex = counter
    self.counterParameters[counter].modeOptions = mode
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def CounterParametersR(self, counter):
    """
    This command reads or sets the mode and options of a counter.
    This only works on the Options on the Encoder counters.

    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterParamsR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter & 0xf
    data = unpack('BB',self.udev.controlRead(request_type, self.COUNTER_PARAMETERS, wValue, wIndex, 2, self.HS_DELAY))
    self.counterParameters[counter].counter = counter  
    self.counterParameters[counter].modeOptions = data[0]      # the current counter's mode
    self.counterParameters[counter].counterOptions = data[1]   # the current counter's mode's options
    return

  def CounterParametersW(self, counter, counterMode, counterOptions):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterParamsR: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_PARAMETERS
    wValue = 0x0
    wIndex = counter & 0xf
    self.counterParameters[counter].modeOptions = counterMode
    self.counterParameters[counter].counterOptions  = counterOptions
    s_buffer = [counterMode, counterOptions]
    
    self.udev.controlWrite(request_type, request, wValue, wIndex, s_buffer, self.HS_DELAY)

  ##########################################
  #             Timer Commands             #
  ##########################################
  def TimerControlR(self, timer):
    """
    This command reads or writes the timer control register.  Timer output is 0 or 5V.

    timer: the timer selected: (0-1)
    control: the new control register value:
      bit 0: 1 = enable timer, 0 = disable timer (This needs to be a 0 when bit 4 is a 1)
      bit 1: 1 = timer running, 0 = timer stopped.
      bit 2: 1 = timer inverted output (active low), 0 = timer normal output (active high (5V))
      bit 3: Reserved
      bit 4: 1 = Timer will begin output when the OTRIG pin has triggered, 0 = Normal timer operation
      bit 5: Reserved
      bit 6: 1 = When bit 4 is equal to 1, Timer will continue to output on every OTRIG it receives
             0 = When bit 4 is equal to 1, Timer will output only on the first OTRIG received.
      bit 7: Reserved
    """
    if timer > self.NTIMER:
      raise ValueError('TimerControlR: timer out of range')
      return
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = timer
    data, = self.udev.controlRead(request_type, self.TIMER_CONTROL, wValue, wIndex, 1, self.HS_DELAY)
    self.timerParameters[timer].control = data
    return data

  def TimerControlW(self, timer, control):
    if timer > self.NTIMER:
      raise ValueError('TimerControlW: timer out of range')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_CONTROL
    wValue = control
    wIndex = timer
    self.timerParameters[timer].control = control
    try:
      self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)
    except:
      raise

  def TimerParametersR(self, timer):
    """
    This command reads or writes all of a given timer's parameters in one call.

    The timer is based on a 100MHz input clock and has a 32-bit
    period register.  The frequency of the output is set to:

        frequency = 100MHz / (period + 1)

    Note that the value for pulseWidth should always be smaller than
    the value for the period register or you may get unexpected
    results.  This results in a minimum allowable value for period of
    1, which sets the maximm freqauency to 100MHz/2 (50MHz).

    The timer has a 32-bit pulse width register.  The width of the ouput pulse isset to 

        pulse width = (pulseWidth + 1) / 100MHz

    Noe tht the value for pulseWidth should always be smaller than
    the value for the period register or you may get unexpted
    results.

    The number of output pulses can be controlled with the Count
    register.  Setting this register to 0 will result in pulses being
    generated until the timer is disabled.  Setting it to a non-zero
    value will result in the specified number of pulses being
    generated then the output will go low until the timer is disabled.

    The Delay register is the amount of time to delay before starting
    the timer output after enabling the output.  The value specifies
    the number of 100MHz clock pulses to delay.  This value may not be
    written while the timer output is enabled.

    Note: The TimerParameterR method currently does not return the
    correct register values in the firmware.  Return a tuple
    of (frequency [Hz], duty cycle, count and delay (in ms))
    """
    if timer >= self.NTIMER:
      raise ValueError('TimerParametersW: timer out of range')
      return

    frequency = self.BASE_CLOCK / (self.timerParameters[timer].period + 1)
    duty_cycle = self.timerParameters[timer].pulseWidth / self.timerParameters[timer].period
    count =  self.timerParameters[timer].count
    delay =  (self.timerParameters[timer].delay * 1000.) / self.BASE_CLOCK # delay in ms

    return (frequency, duty_cycle, count, delay)

  def TimerParametersW(self, timer, frequency, dutyCycle, count, delay):
    if timer > self.NTIMER:
      raise ValueError('TimerParametersW: timer out of range')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_PARAMETERS
    wValue = 0x0
    wIndex = timer

    period = round(self.BASE_CLOCK/frequency - 1)
    pulseWidth = round(period * dutyCycle)

    if period < 1:
      raise ValueError('TimerParametersW: period less than 1.')
      period = 1         # set to lowest value
 
    self.timerParameters[timer].period = int(period)
    self.timerParameters[timer].pulseWidth = int(pulseWidth)
    self.timerParameters[timer].count = int(count)
    self.timerParameters[timer].delay = int(delay)
    self.timerParameters[timer].frequency = frequency
        
    barray = pack('IIII', self.timerParameters[timer].period, self.timerParameters[timer].pulseWidth, \
                  self.timerParameters[timer].count, self.timerParameters[timer].delay)
    self.udev.controlWrite(request_type, request, wValue, wIndex, barray, self.HS_DELAY)


  ##########################################
  #           Memory Commands              #
  ##########################################

  def MemoryR(self, length):
    """
    This command reads or writes data from the EEPROM memory.  The
    read will begin at the current address, which may be set with
    MemAddress.  The address will automatically increment during a
    read or write but stay within the range allowed for the EEPROM.
    The amount of data to be written or read is specified in wLength.

    The range from 0x0000 to 0x3FFF is used for storing the
    microcontroller firmware and is write-protected during normal
    operation.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    data = self.udev.controlRead(request_type, self.MEMORY, wValue, wIndex, length, self.HS_DELAY*length)
    return data

  def MemoryW(self, data):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.MEMORY
    wValue = 0
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, data, self.HS_DELAY)

  def MemAddressR(self):
    """
    This command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas.  The
    memory map for this device is

    Address                            Description
    =============               ============================
    0x0000-0x6FF7               Microcontroller firmware (write protected)
    0x6FF8-0x6FFF               Serial Number
    0x7000-0x7FFF               User data (Calibration Coefficients)

    The firmware area is protected by a separate command so is not typically
    write-enabled.  The calibration area is unlocked by writing the value 0xAA55
    to address 0x8000.  The area will remain unlocked until the device is reset
    or a value other than 0xAA55 is written to address 0x8000.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    address = self.udev.controlRead(request_type, self.MEM_ADDRESS, wValue, wIndex, 2, self.HS_DELAY)
    return address[0] + (address[1] << 8)
    
  def MemAddressW(self, address):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.MEM_ADDRESS
    wValue = 0
    wIndex = 0
    barray = [address & 0xff, (address >> 8) & 0xff]
    self.udev.controlWrite(request_type, request, wValue, wIndex, barray, self.HS_DELAY)

  def MemWriteEnable(self):
    """
    This command enables writes to the EEPROM memory in the range
    0x0000-0x6FFF.  This command is only to be used when updating the
    microcontroller firmware.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.MEM_WRITE_ENABLE
    wValue = 0
    wIndex = 0
    unlock_code = 0xad
    self.udev.controlWrite(request_type, request, wValue, wIndex, [unlock_code], self.HS_DELAY)


  ##########################################
  #        Miscellaneous Commands          #
  ##########################################

  def Status(self):
    """
    This command retrieves the status of the device.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= unpack('H',self.udev.controlRead(request_type, self.STATUS, wValue, wIndex, 2, self.HS_DELAY))
    return value

  def BlinkLED(self, count):
    """
    This command will blink the device LED "count" number of times
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.BLINK_LED
    wValue = 0
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [count], self.HS_DELAY)

  def Reset(self):
    """
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.RESET
    wValue = 0
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def TriggerConfig(self, options):
    """
    This function configures the Scan trigger.  Once the trigger is
    received, the Scan will proceed as configured.  The "use
    trigger" option must be used in the ScanStart command to
    utilize this feature.

      options:     bit 0: trigger mode (0 = level,  1 = edge)
                   bit 1: trigger polarity (0 = low / falling, 1 = high / rising)
                   bits 2-7: reserved
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TRIGGER_CONFIG
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [options], self.HS_DELAY)

  def PatternDetectConfigR(self):
    """
    This function configures the Pattern Detection trigger.  Once the
    trigger is received, the Scan will proceed as configured.  The
    "use Patern Detection trigger" option must be used in the
    AInScanStart command to utilize this feature.

    value:   the pattern on which to trigger
    mask:    these bits will mask the inputs such taht only bits set to 1 here wil be compared to the pattern.
    options: bit field that controls various options
             bit 0:    Reserved
             bit 1-2:  00 = Equal to Pattern
                       01 = Not equal to Pattern
                       10 = Greater than Pattern's numeric value
                       11 = Less than Pattern's numeric value
             bits 3-7: eserved
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = unpack('BBB',self.udev.controlRead(request_type, self.PATTERN_DETECT_CONFIG, wValue, wIndex, 3, self.HS_DELAY))
    return (value[0], value[1], value[2])

  def PatternDetectConfigW(self, value, mask, options):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    data = pack('BBB', value, mask, options)
    result = self.udev.controlWrite(request_type, self.PATTERN_DETECT_CONFIG, wValue, wIndex, data, self.HS_DELAY)

  def GetSerialNumber(self):

    """
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001").
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = self.udev.controlRead(request_type, self.SERIAL, wValue, wIndex, 8, self.HS_DELAY)
    return value.decode()

  def WriteSerialNumber(self, serial):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SERIAL
    wValue = 0
    wIndex = 0
    barray = bytearray(8)
    for i in range(8):
      barray[i] = ord(serial[i])
    self.udev.controlWrite(request_type, request, wValue, wIndex, barray, self.HS_DELAY)

  ##########################################
  #            FPGA Commands               #
  ##########################################
  
  def FPGAConfig(self):
    """
    This command puts the device into FPGA configuration update mode,
    which allows downloading the configuration for the FPGA.  The
    unlock code must be correct as a further safely device.  If the
    device is not in FPGA config mode, then the FPGAData command will
    result in a control pipe stall.

    Use the Status command to determine if the FPGA needs to be
    configured.  If so, use this command to enter configuration mode.
    Open the .rbf file containing the FPGA configuration and stream
    the data to the device using FPGAData.  After the FPGA is
    configured, then the DAQ commands will work.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.FPGA_CONFIG
    unlock_code = 0xad
    wValue = 0
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [unlock_code], self.HS_DELAY)

  def FPGAData(self, data):
    """
    This command writes the FPGA configuration data to the device.  This
    command is not accepted unless the device is in FPGA config mode.  The
    number of bytes to be written must be specified in wLength.

    data: max length is 64 bytes
    """
    if len(data) > 64:
      raise ValueError('FPGAData: max length is 64 bytes.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.FPGA_DATA
    wValue = 0
    wIndex = 0
    self.udev.controlWrite(request_type, request, wValue, wIndex, data, self.HS_DELAY)

  def FPGAVersion(self):
    """
    This command reads the FPGA version.  The version is in
    hexadecimal BCD, i.e. 0x0102 is version 01.02.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    version ,= unpack('H',self.udev.controlRead(request_type, self.FPGA_VERSION, wValue, wIndex, 2, self.HS_DELAY))
    return "{0:02x}.{1:02x}".format((version>>8)&0xff, version&0xff)

  def printStatus(self):
    status = self.Status()
    print('**** USB-1608G Status ****')
    if status & self.AIN_SCAN_RUNNING:
      print('  Analog Input scan running.')
    if status & self.AIN_SCAN_OVERRUN:
      print('  Analog Input scan overrun.')
    if status & self.AOUT_SCAN_RUNNING:
      print('  Analog Output scan running.')
    if status & self.AOUT_SCAN_UNDERRUN:
      print('  Analog Output scan underrun.')
    if status & self.AIN_SCAN_DONE:
      print(' Analog Input scan done.')
    if status & self.AOUT_SCAN_DONE:
      print(' Analog Output scan done.')
    if status & self.FPGA_CONFIGURED:
      print('  FPGA is configured.')
    if status & self.FPGA_CONFIG_MODE:
      print('  FPGA in configuration mode.')

  def volts(self, gain, value):
    # converts 18 bit unsigned int value to volts
    if gain == self.BP_10V:
      volt = (value - 131072) * 10. / 131072.
    elif gain == self.BP_5V:
      volt = (value - 131072) * 5. / 131072
    elif gain == self.UP_10V:
      volt = value * 10. / 262143.
    elif gain == self.UP_5V:
      volt = value * 5. / 262143.
    else:
      raise ValueError('volts: Unknown range.')
      return

    return volt

################################################################################################################

class usb_1808(usb1808):
  def __init__(self, serial=None):
    self.productID = self.USB_1808_PID   #usb-1808
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1808 not found")
      return
    usb1808.__init__(self)

class usb_1808X(usb1808):
  def __init__(self, serial=None):
    self.productID = self.USB_1808X_PID   #usb-1808X
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1808X not found")
      return
    usb1808.__init__(self)


