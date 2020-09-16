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

class usb1608G(mccUSB):

  # USB PIDs for family of devices
  USB_1608G_PID = 0x0134
  USB_1608GX_PID = 0x0135
  USB_1608GX_2AO_PID = 0x0136

  # Gain Ranges
  BP_10V   = 0x0       # +/- 10.0 V
  BP_5V    = 0x1       # +/- 5.00 V
  BP_2V    = 0x2       # +/- 2.00 V
  BP_1V    = 0x3       # +/- 1.25 V

  # Analog input mode
  SINGLE_ENDED = 1  # channels 0 - 15
  DIFFERENTIAL = 0  # channels 0 - 7
  LAST_CHANNEL = 0x80

  # Status Bits
  AIN_SCAN_RUNNING   = 0x2   # AIn pacer running
  AIN_SCAN_OVERRUN   = 0x4   # AIn scan overrun
  AOUT_SCAN_RUNNING  = 0x8   # AOut scan running
  AOUT_SCAN_UNDERRUN = 0x10  # AOut scan underrrun
  AIN_SCAN_DONE      = 0x20  # AIn scan done
  AOUT_SCAN_DONE     = 0x40  # AOut scan done
  FPGA_CONFIGURED    = 0x100 # FPGA is configured
  FPGA_CONFIG_MODE   = 0x200 # FPGA config mode

  NCHAN              = 16    # max number of A/D channels in the device (single_ended)
  NGAIN              = 4     # max number of gain levels
  MAX_PACKET_SIZE_HS = 512   # max packet size for HS device
  MAX_PACKET_SIZE_FS = 64    # max packet size for HS device
  BASE_CLOCK         = 64.E6 # base frequency of the board
  COUNTER0           = 0
  COUNTER1           = 1

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
  AIN_SCAN_START       = 0x12  # Start analog input scan
  AIN_SCAN_STOP        = 0x13  # Stop analog input scan
  AIN_SCAN_CONFIG      = 0x14  # Read/Write analog input configuration
  AIN_SCAN_CLEAR_FIFO  = 0x15  # Clear the analog input scan FIFO

  # Counter/Timer Commands
  COUNTER              = 0x20  # Read/reset counter
  TIMER_CONTROL        = 0x28  # Read/Write timer control register
  TIMER_PERIOD         = 0x29  # Read/Write timer period register
  TIMER_PULSE_WIDTH    = 0x2A  # Read/Write timer pulse width registers
  TIMER_COUNT          = 0x2B  # Read/Write timer count registers
  TIMER_START_DELAY    = 0x2C  # Read/Write timer start delay registers
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
  CAL_CONFIG           = 0x44  # Calibration voltage configuration
  TEMPERATURE          = 0x45  # Read internal temperature
  SERIAL               = 0x48  # Read/Write USB Serial Number

  # FPGA Configuration Commands
  FPGA_CONFIG          = 0x50 # Start FPGA configuration
  FPGA_DATA            = 0x51 # Write FPGA configuration data
  FPGA_VERSION         = 0x52 # Read FPGA version

  HS_DELAY = 2000

  def __init__(self):
    self.status = 0                       # status of the device
    self.samplesToRead = -1               # number of bytes left to read from a scan
    self.scanList = bytearray(self.NCHAN) # depth of scan queue is 15
    self.lastElement = 0                  # last element of the scan list
    self.count = 0
    self.retrig_count = 0
    self.options = 0
    self.frequency = 0.0                  # frequency of scan (0 for external clock)
    self.packet_size = 512                # number of samples to return from FIFO
    self.mode = 0                         # mode bits:
                                          # bit 0:   0 = counting mode,  1 = CONTINUOUS_READOUT
                                          # bit 1:   1 = SINGLEIO
                                          # bit 2:   1 = use packet size in self.packet_size
                                          # bit 3:   1 = convert raw readings to voltages
                
    # Configure the FPGA
    if not (self.Status() & self.FPGA_CONFIGURED) :
      # load the FPGA data into memory
      from usb_1608G_rbf import FPGA_V2_data
      print("Configuring FPGA.  This may take a while ...")
      self.FPGAConfig()
      if self.Status() & self.FPGA_CONFIG_MODE:
        for i in range(0, len(FPGA_V2_data) - len(FPGA_V2_data)%64, 64) :
          self.FPGAData(FPGA_V2_data[i:i+64])
        i += 64
        if len(FPGA_V2_data) % 64 :
          self.FPGAData(FPGA_V2_data[i:i+len(FPGA_V2_data)%64])
        if not self.Status() & self.FPGA_CONFIGURED:
          print("Error: FPGA for the USB-1608G is not configured.  status = ", hex(self.Status()))
          return
      else:
        print("Error: could not put USB-1608G into FPGA Config Mode.  status = ", hex(self.Status()))
        return
    else:
      print("USB-1608G FPGA configured.")

    if sys.platform.startswith('linux'):
      if self.udev.kernelDriverActive(0):
        self.udev.detachKernelDriver(0)
        self.udev.resetDevice()

    # claim all the needed interfaces for AInScan
    self.udev.claimInterface(0)

    # Find the maxPacketSize for bulk transfers
    self.wMaxPacketSize = self.getMaxPacketSize(libusb1.LIBUSB_ENDPOINT_IN | 0x6)  #EP IN 6

    # Set up the Timers
    self.timerParameters = TimerParameters()
    self.timerParameters.timer = 0        # only 1 timer
    self.TimerParamsW(1000., 0.5, 0, 0)

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    #   self.table_AIn[gain]    0 <= gain < 4
    #
    self.table_AIn = [table(), table(), table(), table()]
    address = 0x7000
    self.MemAddressW(address)
    for gain in range(self.NGAIN):
      self.table_AIn[gain].slope, = unpack('f', self.MemoryR(4))
      self.table_AIn[gain].intercept, = unpack('f', self.MemoryR(4))

  def CalDate(self):
    """
    Get the manufacturers calibration data (timestamp) from the
    Calibration memory.

    Note: The calibration date is stored in the EEPROM
    starting at address 0x7098.  The six date elements
    (year, month, day, hour, minute, second) are stored
    as big endian unsigned short.
    """

    address = 0x7098   # staring address of the MFG calibration date
    self.MemAddressW(address)

    data = unpack('>HHHHHH', self.MemoryR(12))  # 12 bytes big endian
    mdate = datetime(data[0], data[1], data[2], data[3], data[4], data[5])
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
    value ,= self.udev.controlRead(request_type, self.DTRISTATE, wValue, wIndex, 2, self.HS_DELAY)
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
    value ,= self.udev.controlRead(request_type, self.DPORT, wValue, wIndex, 2, self.HS_DELAY)
    return value

  def DLatchR(self):
    """
    This command reads the digital port latch register
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= self.udev.controlRead(request_type, self.DLATCH, wValue, wIndex, 2, self.HS_DELAY)
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

  def AIn(self, channel, gain, voltage = False):
    """
    This command reads the value of an analog input channel.  This
    command will result in a bus stall if an AInScan is currently
    running.  

     channel: the channel to read (0-15)
     value:   16 bits of data, right justified.
     voltage: True = return voltage, False = return raw reading
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue =  channel
    wIndex = 0
    
    if gain < 0 or gain >= self.NGAIN:
      raise ValueError('AIn: Error in gain value')
      return
    
    data ,= unpack('H',self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 2, timeout = 200))

    if data > 0xfffd:
      raise ValueError("DAC is saturated at +FS")
    elif data < 0x60:
      raise ValueError("DAC is saturated at -FS")
    else:
      data = round(float(data)*self.table_AIn[gain].slope + self.table_AIn[gain].intercept)

    if voltage == True:
      data = self.volts(gain, value)

    return data
    

  def AInScanStart(self, count, retrig_count, frequency, options, mode=0):
    """
    This command starts the analog input channel scan.  The gain
    ranges that are currently set on the desired channels will be
    used (these may be changed with AInConfig) This command will
    result in a bus stall if an AInScan is currently running.

    count:        the total number of scans to perform (0 for continuous scan)
    retrig_count: the number of scans to perform for each trigger in retrigger mode
    pacer_period: pacer timer period value (0 for AI_CLK_IN)
    frequency:    pacer frequency in Hz
    packet_size:  number of samples to transfer at a time
    options:      bit field that controls various options
                   bit 0: 1 = burst mode, 0 = normal mode
                   bit 1: Reserved
                   bit 2: Reserved
                   bit 3: 1 = use trigger  0 = no trigger
                   bit 4: Reserved
                   bit 5: Reserved
                   bit 6: 1 = retrigger mode, 0 = normal trigger
                   bit 7: Reserved
    mode:  mode bits:
           bit 0:  0 = counting mode,  1 = CONTINUOUS_READOUT
           bit 1:  1 = SINGLEIO
           bit 2:  1 = use packet size passed usbDevice1808->packet_size
           bit 3:  1 = convert to voltages  

    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 64 MHz.  The timer is controlled by
    pacer_period. If burst mode is specified, then this value is the
    period of the scan and the A/D is clocked at this maximum rate
    (500 kHz) for each channel in the scan.  If burst mode is not
    specified, then this value is the period of the A/D readings.  A
    pulse will be output at the AI_CLK_OUT pin at every pacer_period
    interval regardless of the mode.

    If pacer_period is set to 0, the device does not generate an A/D
    clock.  It uses the AI_CLK_IN pin as the pacer source.  Burst
    mode operates in the same fashion: if specified, the scan starts
    on every rising edge of AI_CLK_IN and the A/D is clocked at 500 kHz
    for the number of channels in the scan; if not specified, the A/D
    is clocked on every rising edge of AI_CLK_IN.

    The timer will be reset and sample acquired when its value equals
    timer_period.  The equation for calculating timer_period is:

           timer_period = [64MHz / (sample frequency)] - 1

    The data will be returned in packets utilizing a bulk IN endpoint.
    The data will be in the format:

      lowchannel sample 0: lowchannel + 1 sample 0: ... :hichannel sample 0
      lowchannel sample 1: lowchannel + 1 sample 1: ... :hichannel sample 1
      ...
      lowchannel sample n: lowchannel + 1 sample n: ... :hichannel sample n

    The packet_size parameter is used for low sampling rates to avoid
    delays in receiving the sampled data. The buffer will be sent,
    rather than waiting for the buffer to fill.  This mode should
    not be used for high sample rates in order to avoid data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate.  No messages will be sent until the trigger
    is detected.

    The retrigger mode option and the retrig_count parameter are only
    used if trigger is used.  This option will cause the trigger to
    be rearmed after retrig_count samples are acquired, with a total
    of count samples being returned from the entire scan.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    bytesPerScan = (self.lastElement+1)*2
    self.mode = mode

    if self.productID == self.USB_1608G_PID and frequency > 250000: # 250k S/s throughput
      frequency = 250000.
    elif frequency > 500000: # 500 kS/s throughput
      frequency = 500000.

    if frequency == 0.0:
      pacer_period = 0     # use external pacer
    else:
      pacer_period = round((self.BASE_CLOCK / frequency) - 1)

    if count == 0:
      self.mode |= self.CONTINUOUS_READOUT
      self.bytesToRead = -1                    # disable and sample forever
    else:
      self.bytesToRead = count*bytesPerScan*2  # total number of bytes to read

    if self.mode & self.FORCE_PACKET_SIZE:
      packet_size = self.packet_size
    elif self.mode & self.SINGLEIO:
      packet_size = self.lastElement + 1
    elif self.mode & self.CONTINUOUS_READOUT:
      packet_size = int((( (self.wMaxPacketSize//bytesPerScan) * bytesPerScan) // 2))
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
    self.frequency = frequency

    packet_size -= 1  # force to uint8_t size in range 0-255    
    scanPacket = pack('IIIBB', count, retrig_count, pacer_period, packet_size, options)
    result = self.udev.controlWrite(request_type, self.AIN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

    self.status = self.Status()

  def AInScanRead(self):
    if self.mode & self.CONTINUOUS_READOUT or self.mode & self.SINGLEIO :
      nSamples = self.packet_size
    else:
      nSamples = self.count*(self.lastElement+1)

    try:
      data =  list(unpack('H'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, int(2*nSamples), self.HS_DELAY)))
    except:
      print('AInScanRead: error in bulkRead.')

    if len(data) != nSamples:
      raise ValueError('AInScanRead: error in number of samples transferred.')
      return len(data)
    

    if self.mode & self.VOLTAGE:
      for i in range(len(data)):
        gain = (self.scanList[i%(self.lastElement+1)] >> 3) & 0x3
        data[i] = data[i]*self.table_AIn[gain].slope + self.table_AIn[gain].intercept
        if data[i] > 0xffff:
          data[i] = 0xffff
        elif data[i] < 0.0:
          data[i] = 0x0
        else:
          data[i] = int(round(data[i]))
        data[i] = self.volts(gain, data[i])
        
    if self.bytesToRead > len(data)*2:
      self.bytesToRead -= len(data)*2
    elif self.bytesToRead > 0 and self.bytesToRead < len(data)*2:  # all done
      self.AInScanStop()
      self.AInScanClearFIFO()
      self.status = self.Status()
      return data

    if self.mode & self.CONTINUOUS_READOUT:
      return data

    # if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
    if nSamples*2%self.wMaxPacketSize == 0:
     dummy = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, 2, 100)
     
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

  def AInConfigR(self):
    """
    This command reads or writes the analog input channel
    configurations.  This command will result in a bus stall if an
    AIn scan is currently running.  The scan list is setup to
    acquire data from the channels in the order that they are placed
    in the scan list.

    ScanList[16]  channel configuration:
      bit 0:  Channel Mux 0
      bit 1:  Channel Mux 1
      bit 2:  Channel Mux 2
      bit 3:  Range Mux 0
      bit 4:  Range Mux 1
      bit 5:  Mode Mux 0
      bit 6:  Mode Mux 1
      bit 7:  Last Channel
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = self.udev.controlRead(request_type, self.AIN_SCAN_CONFIG, wValue, wIndex, 16, self.HS_DELAY)
    for i in range(len(value)):
      self.scanList[i] = value[i]
      if value[i] & self.LAST_CHANNEL:
        self.lastElement = i  # depth of queue
    return

  def AInConfigW(self, entry, channel, gain, mode, lastElement=False):
    """
    entry:    channel entry in the queue (0 - 15)
    channel:  channel number (Differential: 0-7, Single Ended: 0-15)
    gain:     range  ( 0:+/- 10V,  1: +/- 5V, 2: +/- 2V, 3: +/- 1V)
    lastElement: Set to True if last element in the queue
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CONFIG
    wValue = 0
    wIndex = 0

    if entry < 0 or entry >= self.NCHAN:
      raise ValueError('AInConfigW: Exceed depth of queue')
      return

    if mode == self.DIFFERENTIAL:
      if  channel > 7:
        raise ValueError('AInConfigW: Exceed number of channels in differential mode')
      else:
        self.scanList[entry] = (gain & 0x3) << 3 | (channel & 0x7)
    else:         # single ended mode
      if channel >= self.NCHAN:
        raise ValueError('AInConfigW: Exceed number of channels in single ended mode')
      elif channel > 7:
        self.scanList[entry] = (0x1 << 6) | (gain & 0x3) << 3 | (channel & 0x7)
      else:
        self.scanList[entry] = (0x1 << 5) | (gain & 0x3) << 3 | (channel & 0x7)

    if lastElement == True:
      self.lastElement = entry
      self.scanList[entry] |= self.LAST_CHANNEL

    if self.Status() & self.AIN_SCAN_RUNNING:
      self.AInScanStop()

    try:
      result = self.udev.controlWrite(request_type, request, wValue, wIndex, self.scanList, self.HS_DELAY)
    except:
      print('AInConfigW: error in control Write result =', result)
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

  ##########################################
  #         Counter/Timer Commands         #
  ##########################################

  def CounterInit(self, counter):
    """
    This command initializes the 32-bit event counter.  On a write,
    the specified counter (0 or 1) will be reset to zero.
    """
    if counter < 0 or counter > 1:
      raise ValueError('CounterInit: error in counter number.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = counter & 0x1
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.COUNTER, wValue, wIndex, [0x0], timeout = 100)

  def Counter(self, counter):
    """
    This command reads the 32-bit event counters.  
    """
    if counter < 0 or counter > 1:
      raise ValueError('Counter: error in counter number.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    data = unpack('II',self.udev.controlRead(request_type, self.COUNTER, wValue, wIndex, 8, self.HS_DELAY))
    return data[counter & 0x1]

  def TimerControlR(self):

    """
    This command reads/writes the timer control register
      control:   bit 0:    1 = enable timer,  0 = disable timer
                 bit 1:    1 = timer running, 0 = timer stopped
                          (read only, useful when using count)
                 bit 2:    1 = inverted output (active low)
                           0 = normal output (active high)
                 bits 3-7: reserved
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    data, = self.udev.controlRead(request_type, self.TIMER_CONTROL, wValue, wIndex, 1, self.HS_DELAY)
    self.timerParameters.control = data
    return data

  def TimerControlW(self, control):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_CONTROL
    wValue = control
    wIndex = 0
    self.timerParameters.control = control
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def TimerPeriodR(self):
    """
    This command reads or writes the timer period register.

    The timer is based on a 64 MHz input clock and has a 32-bit period register. The
    frequency of the output is set to:

    frequency = 64 MHz / (period + 1)

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.  This results in a minimum
    allowable value for the period of 1, which sets the maximum frequency to 64 MHz/2.

    Value returned is the timer period is ms
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    period ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_PERIOD, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters.period = period
    self.timerParameters.frequency = self.BASE_CLOCK/(period + 1)
    return 1000./self.timerParameters.frequency

  def TimerPeriodW(self, period):
    # period is in ms
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_PERIOD
    period = round(period*self.BASE_CLOCK/1000. - 1)
    self.timerParameters.period = period
    wValue = period & 0xffff
    wIndex = (period >> 16) & 0xffff
    period = pack('I', period)
    self.udev.controlWrite(request_type, request, wValue, wIndex, period, self.HS_DELAY)

  def TimerPulseWidthR(self):
    """
    This command reads/writes the timer pulse width register.
    The timer is based on a 64 MHz input clock and has a 32-bit pulse width register.
    The width of the output pulse is set to:

    pulse width = (pulseWidth + 1) / 64 MHz

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.

    Note: pule_width is in ms
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    pulse_width ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_PULSE_WIDTH, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters.pulseWidth = pulse_width
    return (pulse_width + 1)*1000/self.BASE_CLOCK

  def TimerPulseWidthW(self, pulse_width):
    # Note: pulse_width is in ms
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_PULSE_WIDTH
    pulseWidth = round(pulse_width*self.BASE_CLOCK/1000. - 1)
    self.timerParameters.pulseWidth = pulseWidth
    wValue = pulseWidth & 0xffff
    wIndex = (pulseWidth >> 16) & 0xffff
    pulseWidth = pack ('I', pulseWidth)
    self.udev.controlWrite(request_type, request, wValue, wIndex, pulseWidth, self.HS_DELAY)
    
  def TimerCountR(self):
    """
    This command reads/writes the timer count register.
    The number of output pulses can be controlled with the count register.  Setting
    this register to 0 will result in pulses being generated until the timer is disabled.
    Setting it to a non-zero value will results in the specified number of pulses being
    generated then the output will go low until the timer is disabled.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    count ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_COUNT, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters.count = count
    return count

  def TimerCountW(self, count):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_COUNT
    count = int(count)
    wValue = count & 0xffff
    wIndex = (count >> 16) & 0xffff
    count = pack('I', count)
    self.udev.controlWrite(request_type, request, wValue, wIndex, count, self.HS_DELAY)
    return count

  def TimerStartDelayR(self):
    """
    This command reads/writes the timer start delay register.  This
    register is the amount of time to delay before starting the timer
    output after enabling the output.  The value specifies the number
    of 64 MHZ clock pulses to delay.  This value may not be written
    while the timer output is enabled.

    Note: the start_delay is in ms
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    delay ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_START_DELAY, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters.delay = delay
    return delay*1000/self.BASE_CLOCK

  def TimerStartDelayW(self, delay):
    # delay is in ms
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_START_DELAY
    delay = round(delay*self.BASE_CLOCK/1000)
    self.timerParameters.delay = delay
    wValue = delay & 0xffff
    wIndex = (delay >> 16) & 0xffff
    delay = pack('I', delay)
    self.udev.controlWrite(request_type, request, wValue, wIndex, delay, self.HS_DELAY)

  def TimerParamsR(self):
    """
    This command reads/writes all timer parameters in one call.
    See the individual command descriptions for futher information
    on each parameter.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    data = unpack('IIII', self.udev.controlRead(request_type, self.TIMER_PARAMETERS, wValue, wIndex, 16, self.HS_DELAY))
    self.timerParameters.period = data[0]
    self.timerParameters.pulseWidth = data[1]
    self.timerParameters.count = data[2]
    self.timerParameters.delay = data[3]
    self.TimerControlW(self.timerParameters.control)

    frequency = self.BASE_CLOCK / (self.timerParameters.period + 1)
    duty_cycle = self.timerParameters.pulseWidth / self.timerParameters.period
    count =  self.timerParameters.count
    delay =  (self.timerParameters.delay * 1000.) / self.BASE_CLOCK # delay in ms

    return (frequency, duty_cycle, count, delay)

  def TimerParamsW(self, frequency, dutyCycle, count, delay):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_PARAMETERS
    wValue = 0
    wIndex = 0

    period = round(self.BASE_CLOCK/frequency - 1)
    pulseWidth = round(period * dutyCycle)

    if period < 1:
      raise ValueError('TimerParametersW: period less than 1.')
      period = 1         # set to lowest value

    self.timerParameters.period = int(period)
    self.timerParameters.pulseWidth = int(pulseWidth)
    self.timerParameters.count = int(count)
    self.timerParameters.delay = int(delay)
    self.timerParameters.frequency = frequency
    
    barray = pack('IIII', self.timerParameters.period, self.timerParameters.pulseWidth, \
                  self.timerParameters.count, self.timerParameters.delay)
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
    0x0000-0x3FFF               Microcontroller firmware (write protected)
    0x4000-0x40FF               Caliabration Storage
    0x4100-0x7FFF               User data

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
    0x0000-0x3FFF.  This command is only to be used when updating the
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

  def TriggerConfigR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= unpack('B',self.udev.controlRead(request_type, self.TRIGGER_CONFIG, wValue, wIndex, 1, self.HS_DELAY))
    return value

  def Temperature(self):
    """
    The command reads the internal temperature.  The temperature
    (degrees C) is calculated as:

        T = reading / 2^15 * 128
    """
    
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= unpack('h',self.udev.controlRead(request_type, self.TEMPERATURE, wValue, wIndex, 2, self.HS_DELAY))
    return value/256.

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
      print(' Analog Outputt scan done.')
    if status & self.FPGA_CONFIGURED:
      print('  FPGA is configured.')
    if status & self.FPGA_CONFIG_MODE:
      print('  FPGA in configuration mode.')

  def volts(self, gain, value):
    # converts unsigned short value to volts
    if gain == self.BP_10V:
      volt = (value - 0x8000) * 10. / 32768
    elif gain == self.BP_5V:
      volt = (value - 0x8000) * 5. / 32768
    elif gain == self.BP_2V:
      volt = (value - 0x8000) * 2. / 32768
    elif gain == self.BP_1V:
      volt = (value - 0x8000) * 1. / 32768
    else:
      raise ValueError('volts: Unknown range.')

    return volt

    
################################################################################################################

class usb_1608G(usb1608G):
  def __init__(self, serial=None):
    self.productID = self.USB_1608G_PID   #usb-1608G
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1608G not found")
      return
    usb1608G.__init__(self)

class usb_1608GX(usb1608G):
  def __init__(self, serial=None):
    self.productID = self.USB_1608GX_PID   #usb-1608GX
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1608GX not found")
      return
    usb1608G.__init__(self)

class usb_1608GX_2AO(usb1608G):

  # Analog Output Commands
  AOUT                 = 0x18  # Read/write analog output channel
  AOUT_SCAN_START      = 0x1A  # Start analog output scan
  AOUT_SCAN_STOP       = 0x1B  # Stop analog output scan
  AOUT_SCAN_CLEAR_FIFO = 0x1C  # Clear the analog output scan FIFO

  # Analog Output Scan Options
  AO_CHAN0       = 0x1   # Include Channel 0 in output scan
  AO_CHAN1       = 0x2   # Include Channel 1 in output scan
  AO_TRIG        = 0x10  # Use Trigger
  AO_RETRIG_MODE = 0x20  # Retrigger Mode

  NCHAN_AO       = 2     # Number of analog output channels

  def __init__(self, serial=None):
    self.productID = self.USB_1608GX_2AO_PID   # usb-1608GX_2AO
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1608GX_2AO not found")
      return
    usb1608G.__init__(self)

    self.samplesToWrite       = -1    # Total number of samples to write

    # Read calibration table for analog out
    self.table_AOut = [table(), table()]
    address = 0x7080
    self.MemAddressW(address)
    for chan in range(self.NCHAN_AO):
      self.table_AOut[chan].slope, = unpack('f', self.MemoryR(4))
      self.table_AOut[chan].intercept, = unpack('f', self.MemoryR(4))

  #############################################
  #           Analog Output                   #
  #############################################

  def AOut(self, channel, voltage):
    """
    This command reads or writes the values for the analog output
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

    if channel >= self.NCHAN_AO:
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

  def AOutR(self, channel):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    channel = int(channel)
    if channel < 0 or channel >= self.NCHAN_AO:
      raise ValueError('AOutR: channel out of range')
      return

    value = unpack('HH',self.udev.controlRead(request_type, self.AOUT, wValue, wIndex, 8, timeout = 400))
    voltage = (value[channel] - self.table_AOut[channel].intercept) / self.table_AOut[channel].slope
    voltage = (voltage - 32768)*10./32768.
    return voltage

  def AOutScanStart(self, count, retrig_count, frequency, options):
    """
    This command starts the analog output channel scan.  This command
    will result in a bus stall if an AOutScan is currently running.

    count:        the total number of scans to perform (0 = continuous mode)
    retrig_count: the number of scans to perform for each trigger in
                  retrigger mode
    frequency:    pacer frequency (0 for AO_CLK_IN)
    options:      bit 0: 1 = include channel 0 in output scan
                  bit 1: 1 = include channel 1 in output scan
                  bit 2: 1 = reserved
                  bit 3: 1 = reserved
                  bit 4: 1 = use trigger
                  bit 5: 1 = retirgger mode, 0 = normal trigger
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
    a base rate of 64 MHz.  The timer is controlled by pacer_period.  
    The equation for calculating pacer_period is:

        pacer_period = (64 MHz / sample_frequency) - 1

    The same time base is used for all channels when the scan involved
    multiple channels.  The output data is to be sent using the bulk
    out endpoint.  The data must be in the format:

      low channel sample 0 : [high channel sample 0]
      low channel sample 1 : [high channel sample 1]
      ...
      low channel sample n : [high channel sample n]

    The output is written to an internal FIFO.  The bulk endpoint data
    is only accepted if there is room in the FIFO.  Output data bay be
    sent to the FIFO before the start of the scan, and the FIFO is
    cleared when the AOutScanClearFIFO command is received.  The scan
    will not begin until the AOutScanStart command is sent (and outupt
    data is in the FIFO).  Data will be output until reaching the
    specified number of scans (in single execution mode) or an
    AOutScanStrop command is sent.
    """

    if frequency < 0.:
      raise ValueError('AOutScanStart: frequency must be positive')
      return
    elif frequency > 50000:
      raise ValueError('AOutScanStart: frequency must be less than 50 kHz')
      return

    if frequency == 0:
      pacer_period = 0    # use AOCKI pin 47
    else:
      pacer_period = round((self.BASE_CLOCK / frequency) - 1)

    self.frequency_AOut = frequency
    self.options_AOut = options
    self.count_AOut = count
    if count == 0:
      self.continuous_mode_AOUT = True
      self.samplesToWrite = -1
    else:
      self.continuous_mode_AOUT = False
      # calculate number of channels per scan
      for i in range(self.NCHAN_AO): 
        if (options >> i) & 0x1 == 1:
          self.samplesToWrite += 1
      self.samplesToWrite *= count

    self.retrig_count = retrig_count

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    scanPacket = pack('IIIB', count, retrig_count, pacer_period, options)
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

  def AOutScanWrite(self, data):
    # data is a list of unsigned 16 bit numbers
    value = [0]*len(data)*2
    timeout = int(500 + 1000*len(data)/self.frequency_AOut)

    for i in range(len(data)):
      value[2*i] = data[i] & 0xff
      value[2*i+1] = (data[i] >> 8) & 0xff
    try:
      result = self.udev.bulkWrite(2, value, timeout)
    except:
      print('AOutScanWrite: error in bulkWrite')
      return
    
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
    The command clears the internal scan endoint FIFOs
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, self.AOUT_SCAN_CLEAR_FIFO, wValue, wIndex, [0x0], timeout = 100)

