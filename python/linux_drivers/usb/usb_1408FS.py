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
from mccUSB import *


class usb_1408FS(mccUSB):
  # Gain Ranges
  SE_10_00V   = 0x9         # Single Ended 0-10.0 V
  BP_20_00V   = 0x0         # Differential +/- 20.0 V
  BP_10_00V   = 0x1         # Differential +/- 10.0 V
  BP_5_00V    = 0x2         # Differential +/- 5.00 V
  BP_4_00V    = 0x3         # Differential +/- 4.00 V
  BP_2_50V    = 0x4         # Differential +/- 2.50 V
  BP_2_00V    = 0x5         # Differential +/- 2.00 V
  BP_1_25V    = 0x6         # Differential +/- 1.25 V
  BP_1_00V    = 0x7         # Differential +/- 1.00 V

  # Status Bits
  SYNC               = 0x1  # 0 = Sync slave,             1 = Sync master
  EXT_TRIG_EDGE      = 0x2  # 0 = trigger falleng edge,   1 = trigger rising edge
  GATED_SYNC         = 0x4  # 0 = normal sync(slave mode) 1 = gated sync  
  UPDATE_MODE        = 0x8  # 1 = program memory update mode

  # Option values for AInScan
  AIN_EXECUTION      = 0x1  # 1 = Single execution, 0 = Continuous execution
  AIN_TRANSFER_MODE  = 0x2  # 1 = Immediate transfer mode  0 = block transfer mode
  AIN_TRIGGER        = 0x4  # 1 = Use external trigger
  AIN_DEBUG          = 0x8  # 1 = Debug mode.
  AIN_GAIN_QUEUE     = 0x10 # 1 = Use Channel Gain Queue, 0 = Use channnel parameters
  AIN_RETRIGGER      = 0x20 # 1 = Retrigger mode, 0 = normal trigger

  #Option values for AOutScan
  AOUT_EXECUTION     = 0x1  # 1 = single execution, 0 = continuous execution
  AOUT_TRIGGER       = 0x2  # 1 = Use external trigger

  DIO_PORTA       =  0
  DIO_PORTB       =  1
  DIO_DIR_IN      =  1
  DIO_DIR_OUT     =  0
  
  # Commands and Codes for USB 1[2,4,6]08FS HID reports
  # Digital I/O Commands
  DCONFIG          = 0x01   # Configure digital port
  DIN              = 0x03   # Read digital port
  DOUT             = 0x04   # Write digital port

  # Analog Input Commands
  AIN              = 0x10   # Read analog input channel
  AIN_SCAN         = 0x11   # Scan analog channels
  AIN_STOP         = 0x12   # Stop input scan
  ALOAD_QUEUE      = 0x13   # Load the channel/gain queue

  # Analog Output Commands
  AOUT             = 0x14   # Write analog output channel
  AOUT_SCAN        = 0x15   # Output values to a range of output channels
  AOUT_STOP        = 0x16   # Stop output scan

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
  GET_ALL          = 0x46   # Retrieve all analog and digital input values

  # Code Update Commands
  PREPARE_DOWNLOAD = 0x50   # Prepare for program memory download
  WRITE_CODE       = 0x51   # Write program memory
  WRITE_SERIAL     = 0x53   # Write new serial number to device
  READ_CODE        = 0x55   # Read program memory

  productID        = 0

  def __init__(self, serial=None):
    self.productID = 0x00a1                            # MCC USB-1408FS
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-1408FS not found")
    for i in range(4):
      if sys.platform.startswith('linux'):
        if self.udev.kernelDriverActive(i):
          self.udev.detachKernelDriver(i)
          self.udev.resetDevice()
      self.udev.claimInterface(i)

    # need to get wMaxPacketSize
    self.wMaxPacketSize = self.getMaxPacketSize()

    # Build a lookup table of calibration coefficients to translate values into voltages:
    # The calibration coefficients are stored in the onboard FLASH memory on the device in
    # IEEE-754 4-byte floating point values.
    #
    #   calibrated code = code*slope + intercept
    self.CalDF = [[table(), table(), table(), table(), table(), table(), table(), table()], \
                  [table(), table(), table(), table(), table(), table(), table(), table()], \
                  [table(), table(), table(), table(), table(), table(), table(), table()], \
                  [table(), table(), table(), table(), table(), table(), table(), table()]]
    self.CalSE = [table(), table(), table(), table(), table(), table(), table(), table()]

  # Analog Input Calibration, differential 0x200 - 0x01C
    address = 0x200
    for chan in range(4):
      for gain in range(8):
        self.CalDF[chan][gain].slope ,= unpack('f',self.MemRead(address, 4))
        address += 4
        self.CalDF[chan][gain].intercept ,= unpack('f',self.MemRead(address, 4))
        address += 4

    # Analog Input Calibration, single ended 0x0300 - 0x033C
    address = 0x300
    for chan in range(8):
      self.CalSE[chan].slope ,= unpack('f',self.MemRead(address, 4))
      address += 4
      self.CalSE[chan].intercept ,= unpack('f',self.MemRead(address, 4))
      address += 4

    
  #################################
  #     Digital I/O  Commands     #
  #################################

  def DConfig(self, port, direction):
    """
    This command sets the direction of the DIO ports to input or output
       port:       0 - Port A,   1 - Port B
       direction:  0 - output,   1 - input
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.DCONFIG  # HID output
    wIndex = 0                         # interface
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, \
                                    [self.DCONFIG, port, direction], timeout = 100)

  def DIn(self, port):
    """
    This command reads the current state of the digital port.  The return value will be
    the value seen at the port pins.
     port:  0 - Port A,   1 = Port B
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                  # HID Set_Report
    wValue =  (2 << 8) | self.DIN  # HID output
    wIndex = 0                     # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DIN, port], timeout = 100)
    value = unpack('BBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 3, timeout = 100))    

    if port == 0:
      return value[1]
    else:
      return value[2]

  def DOut(self, port, value):
    # This command writes data to the DIO port bits that are configured as outputs.
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                   # HID Set_Report
    wValue =  (2 << 8) | self.DOUT  # HID output
    wIndex = 0                      # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.DOUT, port, value], timeout = 100)

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
    if gain == self.SE_10_00V:
      # offset channels by 8
      # set range value to 0 (+/-10V)
      ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.AIN, channel+8, 0x0], timeout = 100)
    else:
      ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.AIN, channel, gain], timeout = 100)
    value = unpack('BBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 3, timeout = 1000))    
    value = value[1] | (value[2] << 8)
    if gain == self.SE_10_00V:
      # The data is 2's complement signed 13 bit number
      if value > 0x7ffc:
        value = 0
      elif value > 0x7ff8:
        value = 0x3fff
      else:
        value >>= 1
        value &= 0x3fff
      value -= 0x2000
      value = int(value*self.CalSE[channel].slope + self.CalSE[channel].intercept)
    else:
      # The data is 2's complement signed 14 bit number
      value ,= unpack('h',pack('H',value))
      value = value/4
      value = int(value*self.CalDF[channel][gain].slope + self.CalDF[channel][gain].intercept)

    return value

  def AInScan(self, lowchannel, hichannel, gains, count, frequency, options):
    """
    This command scans a range of analog input channels and sends the
    readings in interrupt transfers. The gain ranges that are
    currently set on the desired channels will be used (these may be
    changed with AIn or ALoadQueue.

        lowchannel:  the first channel of the scan (0–3 Differential, 0-7 Single Ended)
        hichannel:   the last channel of the scan (0–3 Differential, 0-7 Single Ended)
        gains:       array of integer ranges for the gain queue (See ALoadQueue)
        count:       the total number of samples to perform, used only in single execution mode
        options:     bit 0: 1 = single execution, 0 = continuous execution
                     bit 1: 1 = immediate transfer mode, 0 = block transfer mode
                     bit 2: 1 = use external trigger
                     bit 3: 1 = not used
                     bit 4: 1 = use channel gain queue, 0 = use channel parameters specified
                     bit 5: 1 = retrigger mode, 0 = normal trigger

    The values lowchannel and hichannel specify the channel range for
    the scan.  If lowchannel is higher than hichannel, the scan will
    wrap (ie if lowchannel is 6 and high channel is 1, the scan will
    return channels 6, 7, 0 and 1)
    
    The sample rate is set by the internal 16-bit incrementing timer
    running at a base rate of 10MHz. The timer is controlled by
    timer_prescale and timer_preload. These values are only used if
    the device has been set to master the SYNC pin with the SetSync
    command.

    The timer will be reset and provide an internal interrupt with its
    value equals timer_preload.  This allows for a lowest rate of
    0.596 Hz (1:256 prescale, preload = 0xFFFF).  It is preferable to
    keep the prescaler to the lowest value that will achive the
    desired rate.

              preload = (10 MHz / (frequency * prescaler)) - 1

    The data will be returned in packets utilizing interrupt in endpoints. Two endpoints will be
    used; each endpoint allows 64 bytes of data to be sent every millisecond, so the theoretical
    limit is:
         2 endpoints * 64 bytes/ms = 128 bytes/ms = 128,000 bytes/s = 64,000 samples/s

    The data will be in the format:
    lowchannel sample 0 : lowchannel + 1 sample 0 :… : hichannel sample 0
    lowchannel sample 1 : lowchannel + 1 sample 1 :… : hichannel sample 1
    .
    .
    .
    lowchannel sample n : lowchannel + 1 sample n : … : hichannel sample n

    The data will use successive endpoints, beginning with the first
    endpoint at the start of a scan and cycling through the second
    endpoint until reaching the specified count or an AScanStop is sent.
    Immediate transfer mode is used for low sampling rates to avoid
    delays in receiving the sampled data. The data will be sent at the
    end of every timer period, rather than waiting for the buffer to
    fill. Both endpoints will still be used in a sequential manner. This
    mode should not be used if the aggregate sampling rate is greater
    than 2,000 samples per second in order to avoid data loss.

    The external trigger may be used to start data collection
    synchronously. If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate. No messages will be sent until the trigger is
    detected.

    The retrigger mode option is only used if trigger is used.  This
    option will cause the trigger to be rearmed after count samples
    are acquired if in continuous mode.

    """
    
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                       # HID Set_Report
    wValue =  (2 << 8) | self.AIN_SCAN  # HID output
    wIndex = 0                          # interface

    if hichannel > 7:
      raise ValueError('AInScan: hichannel out of range')
      return
    if lowchannel > 7:
      raise ValueError('AInScan: lowchannel out of range')
      return

    nSamples = count
    count += count%31   # fill up entire scan line

    for prescale in range(9):
      preload = 10.E6/(frequency * (1 << prescale))
      if preload <= 0xffff:
        break

    if prescale == 9 or preload == 0:
      raise ValueError('AInScan: frequency out of range')
      return

    # Load the gain queue
    nchan = (hichannel - lowchannel + 1)
    channels = [0]*8
    for i in range(nchan):
      if gains[i] != self.SE_10_00V:
        channels[i] = lowchannel + i
      else:
        # add 8 to channels for Single Ended
        channels[i] = lowchannel + i + 8
    self.ALoadQueue(nchan, channels, gains)

    if gains[0] == self.SE_10_00V:
      lowchannel += 8
      hichannel += 8
    buf =  [self.AIN_SCAN, lowchannel, hichannel, count & 0xff, (count>>8) & 0xff, (count>>16) & 0xff, \
            (count>>24) & 0xff,  prescale, int(preload) & 0xff, (int(preload)>>8) & 0xff, options]
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, buf, timeout = 5000)
    i = 0
    pipe = 1  # initial endpoint to receive data
    sdata = [0]*nSamples

    while nSamples > 0:
      value = unpack('h'*32,self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | (pipe+2), 64, timeout = 1000))
      if nSamples > 31:
        for k in range(31):
          sdata[i+k] = int(value[k]>>2)
        nSamples -= 31
      else:
        for k in range(nSamples):
          sdata[i+k] = int(value[k]>>2)
        nSamples = 0
        break
      i += 31
      pipe = (pipe%3) + 1   # pip should take on the values 1, 2 or 3
    return sdata

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

  def ALoadQueue(self, num, chan, gain):
    """
    The device can scan analog channels with different gain settings.  This
    function provides the mechanism for configuring each channel with a unique
    range
    
    num:       the number of channel/gain pairs (max 8)
    gainQueue: the  channel/gain pairs
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                          # HID Set_Report
    wValue =  (2 << 8) | self.ALOAD_QUEUE  # HID output
    wIndex = 0                             # interface

    gainQueue = [0]*16   
    for i in range(num):
      if gain[i] == self.SE_10_00V:
        gainQueue[2*i]   = chan[i] + 8
        gainQueue[2*i+1] = 0
      else:
        gainQueue[2*i]   = chan[i]
        gainQueue[2*i+1] = gain[i]
    buf = [0]*18
    buf[0] = self.ALOAD_QUEUE
    buf[1] = num
    for i in range(2*num):
      buf[2+i] = gainQueue[i]

    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, buf, timeout = 100)


  #################################
  #   Analog Output  Commands     #
  #################################

  def AOut(self, channel, value):
    """
    This command writes the value to an analog output channel.
    The value is a 16 bit unsigned value, but the DAC is a
    12-bit DAC.  The lower 4 bits of the value are ignored by
    the DAC.  The equation for the output voltage is:
    
         V_out = (k/2^16)* V_ref  where k is the value
                                  written to the device
         V_ref = 4.096V
         channel: the channel to write (0-1)
         value:   the value to write to the DAC
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                   # HID Set_Report
    wValue =  (2 << 8) | self.AOUT  # HID output
    wIndex = 0                      # interface

    value <<= 4                     # shift over 4 bits
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, \
                                 [self.AOUT, channel, value & 0xff, (value >> 8) & 0xff], timeout = 100)

  def AOutScan(self, lowchannel, hichannel, frequency, data, options):
    """
    This command writes values to the analog output channels at a fixed rate.
    The values lowchannel and hichannel specify the channel range for the
    scan.  If lowchannel is higher than hichannel, the parameters will be
    reversed in the device (lowchannel must be less than hichannel).
    
    The rate of data output is set by the internal 16-bit incrementing
    timer running at a base rate of 10 MHz.  The timer is controlled
    by timer_prescale and timer_preload.
    
    The timer will be reset and provide an internal interrupt when its
    value equals timer_preload.  This allows for a lowest rate of 0.596 Hz
    (1:256 prescale, preload = 0xFFFF).  It is preferable to keep the
    prescaler to the lowest value that will achieve the desired rate.
    
         preload = 10 MHz / (frequency * prescaler)
    
    The data will be sent in packets untilizing the interrupt out
    endpoint on interface 1.  The endpoint allows for 64 bytes of
    data to be sent every millisecond, so the theroretical limit is:
    
       64 bytes/ms = 32,000 S/s
    
    The data will be in the format:
    
     lowchannel sample 0 : [hichannel sample 0]
     lowchannel sample 1 : [hichannel sample 1]
           ....
     lowchannel sample n : [hichannel sample n]
    
    The external trigger may be used to start data output synchronously.
    If the bit is set, the device will wait until an appropriate trigger
    edge is detected, then begin outputting data at the specified rate.
    
    The data transfer is controlled by the USB-1408FS using data requests.
    The USB-1408FS will send an input report on interface 0 with the report
    ID AOUT_SCAN to request a new packet of data.  It will maintain its
    internal FIFO by requesting new data when it is ready.  The first byte
    of the report will be a status indicator; 0 = no errors, 1 = data underrrun,
    2 = scan complete.
    
    The count parameter is only used in single execution mode.  In continuous
    execution mode, data will be sent by the host indefinitely, with the host
    sending an AOutStop() command to end the scan.
    
     lowchannel: the first channel of the scan [0-1]
     count:      the number of scans to perform
     hichannel:  the last chanel of the scan [0-1]
     frequency:  output frequency in Hz.
     options:    bit 0:   1 = single execution, 0 = continuous execution
                 bit 1:   1 = use external trigger
                 bits 2-7: not used
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                        # HID Set_Report
    wValue =  (2 << 8) | self.AOUT_SCAN  # HID output
    wIndex = 0                           # interface

    if (hichannel > 1):
      raise ValueError('AOutScan: hichannel out of range')
      return
    if (lowchannel > 1):
      raise ValueError('AOutScan: lowchannel out of range')
      return
    if (lowchannel > hichannel):
      raise ValueError('AOutScan: lowchannel greater than hichannel')
      return
    if lowchannel == hichannel:
      count = len(data)
    else:
      # 1 scan for every 2 samples
      count = int(len(data)/2)

    if (frequency > 0.596 and frequency < 10000):
      for prescale in range(9):
        preload = 10.E6/(frequency * (1<<prescale))
        if preload <= 0xffff:
          break
    elif frequency == 0:      # external sync
      preload = 0xff
    else:
      raise ValueError('AOutScan: frequency out of range.')
      return

    preload = int(preload)

    if prescale == 9 or preload == 0:
      raise ValueError('AOutScan: frequency out of range.')

    nSamples = len(data)
    timeout = int((nSamples*1000)/frequency + 1000)

    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, \
            [self.AOUT_SCAN, lowchannel, hichannel, count & 0xff, (count >> 8) & 0xff, (count >> 16) & 0xff, \
             (count >> 24) & 0xff, prescale, preload & 0xff, (preload >> 8) & 0xff, options], timeout = 100)

    if (options & self.AOUT_EXECUTION) == 0x0:  # continuous execution
      return

    value = [0]*2*nSamples     # 2 bytes per sample

    for i in range(nSamples):
      x = (data[i] << 4)         # shift over all data 4 bits
      value[2*i] = x & 0xff      # low byte
      value[2*i+1] = (x) & 0xff  # high byte

    i = 0
    while nSamples >= 32:
      try:
        ret = self.udev.interruptWrite(libusb1.LIBUSB_ENDPOINT_OUT | 2, value[64*i:(i+1)*64], timeout)
      except:
        pass
      try:
        ret = unpack('BB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2, timeout = 100))
        if ret[1] == 1: # Underrun error
          raise UnderrunError
      except UnderrunError:
          print('AInScan: data underrun error')
          pass
      except:
        pass
      i += 64
      nSamples -= 32
    if nSamples > 0:  # partial scan
      ret = self.udev.interruptWrite(libusb1.LIBUSB_ENDPOINT_OUT | 2, value[64*i:64*i+nSamples], timeout = 100)
    self.AOutStop()
    try:  # clear out the input buffer
      ret = self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2, timeout = 100)
    except:
      pass

  def AOutWrite(self, data, timeout):
    ret = self.udev.interruptWrite(libusb1.LIBUSB_ENDPOINT_OUT | 2, data, timeout)
    while True:
      try:  # clear out the input buffer
        ret = self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2, 100)
      except:
        return
    
  def AOutStop(self):
    """
    This command stops the analog output scan (if running).
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                        # HID Set_Report
    wValue =  (2 << 8) | self.AOUT_STOP  # HID output
    wIndex = 0                           # interface
    value = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.AOUT_STOP], timeout = 100)


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
    value = unpack('BBBBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 5, timeout = 100))    
    return (value[1] | (value[2]<<8) | (value[3]<<16) | (value[4]<<24))
    
  #################################
  #     Memory  Commands          #
  #################################

  def MemRead(self, address, count):
    """
    This command reads data from the configuration memeory (EEPROM).
    All of the memory may be read
    
     address: the start addess for the read
     count:   the number of byes to read (62 max)
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
    buf = [self.MEM_READ, address & 0xff, (address >> 8) & 0xff, 0x0, count]
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, buf, timeout = 5000)
    try:
      data = self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 64, timeout = 1000)
    except:
      print("MemRead: error")
    for i in range(count):
      value[i] = data[i+1]
    return value

  def MemWrite(self, address, count, data):
    """
    This command writes data to the non-volatile EEPROM memory on the device.
    The non-volatile memory is used to store calibration coefficients, system
    information and user data.
    
    Locations 0x200 - 0x3FF are reserved for calibration data nad require
    the unlock sequence prior to writing
    
    Unlock sequence: Wrie 2 bytes with the value 0xAA55 to address 0x40 to
    unlock.  The unlock status can be checked with GetStatus.  The unlock will
    remain in effect until the device is powered off or reset.
    
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
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.SET_TRIGGER,trig_type], timeout = 100)

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
    kHz) This may also be used with an external trigger; the
    external trigger signal should be brought to the master device,
    and all devices will begin sampling when the master is
    triggered.  If a device is configred as a slave, it will not
    acquire data when given an AInScan ommand until it detects a
    pulse on the sync input.  If configured as a slave with a
    continuous clock, an additional sync pulse is required to set up
    the AInScan.  If configured as a slave with a gated clock, the
    additional sync pulse is not required.  However, if a sync pulse
    is receied while the AInScan setup is being performed by the
    device, improper operation may result.  this is intennded for
    use when synchronizing with another USB-1408FS, where the sync
    signal will not be present until the master device has been
    issued an AInScan command.
    
    The device will switch the SYNC pin to the appropriate
    input/output state when this command is received.
    
     sync_type: 0 = master,
                1 = slave with continuous clock
                2 = slave with gated clock
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
     Bit 2: 0 = Normal sync (slave mode)  1 = gated sync
     Bit 3: 0 = EEPROM cal memory locked  1 = cal memory unlocked
     Bits 4-15 unused.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                         # HID Set_Report
    wValue =  (2 << 8) | self.GET_STATUS  # HID output
    wIndex = 0                            # interface
    value = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.GET_STATUS], timeout = 100)
    value = unpack('BBB',self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 3, timeout = 100))

    return (value[1] | (value[2]<<8)) & 0xf

  def SetCal(self, setting):
    """
    This command sets the voltage on the CAL output.  The output
    will be 0V at power up, and should be returned to 0 when not in
    use.
    
       setting:  0 = 0V,   1 = 2.5V
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.SET_CAL  # HID output
    wIndex = 0                         # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.SET_CAL, setting], timeout = 100)

  def GetAll(self):
    """
    This command reads the value from all analog input channels and digital I/Os.
    """
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                      # HID Set_Report
    wValue =  (2 << 8) | self.GET_ALL  # HID output
    wIndex = 0                         # interface
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, [self.GET_ALL], timeout = 100)
    value = unpack('B'*35,self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 35, timeout = 1000))

    ain = [0]*16
    for i in range(16):
      ain[i] = value[2*i+1] | value[2*i+2] << 8
      ain[i] ,= unpack('h', pack('H', ain[i]))
    return [ain[0], ain[1], ain[2],  ain[3],  ain[4],  ain[5],  ain[6],  ain[7],\
            ain[8], ain[9], ain[10], ain[11], ain[12], ain[13], ain[14], ain[15],\
            value[33], value[34]]

  
  #################################
  #     Code Update Commands      #
  #################################

  def PrepareDowload(self):
    """
    This command puts the device into code update mode.  The unlock code must be correct as a
    further safety device.  Call this once before sending code with WriteCode.  If not in
    code update mode, any WriteCode will be ignored.  
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
    This command writes to the program memory in the device.  This command is not accepted
    unless the device is in update mode.  This command will normally be used when downloading
    a new hex file, so it supports memory ranges that may be found in the hex file.  
    
    The address ranges are:
    
    0x000000 - 0x007AFF:  FLASH program memory
    0x200000 - 0x200007:  ID memory (serial number is stored here on main micro)
    0x300000 - 0x30000F:  CONFIG memory (processor configuration data)
    0xF00000 - 0xF03FFF:  EEPROM memory
    
    FLASH program memory: The device must receive data in 64-byte segments that begin
    on a 64-byte boundary.  The data is sent in messages containing 32 bytes.  count
    must always equal 32.
    
    Other memory: Any number of bytes up to the maximum (32) may be sent.
    """

    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                         # HID Set_Report
    wValue =  (2 << 8) | self.WRITE_CODE  # HID output
    wIndex = 0                            # interface

    if (count > 32):
      raise ValueError('WriteCode: count greater than 32')
      return
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, \
            [self.WRITE_CODE, address&0xff, (address>>8)&0xff, (address>>16)&0xff, count, data[0:count]], timeout = 100)
    
  def ReadCode(self, address, count):
    # This command reads from program memory.
    request_type = libusb1.LIBUSB_ENDPOINT_OUT | \
                   libusb1.LIBUSB_TYPE_CLASS   | \
                   libusb1.LIBUSB_RECIPIENT_INTERFACE
    request = 0x9                        # HID Set_Report
    wValue =  (2 << 8) | self.READ_CODE  # HID output
    wIndex = 0                           # interface

    if (count > 62):
      raise ValueError('ReadCode: count greater than 62')
      return
    ret = self.udev.controlWrite(request_type, request, wValue, wIndex, \
             [self.READ_CODE, address&0xff, (address>>8)&0xff, (address>>16)&0xff, count], timeout = 100)                

    value = unpack('B'*(count+1),self.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, count+1, timeout = 100))    
    return (value[1,count+1])

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

  def volts(self, gain, num):
    # converts signed short value to volts
    if gain == self.BP_20_00V:
      volt = num * 20.0 / 0x1fff
    elif gain == self.BP_10_00V:
      volt = num * 10.0 / 0x1fff
    elif gain == self.BP_5_00V:
      volt = num * 5.0 / 0x1fff
    elif gain == self.BP_4_00V:
      volt = num * 4.0 / 0x1fff
    elif gain == self.BP_2_50V:
      volt = num * 2.5 / 0x1fff
    elif gain == self.BP_2_00V:
      volt = num * 2.0 / 0x1fff
    elif gain == self.BP_1_25V:
      volt = num * 1.25 / 0x1fff
    elif gain == self.BP_1_00V:
      volt = num * 1.0 / 0x1fff
    elif gain == self.SE_10_00V:
      volt = num * 10.0 / 0x1fff + 0.0
    return volt

  def printStatus(self):
    status = self.Status()
    print('**** USB-1408FS Status ****')
    if status & self.SYNC:
      print('    Sync Master')
    else:
      print('    Sync Slave')
    if status & self.EXT_TRIG_EDGE:
      print('    Trigger rising edge')
    else:
      print('    Trigger falling edge')
    if status & self.GATED_SYNC:
      print('    Gated sync')
    else:
      print('    Normal sync')
    if status & self.UPDATE_MODE:
      print('    Program memory update mode')
      
