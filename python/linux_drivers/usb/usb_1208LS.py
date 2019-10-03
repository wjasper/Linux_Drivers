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

class usb_1208(mccUSB):

  DIO_PORTA     = 0x01
  DIO_PORTB     = 0x04
  DIO_DIR_IN    = 0x01
  DIO_DIR_OUT   = 0x00

  MINILAB_CLOCK  = 6000000       # 6 MHz clock

  OFFSET_ADJUSTMENT =  0x1F00    # Offset Adjustment for the A/D        0x1F00 - 0x1F4F
  SE_GAIN_ADJUSTMENT = 0x1F50    # Single Ended Gain Adjustment for A/D 0x1F50 - 0x1F5F
  DE_GAIN_ADJUSTMENT = 0x1F60    # Differential Gain Adjustment for A/D 0x1F60 - 0x1F67
  CAL_PIN_VOLTAGE    = 0x1FA0    # Calibration pin voltage              0x1FA0 - 0x1FA3

  EXT_TRIG_FAILING_EDGE = 0
  EXT_TRIG_RAISING_EDGE = 1

  # Gain Ranges
  SE_10_00V  = 0x8            # Single Ended +/- 10.0 V
  BP_20_00V  = 0x00           # Differential +/- 20.0 V
  BP_10_00V  = 0x10           # Differential +/- 10.0 V
  BP_5_00V   = 0x20           # Differential +/- 5.00 V
  BP_4_00V   = 0x30           # Differential +/- 4.00 V
  BP_2_50V   = 0x40           # Differential +/- 2.50 V
  BP_2_00V   = 0x50           # Differential +/- 2.00 V
  BP_1_25V   = 0x60           # Differential +/- 1.25 V
  BP_1_00V   = 0x70           # Differential +/- 1.00 V

 # Option values for AInScan
  AIN_EXECUTION     = 0x1  # 1 = single execution, 0 = continuous execution
  AIN_BURST_MODE    = 0x2  # 1 = Burst Mode
  AIN_TRANSFER      = 0x4  # 1 = Block Transfer Mode
  AIN_TRIGGER       = 0x8  # 1 = Use External Trigger

  # Commands and Codes for USB 1208-LS HID reports
  # Digital I/O Commands
  DCONFIG     = 0x0D     # Configure digital port
  DIN         = 0x00     # Read digital port
  DOUT        = 0x01     # Write digital port
  DBIT_IN     = 0x02     # Read digital port bit
  DBIT_OUT    = 0x03     # Write digital port bit

  # Analog Input Commands
  AIN         = 0x06     # Read analog input channel
  AIN_SCAN    = 0x0E     # Scan analog channels
  AIN_STOP    = 0x10     # Stop scan
  ALOAD_QUEUE = 0x07     # Load the channel/gain queue

  # Analog Output Commands
  AOUT        = 0x08     # Write analog output channel

  # Counter Commands
  CINIT       = 0x05     # Initialize counter
  CIN         = 0x04     # Read Counter

  # Memory Commands
  MEM_READ    = 0x09     # Read Memory
  MEM_WRITE   = 0x0A     # Write Memory

  # Miscellaneous Commands
  BLINK_LED   = 0x0B     # Causes LED to blink
  RESET       = 0x11     # Reset USB interface
  SET_TRIGGER = 0x14     # Configure external trigger
  SET_ID      = 0x0C     # Set the user ID
  GET_ID      = 0x0F     # Get the user ID

  scanIdx     = 0        # scan index
  productID   = 0        # product ID

  def __init__(self):
    try:
      self.h = hid.device()
    except:
      print('Error creating hid device')

  #################################
  #     Digital I/O  Commands     #
  #################################

  def DConfig(self, port_number, bit_mask):
    '''
    This command sets the direction of the digital bits for a port
    
     port_number:    AUXPORT    =   0x10
                     Port A     =   0x01
                     Port B     =   0x04
    
     bit_mask   bit value:  0 = output,  1 = input
    Note: Bug in AUXPORT, take the one's complement
    '''

    if self.productID == 0x0075 and port_number == self.DIO_AUXPORT:
      bit_mask = ((bit_mask ^ 0xff) & 0xff)
    self.h.write([self.DCONFIG, port_number, bit_mask, 0, 0, 0, 0, 0])

  def DIn(self, port_number):
    '''
    This command reads the current state of the DIO port.  If the digital port
    is bitwise configurable, it may be any combination of inputs and outputs.
    The return value will be the value seen at the port pins.
    
     port_number:    AUXPORT    =   0x10
                     Port A     =   0x01
                     Port B     =   0x04
    Bug: need leading 0 in command string.
    '''

    self.h.write([0x0, self.DIN, port_number, 0, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(8,500)
    except:
      print('DIn: error in reading.')
    return(value[0])

  def DOut(self, port_number, value):
    '''
    This command writes data to the DIO port bits that are configured as outputs.
    
     port_number:    AUXPORT    =   0x10
                     Port A     =   0x01
                     Port B     =   0x04
     value:           value to write to the port
    '''
    self.h.write([self.DOUT, port_number, value, 0, 0, 0, 0, 0])

  def DBitIn(self, port_number, bit):
    '''
    This command reads an individual digital port bit.  It will return the value
    seen at the port pin, so may be used for an input or output bit.
    
     port_number:    AUXPORT    =   0x10
                     Port A     =   0x01
                     Port B     =   0x04
    
     bit:            The bit to read (0-7)
    '''
    
    self.h.write([self.DBIT_IN, port_number, bit, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(1,500)
    except:
      print('DBitIn: error in reading.')
    return(value[0])

  def DBitOut(self, port_number, bit, value):
    '''
    This command writes an individual digital port bit.  

     port_number:    AUXPORT    =   0x10
                     Port A     =   0x01
                     Port B     =   0x04
    
     bit:            The bit to read (0-7)
     value:          The value to write to the bit (0 or 1)
    '''
    self.h.write([self.DBIT_OUT, port_number, bit, value, 0, 0, 0, 0])
    
  #################################
  #    Analog Input  Commands     #
  #################################

  def AIn(self, channel, gain):
    '''
    This command reads the value from an analog input channel, setting the desired
    gain range first.  Note that single-ended inputs include 11 bits of resolution
    while differential inputs have 12 bits of resolution.  Also, the 'range' code
    used includes single-end/differential input setting.
    
       channel: the channel to read (0-3 differential, or 0-7 single ended)
       range:   the gain and input mode setting
    '''

    if (gain == self.SE_10_00V):
      mode = 0    # single ended
    else:
      mode = 1    # differential

    if (channel > 3 and mode == 1):
      print('AIn: channel out of range for differential mode.')
      return -1

    if (channel > 7 and mode == 0):
      print('AIn: channel out of range for single ended mode.')
      return -1

    self.h.write([self.AIN, channel, gain, 0, 0, 0, 0, 0])
    try:
      data = self.h.read(3, 100)
    except:
      print('AIn: error in reading A/D.')

    if (mode == 1):
      # Differential
      # The data is 2's compliment signed 12 bit number
      value = ((data[0]<<4)  | (data[1]<<8))
      value ,= unpack('h',pack('H',value))
      value /= 16
    else:
      # single-ended
      # the data is an 11 bit number signed offset
      value = ((data[1]<<4) | (data[0] & 0x0f))
      value -= 0x400
    return int(value)

  def AInScan(self, count, frequency,  nQueue, chanQueue, gainQueue, options):
    '''
    This command scans a range of analog input channels.  The
    channel configuration (low channel, high channel, and gain
    ranges) must be set with ALoadQueue.
    
    count:   the total number of samples to perform.  In continuous mode,
             count is equal to packet size (64)
    options: bit 0: 1 = single execution, 0 = continuous execution
             bit 1: 1 = burst mode
             bit 2: 1 = block transfer mode
             bit 3: 1 = use external trigger
    nQueue:  The number of channels in the load queue, must be 1, 2, 4, or 8
    chanQueue: list of channels in the queue
    gainQueue: listo of gains in the queue
    
    The external trigger may be used to start data collection synchronously.  In
    external trigger mode, wait for the device to send back notice (0xc3) then
    startup the acquisition
    
    In burst mode, wait for end-of-block acqusition (0xa5) then startup acquisition
    '''

    # Calculate timer_preload and timer_prescale values
    if (100 <= frequency and frequency < 200): 
      prescale = 7        # Select 256:1 prescalar
      setupTime = 0
    elif (200 <= frequency and frequency < 400):
      prescale = 6        # Select 128:1 prescalar
      setupTime = 0
    elif (400 <= frequency and frequency < 800):
      prescale = 5        # Select 64:1 prescalar
      setupTime = 0
    elif (800 <= frequency and frequency < 1500):
      prescale = 4        # Select 32:1 prescalar
      setupTime = 1
    elif (1500 <= frequency and frequency < 3000):
      prescale = 3        # Select 16:1 prescalar
      setupTime = 3
    elif (3000 <= frequency and frequency < 6000):
      prescale = 2        # Select 8:1 prescalar
      setupTime = 6
    elif (6000 <= frequency and frequency < 8192):
      prescale = 1        # Select 4:1 prescalar
      setupTime = 10
    timerMult = 1 << (prescale + 1)
    timerVal = ((256 - (self.MINILAB_CLOCK/(frequency*timerMult))) + 0.5)
    preload = int(timerVal + setupTime)
    prescale = int(prescale)

    # Load the AIn Scan Queue
    self.ALoadQueue(nQueue, chanQueue, gainQueue)

    self.h.write([self.AIN_SCAN, count & 0xff, (count>>8) & 0xff, preload,  prescale, options, 0, 0])

    # If in external trigger mode, then wait for the device to send back notice
    # that the trigger has been received, then startup the acquisition

    buf = [0, 0, 0, 0, 0, 0, 0, 0]
    if (options & self.AIN_TRIGGER):
      while (len(buf) > 0 and buf[0] != 0xc3):   # wait until external trigger received
        buf[0] = 0
        buf = self.h.read(8,1000)

    # If in burst mode, wait for end of block acquisition flag (0xa5)
    buf[0] = 0
    if (options & self.AIN_BURST_MODE):
      while (len(buf) > 0 and buf[0] != 0xa5):
        buf[0] = 0
        buf = self.h.read(8,1000)

    # use get_feature_report to collect the data buffer.  Each buffer will be 105
    # bytes long.  The first byte will contain the record number and can be ignored.
    # The following 96 byes will reqpresent 64 samples of data.
    idx = 0
    buffer = [0]*count
    self.scanIdx = 0

    while (count > 0):
      data = self.h.get_feature_report(0,105)  # get 64 samples
      self.scanIdx += 1
      scanIndex = data[102] | (data[103]<<8)
      try:
        if (scanIndex > self.scanIdx):
          raise OverrunError
      except OverRunError:
        print('AInScan: Overrun Error')
        return
      try:
        if (scanIndex < self.scanIdx):
          raise UnderrunError
      except UnderRunError:
        print('AInScan: Underrun Error')
        return
      # 12 bit signed data packed 2 samples in 3 bytes
      for i in range(0,96,3):
        value = ((data[i+1])  | (data[i+2]<<4) & 0x0f00)
        if (value & 0x800):
            value |= 0xf000
        buffer[idx] ,= unpack('h',pack('H',value))
        value = (data[i+3]  | ((data[i+2]<<8) & 0x0f00))
        if (value & 0x800):
          value |= 0xf000
        buffer[idx+1] ,= unpack('h',pack('H',value))
        idx += 2
        count -= 2
        if (count == 0):  # check to see if finished
          return buffer
    return buffer

  def AInStop():
    '''
    This command stops the analog scan (if running)
    '''
    self.h.write([self.AIN_STOP, 0, 0, 0, 0, 0, 0, 0])

  def ALoadQueue(self, count, chanQueue, gainQueue):
    # count must be 1, 2, 4 or 8
    if (count == 1 or count == 2 or count == 4 or count == 8):
      self.h.write([self.ALOAD_QUEUE, count, chanQueue[0] & 0x7 | gainQueue[0] | 0x80, \
                                             chanQueue[1] & 0x7 | gainQueue[1] | 0x80, \
                                             chanQueue[2] & 0x7 | gainQueue[2] | 0x80, \
                                             chanQueue[3] & 0x7 | gainQueue[3] | 0x80, \
                                             chanQueue[4] & 0x7 | gainQueue[4] | 0x80, \
                                             chanQueue[5] & 0x7 | gainQueue[5] | 0x80])

    if (count == 8): # configure the rest of the channels (channel 6 and 7)
      self.h.write([self.ALOAD_QUEUE, 0x2, chanQueue[6] & 0x7 | gainQueue[6] | 0x80, \
                                           chanQueue[7] & 0x7 | gainQueue[7] | 0x80, 0, 0, 0, 0])
  

  #################################
  #   Analog Output  Commands     #
  #################################

  def AOut(self, channel, value):
    '''
    This command sets the voltage output of the specified analog output channel
    
       channel:  selects output channel (0 or 1)
       value:    value (uint16) in counts to output [10-bits 0-5V]
    '''
    
    if (value > 0x3ff):
      value = 0x3ff
    if (value < 0):
      value = 0
    if (channel < 0 or channel > 1):
      print('AOut: channel out of range.')
      return
    self.h.write([self.AOUT, channel, (value&0xff), (value>>8)&0xff, 0, 0, 0, 0])

  #################################
  #     Counter  Commands         #
  #################################

  def CIn(self):
    '''
    This function reads the 32-bit event counter on the device.  This
    counter tallies the transitions of an external input attached to
    the CTR pin (pin 20) on the screw terminal of the device.
    '''
    
    self.h.write([self.CIN, 0, 0, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(4,100)
    except:
      print('Error in CIn.')
      return 0
    return (value[0] | (value[1]<<8) | (value[2]<<16) | (value[3]<<24))

  def CInit(self):
    ''' 
    This command initializes the event counter and resets the count to zero
    '''
    self.h.write([self.CINIT, 0, 0, 0, 0, 0, 0, 0])

  #################################
  #     Memory  Commands          #
  #################################

  def MemRead(self, address, count):
    '''
    This command reads data from the configuration memeory (EEPROM).
    
       address: the start address for the read.
               |-----------------------------------------|
               |    Range          |       Usage         |
               |-----------------------------------------|
               | 0x1f00 - 0x1f4f   | Offset Adjustments  |
               |-----------------------------------------|
               | 0x1f50 - 0x1f5f   | Single-Ended Gain   |
               |                   | Adjustments         |
               |-----------------------------------------|
               | 0x1f60 - 0x1f67   | Differential Gain   |
               |                   | Adjustments         |
               |-----------------------------------------|
               |0x1fa0 - 0x1fa3    | CAL pin voltage     |
               |-----------------------------------------|
    
     count: the number of bytes to read (maximum 8)
    '''

    if (count > 8):
      print('MemRead: max count is 8')
      return
    self.h.write([self.MEM_READ, address, count, 0, 0, 0, 0, 0])
    try:
      value = self.h.read(count, 100)
    except:
      print('Error in reading memory, value =', value)
    return(value[0:count])

  def MemWrite(self, address, count, data):
    '''
    This command writes data to the non-volatile EEPROM memory on the device.
    The non-volatile memory is used to store calibration coefficients, system
    information and user data.
    
    address: the start address to write.
         |-----------------------------------------|
         |    Range          |       Usage         |
         |-----------------------------------------|
         | 0x1f00 - 0x1f4f   | Offset Adjustments  |
         |-----------------------------------------|
         | 0x1f50 - 0x1f5f   | Single-Ended Gain   |
         |                   | Adjustments         |
         |-----------------------------------------|
         | 0x1f60 - 0x1f67   | Differential Gain   |
         |                   | Adjustments         |
         |-----------------------------------------|
         |0x1fa0 - 0x1fa3    | CAL pin voltage     |
         |-----------------------------------------|

    count: the number of bytes to read (maximum 4)
    data:  the data to be written (4 bytes max)
    '''

    if (count > 4):
      print('MemWrite: max count is 4')
      return
    self.h.write([self.MEM_WRITE, address, count, data[0:count]] + [0]*(5-count))


  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self):
    '''
     This command causes the LED to flash several times.
    '''
    self.h.write([self.BLINK_LED, 0, 0, 0, 0, 0, 0, 0])

  def Reset(self):
    '''
    The command causes the device to perform a soft reset. The device
    simulates a disconnect from the USB bus which in turn causes the
    host computer to re-enumerate the device.
    '''
    self.h.write([self.RESET, 0, 0, 0, 0, 0, 0, 0])

  def SetTrigger(self, type, chan=0):
    '''
    This command configures the external trigger for analog input.  The
    trigger may be configured to activate with either a logic rising
    edge or falling edge input.  Once the trigger is received, the analog
    input will proceed as configured.  The EXTTRIG option must be used
    in the AInScan command to utilize this feature.
    
     type:  the type of trigger  (0 = external trigger falling edge, 1 = external trigger rising edge)
     chan:  the digial bit that receives the trigger signal(0 for USB-120LS, 0-3 for miniLAB 1008)
    '''
    self.h.write([self.SET_TRIGGER, type, chan, 0, 0, 0, 0, 0, 0])

  def SetID(self, id):
    '''
    This command stores an identifier on the unit.  Values of 0-255 are valid.  Note that 0
    is used to flag uninitialized units.
    
    id: user id number (0-255)
    '''

    if id < 0 or id > 255:
      print('SetID: id out of range')
      return
    try:
      self.h.write([self.SET_ID, id, 0, 0, 0, 0, 0, 0])
    except:
      print('Error in writing id')
      
  def GetID(self):
    '''
    This function retrieves the id number stored on the device.  Note that a value of 0
    is used to flag an uninitialized device.
    '''
    try:
      self.h.write([self.GET_ID, 0, 0, 0, 0, 0, 0, 0])
    except:
      print('Error in reading id')
    try:
      value = self.h.read(8, 100)
    except:
      print('Error in reading id, value =', value)
    return(value[0])

  def volts(self, gain, num):
    '''
    converts raw values to volts
    '''
    volt = 0.0
    if gain == self.SE_10_00V:
      volt = num * 10.0 / 0x3ff
    elif gain == self.BP_20_00V:
      volt = num * 20.0 / 0x7ff
    elif gain == self.BP_10_00V:
      volt = num * 10.0 / 0x7ff
    elif gain == self.BP_5_00V:
      volt = num * 5.0 / 0x7ff
    elif gain == self.BP_4_00V:
      volt = num * 4.0 / 0x7ff
    elif gain == self.BP_2_50V:
      volt = num * 2.5 / 0x7ff
    elif gain == self.BP_2_00V:
      volt = num * 2.0 / 0x7ff
    elif gain == self.BP_1_25V:
      volt = num * 1.25 / 0x7ff
    elif gain == self.BP_1_00V:
      volt = num * 1.0 / 0x7ff
    
    return volt

class usb_1208LS(usb_1208):
  def __init__(self, serial=None):
    self.productID = 0x0007a           # USB-1208LS
    usb_1208.__init__(self)
    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      print('Can not open USB-1208LS')
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

    self.DConfig(self.DIO_PORTA, 0x00)  # Port A output
    self.DConfig(self.DIO_PORTB, 0xff)  # Port B input
    self.DOut(self.DIO_PORTA, 0x0)
    self.AOut(0,0x0)
    self.AOut(1,0x0)
      
class usb_miniLAB(usb_1208):
  DIO_AUXPORT   = 0x10          # miniLAB only DIO0-DIO3
  DIO_PORTCH    = 0x02          # miniLAB only
  DIO_PORTCL    = 0x08          # miniLAB only

  def __init__(self, serial=None):
    self.productID = 0x0075                      #MCC miniLAB 1008
    usb_1208.__init__(self)
    try:
      self.h.open(0x9db, self.productID, serial)
    except:
      print('Can not open USB-MiniLAB1008')
      return

    self.DConfig(self.DIO_AUXPORT, 0x3) # default DIO0 and DIO1 to output
