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

class CounterParameters:
  def __init__(self):
    self.counter = 0
    self.modeOptions = 0x0
    self.counterOptions = 0x0
    self.gateOptions = 0x0
    self.outputOptions = 0x0
    self.debounce = 0x0
    self.outputValue0 = 0x0
    self.outputValue1 = 0x0
    self.limitValue0 = 0x0    # Minimum Limit Value
    self.limitValue1 = 0x0    # Maximum Limit Value

class TimerParameters:
  def __init__(self):
    self.timer = 0
    self.period = 0
    self.pulseWidth = 0
    self.count = 0
    self.dely = 0
    self.control = 0x0

class usb_ctr(mccUSB):

  NTIMER   = 4      # Number of PWM Timers

  # Status bit values 
  PACER_RUNNING    = 0x1 << 1
  SCAN_OVERRUN     = 0x1 << 2
  SCAN_DONE        = 0x1 << 5
  FPGA_CONFIGURED  = 0x1 << 8
  FPGA_CONFIG_MODE = 0x1 << 9

 # Mode Register
  TOTALIZE            = 0x0
  PERIOD              = 0x1
  PULSEWIDTH          = 0x2
  TIMING              = 0x3
  PERIOD_MODE_1X      = 0x0
  PERIOD_MODE_10X     = 0x1 << 2
  PERIOD_MODE_100X    = 0x2 << 2
  PERIOD_MODE_1000X   = 0x3 << 2
  TICK_SIZE_20_83ns   = 0x0
  TICK_SIZE_208_3ns   = 0x1 << 4
  TICK_SIZE_2083_3ns  = 0x2 << 4
  TICK_SIZE_20833_3ns = 0x3 << 4  

 # Options Register
  CLEAR_ON_READ       = 0x1 << 0
  NO_RECYCLE          = 0x1 << 1
  COUNT_DOWN          = 0x1 << 2
  RANGE_LIMIT         = 0x1 << 3
  FALLING_EDGE        = 0x1 << 4

  # Commands and Report ID for USB-CTR
  ###########################################
  #  Digital I/O Commands 
  DTRISTATE           = 0x00  # Read/Write Tristate register
  DPORT               = 0x01  # Read digital port pins
  DLATCH              = 0x02  # Read/Write Digital port output latch register
  
  # Counter Commands 
  COUNTER              = 0x10  # Read or set the counter's value
  COUNTER_MODE         = 0x11  # Read or set the counter's mode
  COUNTER_OPTIONS      = 0x12  # Read or set the counter's options
  COUNTER_DEBOUNCE     = 0x13  # Read or set the counter's debounce options
  COUNTER_GATE_CONFIG  = 0x14  # Read or set the options for the Counter Gate
  COUNTER_OUT_CONFIG   = 0x15  # Read or set the options for the Counter Out
  COUNTER_OUT_VALUES   = 0x16  # Read or set the counter's output values
  COUNTER_LIMIT_VALUES = 0x17  # Read or set the limit values
  COUNTER_PARAMETERS   = 0x18  # Read or set all of one counter's parameters
  
  # Acquisition Comands
  SCAN_CONFIG          = 0x20  # Read/Write the scan list
  SCAN_START           = 0x21  # Start input scan
  SCAN_STOP            = 0x22  # Stop input scan
  SCAN_CLEAR_FIFO      = 0x23  # Clear data in the input FIFO
  BULK_FLUSH           = 0x24  # Flush the Bulk pipe

  # Timer Commands
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
  SERIAL               = 0x48  # Read/Write USB Serial Number

  # FPGA Configuration Commands
  FPGA_CONFIG          = 0x50 # Start FPGA configuration
  FPGA_DATA            = 0x51 # Write FPGA configuration data
  FPGA_VERSION         = 0x52 # Read FPGA version

  HS_DELAY = 2000

  def __init__(self):
    self.scanList = bytearray(33)
    self.lastElement = 0         # the last element of the scanlist
    self.continuous_mode = False

    # Find the maxPacketSize for bulk transfers
    self.wMaxPacketSize = self.getMaxPacketSize(libusb1.LIBUSB_ENDPOINT_IN | 0x6)  #EP IN 6

    # Set up the Timers
    self.timerParameters = [TimerParameters(), TimerParameters(), TimerParameters(), TimerParameters()]
    for timer in range(self.NTIMER):
      self.timerParameters[timer].timer = timer

    # Configure the FPGA
    if  not (self.Status() & self.FPGA_CONFIGURED) :
      # load the FPGA data into memory
      from usb_ctr_rbf import FPGA_data
      print("Configuring FPGA.  This may take a while ...")
      self.FPGAConfig()
      if self.Status() & self.FPGA_CONFIG_MODE:
        for i in range(0, len(FPGA_data) - 64, 64) :
          self.FPGAData(FPGA_data[i:i+64])
        if len(FPGA_data) % 64 :
          self.FPGAData(FPGA_data[i:i+len(FPGA_data)%64])
        if not self.Status() & self.FPGA_CONFIGURED:
          print("Error: FPGA for the USB-CTR is not configured.  status = ", hex(self.Status()))
          return
      else:
        print("Error: could not put USB-CTR into FPGA Config Mode.  status = ", hex(self.Status()))
        return
    else:
      print("USB-CTR FPGA configured.")
      return

  
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
    
  ##########################################
  #            Counter Commands            #
  ##########################################

  def CounterSet(self, counter, count):
    """
    This command reads or sets the value of the 64-bit counters.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER
    wValue = 0x0
    wIndex = counter & 0xff

    if counter > self.NCOUNTER:
      raise ValueError('CounterSet: counter out of range.')
      return

    count = pack('Q', count)
    self.udev.controlWrite(request_type, request, wValue, wIndex, count, self.HS_DELAY)

  def Counter(self, counter):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter & 0xff

    if counter > self.NCOUNTER:
      raise ValueError('Counter: counter out of range.')
      return

    value ,= unpack('Q',self.udev.controlRead(request_type, self.COUNTER, wValue, wIndex, 8, self.HS_DELAY))
    return value

  def CounterModeR(self, counter):
    """
    This command reads or sets the mode of the counter.
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
		20.83 ns   = 0
                208.3 ns   = 1
		2083.3 ns  = 2
		20833.3 ns = 3
      bits(6-7): Reserved 		
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterModeR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter
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

  def CounterOptionsR(self, counter):
    """
    This command reads or sets the options of the counter.
    Options:
      bit(0):   1 = Clear on Read,  0 = Read has no effect
      bit(1):   1 = No recycle mode (counter stops at 2^64 or 0, unless Range Limit is enabled)
                0 = counter rolls over to a minimum (or maximum) and continues counting.
      bit(2):   1 = Count down,  0 = Count up
      bit(3):   1 = Range Limit on (use max and min limits) 0 = 64-bit counter (max = 2^64, min = 0)
      bit(4):   1 = Count on falling edge,  0 = Count on rising edge
      bit(5-7): Reserved
    """
    
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOptionsR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter
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

  def CounterDebounceR(self, counter):
    """
    This command reads or sets the debounce options of a counter.
    Debounce:
      bits(0-4): No debounce:   0
                 500ns:         1
                 1500ns:        2
                 3500ns:        3
		 7500ns:        4
		 15500ns:       5
		 31500ns:       6
		 63500ns:       7
		 127500ns:      8
		 100us:         9
		 300us:        10
		 700us:        11
		 1500us:       12
		 3100us:       13
		 6300us:       14
		 12700us:      15
                 25500us:      16
      bit(5):    1 = Trigger before stable, 0 = Trigger after stable
      bits(6-7): Reserved
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterDebounceR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter
    debounce ,= unpack('B',self.udev.controlRead(request_type, self.COUNTER_DEBOUNCE, wValue, wIndex, 1, self.HS_DELAY))
    self.counterParameters[counter].debounce = debounce
    return debounce
    
  def CounterDebounceW(self, counter, debounce):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterDebounceW: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_DEBOUNCE
    wValue = debounce
    wIndex = counter
    self.counterParameters[counter].debounce = debounce
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def CounterGateConfigR(self, counter):
    """
    This command reads and sets the options of a counter's gate

    Options:
      bit(0):  1 = Enable Gate pin,  0 = Disable Gate pin
      bit(1):  1 = Active Low/Falling/Gate Low increments counter
               0 = Active High/Rising/Gate High increments counter
      bit(2-3): Gate Mode:
        0 = Gate state determines whether counter input is live (typical gate functionality)
        1 = Gate state determines the direction of the counter (instead of CounterOptions(bit 2))
        2 = Gate going active will clear counter
        3 = Gate Trigger: Counter In and Out pins are inactive until the active edge of the gate.
	    This can be used repeatedly with Stop at the Top set to provide controlled outputs.
      bits(4-7) Reserved
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterGateConfigR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter
    options ,= unpack('B',self.udev.controlRead(request_type, self.COUNTER_GATE_CONFIG, wValue, wIndex, 1, self.HS_DELAY))
    self.counterParameters[counter].gateOptions = options
    return options
    
  def CounterGateConfigW(self, counter, options):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterGateConfigW: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_GATE_CONFIG
    wValue = options
    wIndex = counter
    self.counterParameters[counter].gateOptions = options
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def CounterOutConfigR(self, counter):
    """
    The command reads or sets the options of a counter's output
    Options:
      bit(0): 1 = Output on,  0 = Output off
      bit(1): 1 = Initial State is High,  0 = Initial State is Low
      bits(2-7): Reserved
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOutConfigR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter
    options ,= unpack('B',self.udev.controlRead(request_type, self.COUNTER_OUT_CONFIG, wValue, wIndex, 1, self.HS_DELAY))
    self.counterParameters[counter].outputOptions = options
    return options
    
  def CounterOutConfigW(self, counter, options):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOutConfigW: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_OUT_CONFIG
    wValue = options
    wIndex = counter
    self.counterParameters[counter].outputOptions = options
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def CounterOutValuesR(self, counter, index):
    """
    This command reads or sets the counter's output values.  These
    values determine when the output changes state.  The output will
    change state when the counter reaches value0, and then change back
    when the counter reaches value1.  If both values are set to the
    same count, the output will only change state once - at that
    value.
      counter: the counter to set (0-7)
      index:   the index of the value to set (0-1)
               0 = Minimum Limit Value
               1 = Maximum Limit Value
      value:  when the counter reaches this value, the output changes state.
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOutValuesR: counter value too large.')
      return
    if index > 2:
      raise ValueError("CounterOutValuesR: index must be '0' or '1'.")
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = index
    wIndex = counter
    value ,= unpack('Q',self.udev.controlRead(request_type, self.COUNTER_OUT_VALUES, wValue, wIndex, 8, self.HS_DELAY))
    if index == 0:
      self.counterParameters[counter].outputValue0 = value
    else:
      self.counterParameters[counter].outputValue1 = value
    return value
    
  def CounterOutValuesW(self, counter, index, value):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterOutValuesW: counter value too large.')
      return
    if index > 2:
      raise ValueError("CounterOutValuesW: index must be '0' or '1'.")
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_OUT_VALUES
    wValue = index
    wIndex = counter
    value = pack('Q', value)
    self.udev.controlWrite(request_type, request, wValue, wIndex, value, self.HS_DELAY)

  def CounterLimitValuesR(self, counter, index):
    """
    This command reads or sets the counter's limit values.
      index: 0 =  Minimum Limit Value, 1 = Maximum Limit Value 
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
    wIndex = counter
    value ,= unpack('Q',self.udev.controlRead(request_type, self.COUNTER_LIMIT_VALUES, wValue, wIndex, 8, self.HS_DELAY))
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
    wIndex = counter
    value = pack('Q',value)

    self.udev.controlWrite(request_type, request, wValue, wIndex, value, self.HS_DELAY)

  def CounterParamsR(self, counter):
    """
    This command reads or writes all of a given counter's parameters in one call
    """
    if counter >= self.NCOUNTER:
      raise ValueError('CounterParamsR: counter value too large.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = counter
    data = unpack('BBBBB',self.udev.controlRead(request_type, self.COUNTER_PARAMETERS, wValue, wIndex, 5, self.HS_DELAY))
    self.counterParameters[counter].counter = counter
    self.counterParameters[counter].modeOptions = data[0]
    self.counterParameters[counter].counterOptions = data[1]
    self.counterParameters[counter].gateOptions = data[2]
    self.counterParameters[counter].outputOptions = data[3]
    self.counterParameters[counter].debounce = data[4]
    return

  def CounterParamsW(self, counter):
    if counter >= self.NCOUNTER:
      raise ValueError('CounterParamsR: counter value too large.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.COUNTER_PARAMETERS
    wValue = 0x0
    wIndex = counter
    modeOptions = self.counterParameters[counter].modeOptions       & 0xff
    counterOptions = self.counterParameters[counter].counterOptions & 0xff
    gateOptions = self.counterParameters[counter].gateOptions       & 0xff
    outputOptions = self.counterParameters[counter].outputOptions   & 0xff
    debounce = self.counterParameters[counter].debounce             & 0xff
    s_buffer = [ modeOptions, counterOptions, gateOptions, outputOptions, debounce ]
    
    self.udev.controlWrite(request_type, request, wValue, wIndex, s_buffer, self.HS_DELAY)

  ##########################################
  #       Acquisition  Commands            #
  ##########################################

  def ScanConfigR(self):
    """
   This command reads or writes the input channel configurations.  This command will result
    in a bus stall if a scan is currently running.  The scan list is setup to acquire data
    from the channels in the order that they are placed in the scan list.

    Each counter has 4 banks of 16-bit registers that can be scanned in any order.
    Bank 0 contains bit 0-15, Bank 1 contains bit 16-31, Bank 2 contains bit 32-47
    and Bank 4 contains bit 48-63.  If bit(5) is set to 1, this element will
    be a read of the DIO, otherwise it will use the specified counter and bank.

    ScanList[33] channel configuration:
      bit(0-2): Counter Number (0-7)
      bit(3-4): Counter Bank (0-3)
      bit(5): 1 = DIO,  0 = Counter
      bit(6): 1 = Fill with 16-bits of 0's,  0 = normal (This allows for the creation
      of a 32 or 64-bit element of the DIO when it is mixed with 32 or 64-bit elements of counters)
     """
    
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = self.lastElement
    value = self.udev.controlRead(request_type, self.SCAN_CONFIG, wValue, wIndex, 33, self.HS_DELAY)
    for i in range(len(value)):
      self.scanList[i] = value[i]
    return

  def ScanConfigW(self):
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SCAN_CONFIG
    wValue = 0x0
    wIndex = self.lastElement
    self.udev.controlWrite(request_type, request, wValue, wIndex, self.scanList, self.HS_DELAY)
    return

  def ScanStart(self, count, retrig_count, frequency, options):
    """
    count:         the total number of scans to perform (0 for continuous scan)
    retrig_count:  the number of scans to perform for each trigger in retrigger mode
    pacer_period:  pacer timer period value (0 for external clock)
    packet_size:   number of samples - 1 to transfer at a time.
    options:       bit field that controls various options
      bit 0:   1 = Maintain counter value on scan start, 0 = Clear counter value on scan start
      bit 1:   Reserved
      bit 2:   Reserved
      bit 3:   1 = use trigger
      bit 4:   Reserved
      bit 5:   Reserved
      bit 6:   1 = retrigger mode,  0 = normal trigger
      bit 7:   Reserved
 
    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 96 MHz.  The timer is controlled by
    pacer_period.  A pulse will be output at the PACER_OUT pin every
    pacer_period interval regarless of mode.

    If pacer_period is set to 0, the device does not generate an A/D
    clock.  It uses the PACER_IN pin as the pacer source.

    The timer will be reset and sample acquired when its value equal
    timer_period.  The equation for calculating timer_period is:

           timer_period = [96 MHz / (sample frequency)]  - 1

    The data will be returned in packets utilizing a bulk IN endpoint.
    The data will be in the format:

    lowchannel sample 0:  low channel + 1 sample 0: ... : hichannel sample 0
    lowchannel sample 1:  low channel + 1 sample 1: ... : hichannel sample 1
    ...
    lowchannel sample n:  low channel + 1 sample n: ... : hichannel sample n
    
    The scan will not begin until the ScanStart command is sent (and
    any trigger conditions are met.)  Data will be sent until reaching
    the specified count or an ScanStop command is sent.

    The packet_size parameter is used for low sampling rates to avoid
    delays in receiving the sampled data.  The buffer will be sent,
    rather than waiting for the buffer to fill.  This mode should not
    be used for high sample rates in order to avoid data loss.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate.  No messages will be sent until the trigger is
    detected.

    The external trigger may be used to start data collection
    synchronously.  If the bit is set, the device will wait until the
    appropriate trigger edge is detected, then begin sampling data at
    the specified rate.  No messages will be sent until the trigger is
    detected.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SCAN_START
    wValue = 0x0
    wIndex = 0x0

    self.frequency = frequency
    self.options = options

    if frequency == 0:
      pacer_period = 0
    else:
      pacer_period = int((96.E6/frequency) - 1)

    if count == 0:    # continuous mode
      self.continuous_mode = True
    else:
      self.continuous_mode = False

    data = pack('IIIBB', count, retrig_count, pacer_period, (self.wMaxPacketSize-1)&0xff, options&0xff)
    self.udev.controlWrite(request_type, request, wValue, wIndex, data, self.HS_DELAY)

  def ScanRead(self, count):
    if self.continuous_mode:
      nSamples = int(self.wMaxPacketSize/2)
    else:
      nSamples = count*(self.lastElement+1)
    time_delay = int(self.HS_DELAY + 1000*nSamples/self.frequency)
    data = []
    try:
     data = unpack('H'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, int(nSamples*2), time_delay))
    except:
      print("Error in ScanRead.  Bytes received = ", len(data), "nSamples = ", nSamples)

    status = self.Status()
    if status & self.SCAN_OVERRUN:
      raise OverrunERROR

    if self.continuous_mode:
      return list(data)

    # if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
    if ((int(nSamples*2) % self.wMaxPacketSize) == 0 and  not(status & self.PACER_RUNNING)):
      data2 = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 2, 100)

    self.ScanStop()
    self.ScanClearFIFO()
    self.BulkFlush(5)
    
    return list(data)

  def ScanStop(self):
    """
    This command stops the input scan (if running)
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SCAN_STOP
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex,[0x0], self.HS_DELAY)

  def ScanClearFIFO(self):
    """
    This command clears the input firmware buffer
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.SCAN_CLEAR_FIFO
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex,[0x0], self.HS_DELAY)

  def BulkFlush(self, count=1):
    """
    This command flushes the Bulk pipe a number of times.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.BULK_FLUSH
    wValue = count
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex,[0x0], self.HS_DELAY)
    
  ##########################################
  #             Timer Commands             #
  ##########################################

  def TimerControlR(self, timer):
    """
    This command reads/writes the timer control register
      timer:     the timer selected (0-3)
      control:   bit 0:    1 = enable timer,  0 = disable timer
                 bit 1:    1 = timer running, 0 = timer stopped
                          (read only, useful when using count)
                 bit 2:    1 = inverted output (active low)
                           0 = normal output (active high)
                 bits 3-7: reserved
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
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def TimerPeriodR(self, timer):
    """
    This command reads or writes the given timer period register.

    The timer is based on a 96 MHz input clock and has a 32-bit period register. The
    frequency of the output is set to:

    frequency = 96 MHz / (period + 1)

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.  This results in a minimum
    allowable value for the period of 1, which sets the maximum frequency to 96 MHz/2.
    """
    if timer > self.NTIMER:
      raise ValueError('TimerPeriodR: timer out of range')
      return
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = timer
    period ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_PERIOD, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters[timer].period = period
    return period

  def TimerPeriodW(self, timer, period):
    if timer > self.NTIMER:
      raise ValueError('TimerPeriodW: timer out of range')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_PERIOD
    wValue = 0x0
    wIndex = timer
    period = pack('I', period)
    self.udev.controlWrite(request_type, request, wValue, wIndex, period, self.HS_DELAY)

  def TimerPulseWidthR(self, timer):
    """
    This command reads/writes the timer pulse width register.
    The timer is based on a 96 MHz input clock and has a 32-bit pulse width register.
    The width of the output pulse is set to:

    pulse width = (pulseWidth + 1) / 96 MHz

    Note that the value for pulseWidth should always be smaller than the value for
    the period register or you may get unexpected results.
    """
    if timer > self.NTIMER:
      raise ValueError('TimerPulseWidthR: timer out of range')
      return
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = timer
    pulse_width ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_PULSE_WIDTH, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters[timer].pulseWidth = pulse_width
    return pulse_width

  def TimerPulseWidthW(self, timer, pulse_width):
    if timer > self.NTIMER:
      raise ValueError('TimerPulseWidthW: timer out of range')
      return
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_PULSE_WIDTH
    wValue = 0x0
    wIndex = timer
    pulse_width = pack('I', pulse_width)
    self.udev.controlWrite(request_type, request, wValue, wIndex, pulse_width, self.HS_DELAY)
    
  def TimerCountR(self, timer):
    """
    This command reads/writes the timer count register.
    The number of output pulses can be controlled with the count register.  Setting
    this register to 0 will result in pulses being generated until the timer is disabled.
    Setting it to a non-zero value will results in the specified number of pulses being
    generated then the output will go low until the timer is disabled.
    """
    if timer > self.NTIMER:
      raise ValueError('TimerCountR: timer out of range')
      return
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = timer
    count ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_COUNT, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters[timer].count = count
    return count

  def TimerCountW(self, timer, count):
    if timer > self.NTIMER:
      raise ValueError('TimerCountW: timer out of range')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_COUNT
    wValue = 0x0
    wIndex = timer
    count = pack('I', count)
    self.udev.controlWrite(request_type, request, wValue, wIndex, count, self.HS_DELAY)
    return count

  def TimerStartDelayR(self, timer):
    """
    This command reads/writes the timer start delay register.  This
    register is the amount of time to delay before starting the timer
    output after enabling the output.  The value specifies the number
    of 64 MHZ clock pulses to delay.  This value may not be written
    while the timer output is enabled.
    """
    if timer > self.NTIMER:
      raise ValueError('TimerStartDelayR: timer out of range')
      return
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = timer
    delay ,= unpack('I', self.udev.controlRead(request_type, self.TIMER_START_DELAY, wValue, wIndex, 4, self.HS_DELAY))
    self.timerParameters[timer].delay = delay
    return delay

  def TimerStartDelayW(self, timer, delay):
    if timer > self.NTIMER:
      raise ValueError('TimerStartDelayW: timer out of range')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_START_DELAY
    wValue = 0x0
    wIndex = timer
    delay = pack('I', delay)
    self.udev.controlWrite(request_type, request, wValue, wIndex, delay, self.HS_DELAY)

  def TimerParamsR(self, timer):
    """
    This command reads/writes all timer parameters in one call.
    See the individual command descriptions for futher information
    on each parameter.
    """
    if timer > self.NTIMER:
      raise ValueError('TimerParamsR: timer out of range')
      return
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = timer
    data = unpack('IIII', self.udev.controlRead(request_type, self.TIMER_PARAMETERS, wValue, wIndex, 16, self.HS_DELAY))
    self.timerParameters[timer].period = data[0]
    self.timerParameters[timer].pulseWidth = data[1]
    self.timerParameters[timer].count = data[2]
    self.timerParameters[timer].delay = data[3]
    self.TimerControlW(timer, self.timerParameters[timer].control)
    return 

  def TimerParamsW(self, timer):
    if timer > self.NTIMER:
      raise ValueError('TimerParamsW: timer out of range')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TIMER_START_DELAY
    wValue = 0x0
    wIndex = timer
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

    The range from 0x0000 to 0x6FFF is used for storing the
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
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex, data, self.HS_DELAY)

  def MemAddressR(self):
    """
    This command reads or writes the address used for memory accesses.
    The upper byte is used to denominate different memory areas.  The
    memory map for this device is

    Address                            Description
    =============               ============================
    0x0000-0x6FFF               Microcontroller firmware (write protected)
    0X7000-0X7FFF               User data

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
    wValue = 0x0
    wIndex = 0x0
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
    wValue = 0x0
    wIndex = 0x0
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
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [count], self.HS_DELAY)

  def Reset(self):
    """
    This function causes the device to perform a reset.  The device
    disconnects from the USB bus and resets its microcontroller.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.RESET
    wValue = 0x0
    wIndex = 0x0
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
    wValue = 0x0
    wIndex = 0x0
    value ,= unpack('B',self.udev.controlRead(request_type, self.TRIGGER_CONFIG, wValue, wIndex, 1, self.HS_DELAY))
    return value

  def GetSerialNumber(self):
    """
    This commands reads the device USB serial number.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal digits
    (i.e. "00000001").
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
    wValue = 0x0
    wIndex = 0x0
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
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex, data, self.HS_DELAY)

  def FPGAVersion(self):
    """
    This command reads the FPGA version.  The version is in
    hexadecimal BCD, i.e. 0x0102 is version 01.02.
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0x0
    wIndex = 0x0
    version ,= unpack('H',self.udev.controlRead(request_type, self.FPGA_VERSION, wValue, wIndex, 2, self.HS_DELAY))
    return "{0:02x}.{1:02x}".format((version>>8)&0xff, version&0xff)

class usb_ctr04(usb_ctr):
  NCOUNTER = 4      # Number of Counters
  USB_CTR04_PID  =  0x012E
  
  def __init__(self, serial=None):
    self.productID = 0x012E  # usb-ctr04
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-CTR04 not found")
      return
    
    self.counterParameters = [CounterParameters(), CounterParameters(), CounterParameters(), CounterParameters()]
    for i in range(self.NCOUNTER):
      self.counterParameters[i].counter = i

    usb_ctr.__init__(self)

class usb_ctr08(usb_ctr):
  NCOUNTER = 8      # Number of Counters
  USB_CTR08_PID  =  0x0127

  def __init__(self, serial=None):
    self.productID = 0x0127   # usb-ctr08
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-CTR08 not found")
      return
    
    self.counterParameters = [CounterParameters(), CounterParameters(), CounterParameters(), CounterParameters(), \
                             CounterParameters(), CounterParameters(), CounterParameters(), CounterParameters()]
    for counter in range(self.NCOUNTER):
      self.counterParameters[counter].counter = counter

    usb_ctr.__init__(self)
