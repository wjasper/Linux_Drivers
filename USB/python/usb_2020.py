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

class usb_2020(mccUSB):
  """
    Calibration memory map
  |===================================================================|
  |     Address     |               Value                             |
  |===================================================================|
  | 0x0000 - 0x6FFF | Microprocessor Firmware Code                    |
  |-------------------------------------------------------------------|
  | 0x7000 - 0x7003 | +/- 10V Single Ended Slope AI CH0               |
  |-------------------------------------------------------------------|
  | 0x7004 - 0x7007 | +/- 10V Single Ended Offset AI CH0              |
  |-------------------------------------------------------------------|
  | 0x7008 - 0x700B | +/- 5V Single Ended Slope AI CH0                |
  |-------------------------------------------------------------------|
  | 0x700C - 0x700F | +/- 5V Single Ended Offset AI CH0               |
  |-------------------------------------------------------------------|
  | 0x7010 - 0x7013 | +/- 2V Single Ended Slope AI CH0                |
  |-------------------------------------------------------------------|
  | 0x7014 - 0x7017 | +/- 2V Single Ended Offset AI CH0               |
  |-------------------------------------------------------------------|
  | 0x7018 - 0x701B | +/- 1V Single Ended Slope AI CH0                |
  |-------------------------------------------------------------------|
  | 0x701C - 0x701F | +/- 1V Single Ended Offset AI CH0               |
  |-------------------------------------------------------------------|
  | 0x7020 - 0x7023 | +/- 10V Single Ended Slope AI CH1               |
  |-------------------------------------------------------------------|
  | 0x7024 - 0x7027 | +/- 10V Single Ended Offset AI CH1              |
  |-------------------------------------------------------------------|
  | 0x7028 - 0x702B | +/- 5V Single Ended Slope AI CH1                |
  |-------------------------------------------------------------------|
  | 0x702C - 0x702F | +/- 5V Single Ended Offset AI CH1               |
  |-------------------------------------------------------------------|
  | 0x7030 - 0x7033 | +/- 2V Single Ended Slope AI CH1                |
  |-------------------------------------------------------------------|
  | 0x7034 - 0x7037 | +/- 2V Single Ended Offset AI CH1               |
  |-------------------------------------------------------------------|
  | 0x7038 - 0x703B | +/- 1V Single Ended Slope AI CH1                |
  |-------------------------------------------------------------------|
  | 0x703C - 0x703F | +/- 1V Single Ended Offset AI CH1               |
  |-------------------------------------------------------------------|
  | 0x7040 - 0x7043 | 0V Voltage Reference                            |
  |-------------------------------------------------------------------|
  | 0x7044 - 0x7047 | +10V Voltage Reference                          |
  |-------------------------------------------------------------------|
  | 0x7048 - 0x704B | -10V Voltage Reference                          |
  |-------------------------------------------------------------------|
  | 0x704C - 0x704F | +5V Voltage Reference                           |
  |-------------------------------------------------------------------|
  | 0x7050 - 0x7053 | -5V Voltage Reference                           |
  |-------------------------------------------------------------------|
  | 0x7054 - 0x7057 | +2V Voltage Reference                           |
  |-------------------------------------------------------------------|
  | 0x7058 - 0x705B | -2V Voltage Reference                           |
  |-------------------------------------------------------------------|
  | 0x705C - 0x705F | +1V Voltage Reference                           |
  |-------------------------------------------------------------------|
  | 0x7060 - 0x7063 | -1V Voltage Reference                           |
  |-------------------------------------------------------------------|
  | 0x7064 - 0x7097 | Reserved                                        |
  |-------------------------------------------------------------------|
  | 0x7098 - 0x7098 | Manufacturing Cal Date, Year (year - 2000)      |
  |-------------------------------------------------------------------|
  | 0x7099 - 0x7099 | Manufacturing Cal Date, Month                   |
  |-------------------------------------------------------------------|
  | 0x709A - 0x709A | Manufacturing Cal Date, Day                     |
  |-------------------------------------------------------------------|
  | 0x709B - 0x709B | Manufacturing Cal Date, Hour                    |
  |-------------------------------------------------------------------|
  | 0x709C - 0x709C | Manufacturing Cal Date, Minute                  |
  |-------------------------------------------------------------------|
  | 0x709D - 0x709D | Manufacturing Cal Date, Second                  |
  |-------------------------------------------------------------------|
  | 0x709E - 0x70FF | Reserved                                        |
  |-------------------------------------------------------------------|
  | 0x7100 - 0x71FF | User Data                                       |
  |===================================================================|
  """

  USB_2020_PID = 0x011c

  # Gain Ranges
  BP_10V = 0    # +/- 10. V
  BP_5V  = 1    # +/-  5. V
  BP_2V  = 2    # +/-  2. V
  BP_1V  = 3    # +/-  1. V

  # Status Bits
  AIN_SCAN_RUNNING   = 0x2   # AIn pacer running
  AIN_SCAN_OVERRUN   = 0x4   # AIn scan overrun
  AIN_SCAN_DONE      = 0x20  # AIn scan done
  FPGA_CONFIGURED    = 0x100 # FPGA is configured
  FPGA_CONFIG_MODE   = 0x200 # FPGA config mode

  NCHAN              = 2     # max number of A/D channels in the device (single_ended)
  NGAIN              = 4     # max number of gain levels
  MAX_PACKET_SIZE_HS = 512   # max packet size for HS device
  MAX_PACKET_SIZE_FS = 64    # max packet size for HS device
  LAST_CHANNEL       = 0x08  # Last channel
  CALIBRATION_MODE   = 0x10  # Calibration Mode

  # Options for AInScan
  TRIGGER             = 0x8  # Use trigger or gate
  PACER_OUT           = 0x20 # 1 = External Pacer Output, 0 = External Pacer Input
  RETRIGGER           = 0x40 # 1 = retrigger mode, 0 = normal trigger
  DDR_RAM             = 0x80 # 1 = Use DDR RAM as storage (BURSTIO) , 0 = Stream via USB

  # AIn Scan Modes
  CONTINUOUS_READOUT   = 0x1  # Continuous mode
  SINGLEIO             = 0x2  # Return data after every read (used for low frequency scans)
  FORCE_PACKET_SIZE    = 0x4  # Force packet_size
  VOLTAGE              = 0x8  # return values as voltages

  # Commands and Codes for USB-2020
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

  def __init__(self, serial=None):
    self.productID = self.USB_2020_PID    # usb-2020
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-2020 not found")
      return
    self.status = 0                       # status of the device
    self.samplesToRead = -1               # number of bytes left to read from a scan
    self.scanList = bytearray(self.NCHAN) # depth of scan queue is 2
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
      from usb_2020_rbf import FPGA_data
      print("Configuring FPGA.  This may take a while ...")
      self.FPGAConfig()
      if self.Status() & self.FPGA_CONFIG_MODE:
        for i in range(0, len(FPGA_data) - len(FPGA_data)%64, 64) :
          self.FPGAData(FPGA_data[i:i+64])
        i += 64
        if len(FPGA_data) % 64 :
          self.FPGAData(FPGA_data[i:i+len(FPGA_data)%64])
        if not self.Status() & self.FPGA_CONFIGURED:
          print("Error: FPGA for the USB-2020 is not configured.  status = ", hex(self.Status()))
          return
      else:
        print("Error: could not put USB-2020 into FPGA Config Mode.  status = ", hex(self.Status()))
        return
    else:
      print("USB-2020 FPGA configured.")

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
    #   self.table_AIn[channel][gain]    0 <= channel < 2    0 <= gain < 4
    #
    self.table_AIn = [[table(), table(), table(), table()], \
                      [table(), table(), table(), table()]]
    address = 0x7000
    self.MemAddressW(address)
    for channel in range(self.NCHAN):
      for gain in range(self.NGAIN):
        self.table_AIn[channel][gain].slope, = unpack('f', self.MemoryR(4))
        self.table_AIn[channel][gain].intercept, = unpack('f', self.MemoryR(4))

  def CalDate(self):
    """
    Get the manufacturers calibration data (timestamp) from the
    Calibration memory.

    Note: The calibration date is stored in the EEPROM
    starting at address 0x7098.  The six date elements
    (year, month, day, hour, minute, second) are stored
    as unsigned char.
    """

    address = 0x7098   # staring address of the MFG calibration date
    self.MemAddressW(address)

    data = unpack('BBBBBB', self.MemoryR(6))  # 6 bytes
    mdate = datetime(data[0]+2000, data[1], data[2], data[3], data[4], data[5])
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

  def AIn(self, channel, gain, voltage=False):
    """
    This command reads the value of an analog input channel.  This
    command will result in a bus stall if an AInScan is currently
    running.  

     channel: the channel to read (0-1)
     value:   12 bits of data, right justified.
     voltage: True = return voltage, False = return raw reading
    """

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue =  channel
    wIndex = 0
    
    if channel < 0 or channel >= self.NCHAN:
      raise ValueError('AIn: Error in channel value')
      return

    if gain < 0 or gain >= self.NGAIN:
      raise ValueError('AIn: Error in gain value')
      return
    
    data ,= unpack('H',self.udev.controlRead(request_type, self.AIN, wValue, wIndex, 2, timeout = 200))

    # only first 12 bits of data are significant
    if data > 0xffd:
      raise ValueError("DAC is saturated at +FS")
    elif data < 0x60:
      raise ValueError("DAC is saturated at -FS")
    else:
      data = round(float(data)*self.table_AIn[channel][gain].slope + self.table_AIn[channel][gain].intercept)

    if voltage == True:
      data = self.volts(gain, value)

    return data

  def AInScanStart(self, count, retrig_count, frequency, options, mode):
    """
    This command starts the analog input channel scan.  The gain
    ranges that are currently set on the desired channels will be used
    (these may be changed with AInConfig) This command will result in
    a bus stall if an AInScan is currently running.  The data on this
    devie goes straight into DRAM memory and will be uploaded over USB
    when the count is reached, or ethe meory becomes full.

    count:        the total number of scans to perform (0 for continuous scan)
    retrig_count: the number of scans to perform for each trigger in retrigger mode
    pacer_period: pacer timer period value (0 for PACER_IN)
    frequency:    pacer frequency in Hz
    packet_size:  number of samples to transfer at a time
    options:      bit field that controls various options
                   bit 0: Reserved
                   bit 1: Reserved
                   bit 2: Reserved
                   bit 3: 1 = use trigger or gate  0 = no trigger
                   bit 4: Reserved
                   bit 5: 1 = External Pacer Output, 0 = External Pacer Input
                   bit 6: 1 = retrigger mode, 0 = normal trigger
                   bit 7: 1 = Use DDR RAM as storage, 0 = Stream via USB

    mode:         mode bits:
                  bit 0:  0 = counting mode,  1 = CONTINUOUS_READOUT
                  bit 1:  1 = SINGLEIO
                  bit 2:  1 = use packet size passed usb_2020->packet_size
                  bit 3:  1 = convert to voltages  
    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 80 MHz.  The timer is controlled by
    pacer_period. The maximum rate on this ADC is 20 MHz.

    If pacer_period is set to 0 the device does not generate an A/D clock.  It uses
    PACER_IN pin as the pacer source.

    The timer will be reset and sample acquired when its value equals
    timer_period.  The equation for calculating timer_period is:

         timer_period = [80MHz / (sample frequency)] - 1

    The data will be returned in packets utilizing a bulk IN endpoint.
    The data will be in the format:

      lowchannel sample 0: lowchannel + 1 sample 0: ... :hichannel sample 0
      lowchannel sample 1: lowchannel + 1 sample 1: ... :hichannel sample 1
      ...
      lowchannel sample n: lowchannel + 1 sample n: ... :hichannel sample n

    The scan will not begin until the AInScanStart command is sent (and
    any trigger conditiions are met.)  Data will be sent until reaching
    the specified count or an AInScanStop command is sent.

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

    if  frequency > 20000000: # 20MHz S/s throughput
      frequency = 20.E6

    if frequency == 0.0:
      pacer_period = 0     # use external pacer
    else:
      pacer_period = round((80.E6 / frequency) - 1)

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

    if options & self.DDR_RAM:
      # If using the onboard DDR RAM (BURSTIO Mode), there are 3 constraints:
      # 1. The total count must be greater than or equal to 256
      # 2. The total count must be less than 64 MB
      # 3. The total count must be a multiple of 256
      if count < 256:
        raise ValueError('AInScanStart: count less than 256 in BURSTIO mode.')
        return
      if count > 64*1024*1024:
        raise ValueError('AInScanStart: count grester then 64MB.')
        return
      if count%256 != 0:
        raise ValueError('AInScanStart: count must be a multiple of 256.')
        return
    packet_size -= 1  # force to uint8_t size in range 0-255    
    scanPacket = pack('IIIBB', count, retrig_count, pacer_period, packet_size, options)
    result = self.udev.controlWrite(request_type, self.AIN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

    self.status = self.Status()

  def AInScanRead(self):
    """
    Bulk transactions will be considered complete when a packet is sent
    that is less than the max packet size.  If an integer muliple of the
    max packet size of data is to be sent during a transaction, an empty
    packet must be sent to indicate the end of the transaction.
    """

    if self.mode & self.CONTINUOUS_READOUT or self.mode & self.SINGLEIO :
      nSamples = self.packet_size
    else:
      nSamples = self.count*(self.lastElement+1)

    if self.options & self.DDR_RAM:
      timeout = 0
    else:
      timeout = round(500 + 2000*nSamples/self.frequency)

    try:
      data =  list(unpack('H'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, int(2*nSamples), timeout)))
    except:
      print('AInScanRead: error in bulkRead.')
      raise
      return
    
    if len(data) != nSamples:
      raise ValueError('AInScanRead: error in number of samples transferred.')
      return len(data)
    
    if self.mode & self.VOLTAGE:
      for i in range(len(data)):
        gain = (self.scanList[i%(self.lastElement+1)] >> 2) & 0x3
        channel = (self.scanList[i%(self.lastElement+1)]) & 0x1
        data[i] = data[i]*self.table_AIn[channel][gain].slope + self.table_AIn[channel][gain].intercept
        if data[i] > 0xfff:
          data[i] = 0xfff
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

    ScanList[2]  channel configuration:
      bit    0: Channel Number
      bits 1-2: Range 
      bit 3:    Last Channel: 1 = last channel, 0 = not last channel
      bit 4:    Calibration Mode: 1 = Calibration voltage reference is input to ADC, 0 = Channel input is tied to ADC

    Range:  00 = +/- 10V
            01 = +/-  5V
            10 = +/-  2V
            11 = +/-  1V
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = self.udev.controlRead(request_type, self.AIN_SCAN_CONFIG, wValue, wIndex, 2, self.HS_DELAY)
    for i in range(len(value)):
      self.scanList[i] = value[i]
      if value[i] & self.LAST_CHANNEL:
        self.lastElement = i  # depth of queue (either 0 or 1)
    return

  def AInConfigW(self, entry, channel, gain, lastElement=False, calibrationMode=False):
    """
    entry:    channel entry in the queue (0 - 1)
    channel:  channel number (Differential: 0-7, Single Ended: 0-15)
    gain:     range  ( 0:+/- 10V,  1: +/- 5V, 2: +/- 1V, 3: +/- 1V)
    lastElement: Set to True if last element in the queue
    """

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.AIN_SCAN_CONFIG
    wValue = 0
    wIndex = 0

    if entry < 0 or entry >= self.NCHAN:
      raise ValueError('AInConfigW: Exceed depth of queue')
      return

    if channel < 0 or channel >= self.NCHAN:
      raise ValueError('AInConfigW: Invalid channel number')
      return

    if self.Status() & self.AIN_SCAN_RUNNING:
      self.AInScanStop()

    self.scanList[entry] = channel | (gain << 0x1)

    if lastElement == True:
      if entry >= self.NCHAN:
        raise ValueError('AInConfigW: lastelement exceeds scan queue depth.')
      else:
        self.lastElement = entry
        self.scanList[entry] |= self.LAST_CHANNEL

    if calibrationMode == True:
      self.scanList[entry] |= self.CALIBRATION_MODE

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
    0x0000-0x6FFF               Microcontroller firmware (write protected)
    0x7000-0x70FF               Caliabration Storage (write protected)
    0x7100-0x7FFF               User data

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

  def TriggerConfig(self, options, trigger_channel, low_threshold, high_threshold):
    """
    This function configures the AInScan trigger or gaiting function.
    Once the trigger is received, the AInScan will proceed as
    configured.  If gaiting is used, the scan will only continue when
    the gate condition is met.  The High and Low thresholds will need
    to be in uncalibrated counts.  The"use trigger" option must be
    used in the AInScanStart command to utilize this feature.

      options:     bit 0:    Trigger Type (0 = Trigger, 1 = Gate)
                   bit 1:    Trigger Source (0 = Digital, 1 = Analog)
                   bits 2-3: Mode (00 = level, 01 = edge, 10 = Hysteresis, 11 = Window)
                   bit 4:    Polarity (0=low/falling/negative/out, 1 = high/rising/positive/in)
                   bits 5-7: reserved
     Trigger Channel: Channel number (0-1)
     Low Threshold:  Uncalibrated counts, only 12 bits are used
     High Threshold: Uncalibrated counts, only 12 bits are used
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.TRIGGER_CONFIG
    wValue = 0x0
    wIndex = 0x0
    values = pack('BBHH',options,trigger_channel,low_threshold, high_threshold)
    self.udev.controlWrite(request_type, request, wValue, wIndex, values, self.HS_DELAY)

  def TriggerConfigR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = unpack('BBHH',self.udev.controlRead(request_type, self.TRIGGER_CONFIG, wValue, wIndex, 6, self.HS_DELAY))
    return value

  def CalConfig(self, voltage=0):
    """
    This command configures the calibration voltage value.  The
    default will be 0V at power up.
      voltage:  Calibration voltage
                0:  0.0V
                1: +1.0V
                2: -1.0V
                3: +2.0V
                4: -2.0V
                5: +5.0V
                6: -5.0V
                7: +10.0V
                8: -10.0V
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.CAL_CONFIG
    wValue = 0x0
    wIndex = 0x0
    self.udev.controlWrite(request_type, request, wValue, wIndex, [voltage], self.HS_DELAY)

  def CalConfigR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = unpack('B',self.udev.controlRead(request_type, self.TRIGGER_CONFIG, wValue, wIndex, 6, self.HS_DELAY))
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
    print('**** USB-2020 Status ****')
    if status & self.AIN_SCAN_RUNNING:
      print('  Analog Input scan running.')
    if status & self.AIN_SCAN_OVERRUN:
      print('  Analog Input scan overrun.')
    if status & self.AIN_SCAN_DONE:
      print(' Analog Input scan done.')
    if status & self.FPGA_CONFIGURED:
      print('  FPGA is configured.')
    if status & self.FPGA_CONFIG_MODE:
      print('  FPGA in configuration mode.')

  def volts(self, gain, value):
    if gain == self.BP_10V:
      volt = (value - 2048) * 10./2048.
    elif gain == self.BP_5V:
      volt = (value - 2048) * 5./2048.
    elif gain == self.BP_2V:
      volt = (value - 2048) * 2./2048.
    elif gain == self.BP_1V:
      volt = (value - 2048) * 1./2048.
    else:
      raise ValueError('volts: unknown gain')
      return
    return volt
