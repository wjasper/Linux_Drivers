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

class usb_dio32HS(mccUSB):

  USB_DIO32HS_PID = 0x0133
    
  PORTA = 0x0
  PORTB = 0x1
  DIO_PORTS = 0x2  # Ports A & B
  PORT0 = 0x1      # Port A for channel_map
  PORT1 = 0x2      # Port B for channel_map

  DIR_IN = 0x1
  DIR_OUT= 0x0

  # Status bit values
  IN_SCAN_RUNNING   = 0x2    # Input pacer running
  IN_SCAN_OVERRUN   = 0x4    # Input scan overrun
  OUT_SCAN_RUNNING  = 0x8    # Output scan running
  OUT_SCAN_UNDERRUN = 0x10   # Output scan underrun
  IN_SCAN_DONE      = 0x20   # Input scan done
  OUT_SCAN_DONE     = 0x40   # Output scan done
  FPGA_CONFIGURED   = 0x100  # FPGA is configured
  FPGA_CONFIG_MODE  = 0x200  # FPGA config mode
  
  # Scan Modes
  CONTINUOUS_READOUT   = 0x1  # Continuous mode
  SINGLEIO             = 0x2  # Return data after every read (used for low frequency scans)
  FORCE_PACKET_SIZE    = 0x4  # Force packet_size

  BASE_CLOCK         = 96.E6  # Base clock frequency
  MAX_PACKET_SIZE_HS = 512    # max packet size for HS device
  MAX_PACKET_SIZE_FS = 64     # max packet size for HS device

  # Commands and Codes for USB-DIO32HS
  # Digital I/O Commands
  DTRISTATE            = 0x00  # Read/write digital port tristate registers
  DPORT                = 0x01  # Read digital port pins
  DLATCH               = 0x02  # Read/write digital port output latch register

  # Register Commands
  READ_REG             = 0x10  # Read the specified register
  WRITE_REG            = 0x11  # Write the specified register

  # Acquisition Commands
  IN_SCAN_START        = 0x20  # Start input scan
  IN_SCAN_STOP         = 0x21  # Stop input scan
  IN_SCAN_CLEAR_FIFO   = 0x22  # Clear data in the input FIFO
  IN_BULK_FLUSH        = 0x23  # Flush the input Bulk pipe
  OUT_SCAN_START       = 0x24  # Start output scan
  OUT_SCAN_STOP        = 0x25  # Stop output scan
  OUT_SCAN_CLEAR_FIFO  = 0x26  # Clear data in the ouptut FIFO

  # Memory Commands
  MEMORY               = 0x30  # Read/Write EEPROM
  MEM_ADDRESS          = 0x31  # EEPROM read/write address value
  MEM_WRITE_ENABLE     = 0x32  # Enable writes to firmware area

  # Miscellaneous Commands
  STATUS                = 0x40  # Read device status
  BLINK_LED             = 0x41  # Causes the LED to blink
  RESET                 = 0x42  # Reset the device
  TRIGGER_CONFIG        = 0x43  # External trigger configuration
  PATTERN_DETECT_CONFIG = 0x44  # Pattern Detection trigger configuration
  SERIAL                = 0x48  # Read/Write USB Serial Number

  # FPGA Configuration Commands
  FPGA_CONFIG          = 0x50 # Start FPGA configuration
  FPGA_DATA            = 0x51 # Write FPGA configuration data
  FPGA_VERSION         = 0x52 # Read FPGA version

  HS_DELAY = 2000

  def __init__(self, serial=None):

    self.status = 0                         # status of the device
    self.productID = self.USB_DIO32HS_PID   # USB-DIO32HS
    self.udev = self.openByVendorIDAndProductID(0x9db, self.productID, serial)
    if not self.udev:
      raise IOError("MCC USB-DIO32HS not found")
      return

    # Configure the FPGA
    if not (self.Status() & self.FPGA_CONFIGURED) :
      # load the FPGA data into memory
      from usb_dio32HS_rbf import FPGA_data
      print("Configuring FPGA.  This may take a while ...")
      self.FPGAConfig()
      if self.Status() & self.FPGA_CONFIG_MODE:
        for i in range(0, len(FPGA_data) - len(FPGA_data)%64, 64) :
          self.FPGAData(FPGA_data[i:i+64])
        i += 64
        if len(FPGA_data) % 64 :
          self.FPGAData(FPGA_data[i:i+len(FPGA_data)%64])
        if not (self.Status() & self.FPGA_CONFIGURED):
          print("Error: FPGA for the USB-DIO32HS is not configured.  status = ", hex(self.Status()))
          return
      else:
        print("Error: could not put USB-DIO32HS into FPGA Config Mode.  status = ", hex(self.Status()))
        return
    else:
      print("USB-DIO32HS FPGA configured.")


    if sys.platform.startswith('linux'):
      if self.udev.kernelDriverActive(0):
        self.udev.detachKernelDriver(0)
        self.udev.resetDevice()

    # claim all the needed interfaces for InScan
    self.udev.claimInterface(0)

    # Find the maxPacketSize for bulk transfers
    self.wMaxPacketSize = self.getMaxPacketSize(libusb1.LIBUSB_ENDPOINT_IN | 0x6)  #EP IN 6

  ##############################################
  #           Digital I/O  Commands            #
  ##############################################
  # Read/Write digital port tristate register

  def DTristateR(self, port=0):
    """
    This command reads the digital port tristate registers.  The
    tristate register determines if the latch register value is driven
    onto the port pin.  A '1' in the tristate register makes the
    corresponding pin an input, a '0' makes it an output.
    """
    if port < 0 or port > 1:
      raise ValueError('DTristateR: error in port number.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = port      # the port number to select (0-1)
    value ,= unpack('H',self.udev.controlRead(request_type, self.DTRISTATE, wValue, wIndex, 2, self.HS_DELAY))
    return value

  def DTristateW(self, port, value):
    """
    This command writes the digital port tristate register.  The
    tristate register determines if the latch register value is driven
    onto the port pin.  A '1' in the tristate register makes the
    corresponding pin an input, a '0' makes it an output.
    """

    if port < 0 or port > 1:
      raise ValueError('DTristateW: error in port number.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.DTRISTATE
    wValue = value & 0xffff
    wIndex = port
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  def DPort(self, port):
    """
    This command reads the current state of the digital pins from the specified port.
      port  = 0  Read port 0
      port  = 1  Read port 1
      port  = 2  Read both ports
    """

    if port < 0 or port > 2:
      raise ValueError('DPort: error in port number.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value = unpack('HH',self.udev.controlRead(request_type, self.DPORT, wValue, wIndex, 4, self.HS_DELAY))
    if port == 0:
      return value[0]
    elif port == 1:
      return value[1]
    else:
      return list(value)

  def DLatchR(self, port):
    """
    This command reads the digital port latch register
      port  = 0  Read port 0
      port  = 1  Read port 1
      port  = 2  Read both ports
    """
    if port < 0 or port > 2:
      raise ValueError('DLatchR: error in port number.')
      return

    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = port
    value = unpack('HH',self.udev.controlRead(request_type, self.DLATCH, wValue, wIndex, 4, self.HS_DELAY))

    if port == 0:
      return value[0]
    elif port == 1:
      return value[1]
    else:
      return list(value)

  def DLatchW(self, port, value):
    """
    This command writes the digital port latch register
      port  = 0  Write port 0
      port  = 1  Write port 1
      port  = 2  Write both ports
    """

    if port < 0 or port > 2:
      raise ValueError('DLatchW: error in port number.')
      return

    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.DLATCH
    wValue = 0x0
    wIndex = port
    value = pack('HH', value[0], value[1])
    self.udev.controlWrite(request_type, request, wValue, wIndex, value, self.HS_DELAY)

  ##########################################
  #           Register Commands            #
  ##########################################

  def ReadReg(self, address):
    """
    This command reads the FPGA register at the specified address
    """
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = address & 0xff
    data = self.udev.controlRead(request_type, self.READ_REG, wValue, wIndex, 1, self.HS_DELAY)    
    return data

  def WriteReg(self, address, value):
    """
    This command writes the FPGA register at the specified address.
    The user can change the tristate settings with this command, so
    any time it is sent, the software must re-check the DTristate
    status to know the current state.
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.WRITE_REG
    wValue = value & 0xff
    wIndex = address & 0xff
    self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], self.HS_DELAY)

  ##########################################
  #        Acquisition  Commands           #
  ##########################################
  def InScanStart(self, channel_map, count, retrig_count, frequency, options, mode=0):
    """
    This command starts the input channel scan.  This command will
    result in a bus stall if an input scan is currently running.

    Notes:

    The pacer rate is set by an internal 32-bit incrementing timer
    running at a base rate of 96MHz.  The timer is controlled by
    pacer_period.  A pulse will be output at the INPUT_PACER_OUT pin
    at every pacer_period interval regardless of mode.

    If pacer_period is set to 0, the device does not generate a clock.
    It uses the INPUT_PACER_IN pin as the pacer source.

    The timer will be reset and sample acquired when its value equals
    timer_period.  The equation for calculating timer_period is:

        timer_period = [96MHz / (sample frequency)] - 1

    The data will be returned in packets utilizing a bulk
    endpoint. The data will be in the format:

    lowchannel sample 0 : lowchannel + 1 sample 0: ... :hichannel sample 0
    lowchannel sample 1 : lowchannel + 1 sample 1: ... :hichannel sample 1
    ...
    lowchannel sample n : lowchannel + 1 sample n: ... :hichannel sample n
 
    The scan will not begin until the InScanStart command is
    sent (and any trigger conditions are met.)  Data will be sent
    until reaching the specified count or a InScanStop command is
    sent.

    The packet_size parameter is used for low sampling rates to avoid
    delays in receiving the sampled data.  The buffer will be sent,
    rather than waiting for the buffer to fill.  This mode should not
    be used for high sample rates in order to avoid data loss.

    Pattern detection is used with the PatternDetectConfig command to
    set up a specified number of bits to watch, and then trigger when
    those bits reach the specified value.

    The retrigger mode option and retrig_count parameter are only used
    if trigger is used.  This option will cause the trigger to be
    rearmed after retrig_count samples are acquired, with a total of
    count samples being returned for the entire scan.

    channel_map:  bit field marking which channels are in the scan
                    bit 0:    1 = Port 0
                    bit 1:    1 = Port 1
                    bits 2-7: Reserved
    count:        the total number of scans to perform (0 for continuous scan)
    retrig_count: the number of scans to perform for each trigger in retrigger mode
    pacer_period: pacer timer period value (0 for external clock)
    packet_size:  number of samples to transfer at a time
    options:      bit field that controls various options.
                    bit 0:    1 = use external trigger
                    bit 1:    1 = user Pattern Detection trigger
                    bit 2:    1 = retrigger mode, 0 = normal trigger
                    bits 3-7: Reserved
    mode:  mode bits:
                    bit 0:  0 = counting mode,  1 = CONTINUOUS_READOUT
                    bit 1:  1 = SINGLEIO
                    bit 2:  1 = use packet size passed usbDevice1808->packet_size
                    bit 3:  1 = convert to voltages  
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    
    if frequency > 8.E6:  # 8MHz throughput
      frequency = 8.E6

    if frequency == 0.0:
      pacer_period = 0     # use external clock
    else:
      pacer_period = round((self.BASE_CLOCK / frequency) - 1)

    channel_map &= 0x3
    if channel_map == 0x0:
      raise ValueError('InScanRead: error in channel_map.')
      return
    if channel_map == 0x1 or channel_map == 0x2:
      nchan = 1
    else:
      nchan = 2
    bytesPerScan = nchan*2  # 2 ports of 16 bits each

    self.in_nchan = nchan                   # number of input channels
    self.in_frequency = frequency           # input frequency
    self.in_channel_map = channel_map       # input channel map
    self.mode = mode & 0xff                 # input scan mode

    if count == 0:
      self.mode |= self.CONTINUOUS_READOUT
      self.bytesToRead = -1                  # disable and sample forever
    else:
      self.bytesToRead = count*bytesPerScan  # total number of bytes to read      

    if self.mode & self.FORCE_PACKET_SIZE:
      packet_size = self.packet_size
    elif self.mode & self.SINGLEIO:
      packet_size = nchan
    elif self.mode & self.CONTINUOUS_READOUT:
      packet_size = int((( (self.wMaxPacketSize//bytesPerScan) * bytesPerScan) // 2))
    else:
      packet_size = self.wMaxPacketSize // 2
    self.packet_size = packet_size

    if self.mode & self.CONTINUOUS_READOUT:
      self.in_count = 0
    else:
      self.in_count = count

    self.in_retrig_count = retrig_count
    self.in_options = options

    packet_size -= 1  # force to uint8_t size in range 0-255
    scanPacket = bytearray(15)
    scanPacket[0] = channel_map
    pack_into('III',scanPacket, 1, count, retrig_count, pacer_period)
    scanPacket[13] = packet_size
    scanPacket[14] = options
    result = self.udev.controlWrite(request_type, self.IN_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

    self.status = self.Status()

  def InScanRead(self):
    if self.mode & self.CONTINUOUS_READOUT or self.mode & self.SINGLEIO :
      nSamples = self.packet_size
    else:
      nSamples = self.in_count*self.in_nchan

    try:
      data =  list(unpack('H'*nSamples, self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, int(2*nSamples), self.HS_DELAY)))
    except:
      print('InScanRead: error in bulkRead.')

    if len(data) != nSamples:
      raise ValueError('InScanRead: error in number of samples transferred.')
      return len(data)

    if self.bytesToRead > len(data)*2:
      self.bytesToRead -= len(data)*2
    elif self.bytesToRead > 0 and self.bytesToRead < len(data)*2:  # all done
      self.InScanStop()
      self.InScanClearFIFO()
      self.status = self.Status()
      return data

    if self.mode & self.CONTINUOUS_READOUT:
      return data

    # if nbytes is a multiple of wMaxPacketSize the device will send a zero byte packet.
    if nSamples*2%self.wMaxPacketSize == 0:
     dummy = self.udev.bulkRead(libusb1.LIBUSB_ENDPOINT_IN | 6, 2, 100)
     
    self.status = self.Status()
    if self.status & self.IN_SCAN_OVERRUN:
      self.InScanStop()
      raise ValueError('InScanRead: Scan overrun.')
      return

    return data

  
  def InScanStop(self):
    """
    This command stops the input scan (if running).
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.IN_SCAN_STOP
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def InScanClearFIFO(self):
    """
    This command clears the input firmware buffer
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.IN_SCAN_CLEAR_FIFO
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def InBulkFlush(self, count=5):
    """
    This command flushes the input Bulk ipie a number of times
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.IN_BULK_FLUSH
    wValue = count
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def OutScanStart(self, channel_map, count, retrig_count, frequency, options):
    """
    This command starts the output channel scan.  This command will
    result in a bus stall if an input scan is currently running.

    Notes:
    
    The output scan operates with the host continuously transferring
    data from the outputs until the end of the scan.  If the count
    parameter is 0, the scan will run until the OutScanStop command is
    issued by the host; if it is nonzero, the scan will stop
    automatically after the specified number of scans have been output.
    The channels in the scan are selected in the options bit field.
    Scans refers to the number of updates to the channels (if both
    channels are used, one scan is an update to both channels.)

    The time base is controlled by an internal 32-bit timer running at
    a base rate of 96MHz.  The timer is controlled by pacer_period.
    The equation for calculating pacer_period is:
  
      pacer_period = [96MHz / (sample frequency)] - 1

    The same time base is used for all channels when the scan involved
    multiple channels.  The output data is to be sent using bulk out
    endpoints.  The data must be in the format:

    low channel sample 0: [high channel sample 0]
    low channel sample 1: [high channel sample 1]
    ...
    low channel sample 1: [high channel sample n]

    The output data is written to an internal FIFO.  The bulk endpoint
    data is only accepted if there is room in the FIFO.  Output data
    may be sent to the FIFO before the start of the scan, and the FIFO
    is cleared when the OutScanClearFIFO command is received.  The
    scan will not begin until the OutScanStart command is sent (and
    output data is in the FIFO).  Data will be output until reaching
    the specified number of scans (in single execution mode) or an
    OutScanStop command is sent.


    channel_map:  bit field marking which channels are in the scan
                   bit 0: 1 = Port 0
                   bit 1: 1 = Port 1
                   bits 2-7: Reserved
    count:        the total number of scans to perform (0 for continuous scan)
    retrig_count: the number of scans to perform for each trigger in retrigger mode
    pacer_period: pacer timer period value (0 for external clock)
    options:      bit field that controls various options
                    bit 0: 1 = use external trigger
                    bit 1: 1 = use Pattern Detection trigger
                    bit 2  1 = use retrigger mode, 0 = normal trigger
                    bits 3-7: Reserved
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    if frequency == 0:
      pacer_period = 0  # use ICLKO
    else:
      pacer_period = round((self.BASE_CLOCK / frequency) - 1)

    channel_map &= 0x3
    if channel_map == 0x0:
      raise ValueError('OutScanWrite: error in channel_map.')
      return
    if channel_map == 0x1 or channel_map == 0x2:
      nchan = 1
    else:
      nchan = 2
    bytesPerScan = nchan*2  # 2 ports of 16 bits each

    if count == 0:
      self.bytesToWrite = -1
    else:
      self.bytesToWrite = count*bytesPerScan

    self.out_nchan = nchan                   # number of output channels
    self.out_frequency = frequency
    
    scanPacket = bytearray(14)
    scanPacket[0] = channel_map
    pack_into('III',scanPacket, 1, count, retrig_count, pacer_period)
    scanPacket[13] = options
    result = self.udev.controlWrite(request_type, self.OUT_SCAN_START, 0x0, 0x0, scanPacket, timeout = 200)

    self.status = self.Status()

  def OutScanWrite(self, data):
    # data is a list of unsigned 16 bit numbers
    value = [0]*len(data)*2
    timeout = int(500 + 1000*len(data)/self.out_frequency)
    for i in range(len(data)):
      value[2*i] = data[i] & 0xff
      value[2*i+1] = (data[i] >> 8) & 0xff
    try:
      result = self.udev.bulkWrite(2, value, timeout)
    except:
      print('OutScanWrite: error in bulkWrite')
      return

  def OutScanStop(self):
    """
    This command stops the output scan (if running).
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.OUT_SCAN_STOP
    wValue = 0
    wIndex = 0
    result = self.udev.controlWrite(request_type, request, wValue, wIndex, [0x0], timeout = 100)

  def OutScanClearFIFO(self):
    """
    This command clears the output firmware buffer
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.OUT_SCAN_CLEAR_FIFO
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
    0x7000-0x7FFF               User data

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

  def TriggerConfigR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    wValue = 0
    wIndex = 0
    value ,= unpack('B',self.udev.controlRead(request_type, self.TRIGGER_CONFIG, wValue, wIndex, 1, self.HS_DELAY))
    return value

  def PatternDetectConfig(self, pattern, mask, options):
    """
    This function configures the Pattern Detection trigger.  Once the
    trigger is received, the scan will proceed as configued.  The "use
    Pattern Detection trigger" option must be used in the InScanStart
    command to utilize this feature.

    pattern: The pattern on which to trigger
    mask:    These bits will mask the inputs such that only bits set to 1 here will be compared to the pattern
    options: Bit field that controls various options
             bit 0:    Trigger Port (0 = Port 0,  1 = Port 1)
             bits 1-2: 00 = Equal to Pattern
                       01 = Not equal to Pattern
                       10 = Greater than Pattern's numeric value
                       11 = Less than Pattern's numeric value
             bits 3-7: Reserved 
    """
    request_type = (HOST_TO_DEVICE | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.PATTERN_DETECT_CONFIG
    wValue = 0
    wIndex = 0
    value = pack('HHB', pattern, mask, options)
    self.udev.controlWrite(request_type, request, wValue, wIndex, value, self.HS_DELAY)

  def PatternDetectConfigR(self):
    request_type = (DEVICE_TO_HOST | VENDOR_TYPE | DEVICE_RECIPIENT)
    request = self.PATTERN_DETECT_CONFIG
    wValue = 0
    wIndex = 0
    value = unpack('HHB', self.udev.controlRead(request_type, request, wValue, wIndex, 5, self.HS_DELAY))
    return value

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
    print('**** USB-DIO32HS Status ****')
    if status & self.IN_SCAN_RUNNING:
      print('  Input scan running.')
    if status & self.IN_SCAN_OVERRUN:
      print('  Input scan overrun.')
    if status & self.OUT_SCAN_RUNNING:
      print('  Output scan running.')
    if status & self.OUT_SCAN_UNDERRUN:
      print('  Output scan underrun.')
    if status & self.IN_SCAN_DONE:
      print('  Input scan done.')
    if status & self.OUT_SCAN_DONE:
      print('  Output scan done.')
    if status & self.FPGA_CONFIGURED:
      print('  FPGA is configured.')
    if status & self.FPGA_CONFIG_MODE:
      print('  FPGA in configuration mode.')

