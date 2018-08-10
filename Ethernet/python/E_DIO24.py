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

from datetime import datetime
from struct import *
from mccPy import *

EDIO24_PID = 0x0137  # Product code for the MCC E-DIO24

# Digital I/O Commands
CMD_DIN_R          = 0x00  # Read DIO pins
CMD_DOUT_R         = 0x02  # Read DIO latch value
CMD_DOUT_W         = 0x03  # Write DIO latch value
CMD_DCONF_R        = 0x04  # Read DIO configuration value
CMD_DCONF_W        = 0x05  # Write DIO Configuration value

# Counter Commands
CMD_COUNTER_R      = 0x30  # Read event counter
CMD_COUNTER_W      = 0x31  # Reset event counter

# Memory Commands
CMD_CONF_MEM_R     = 0x40  # Read configuration memeory
CMD_CONF_MEM_W     = 0x41  # Write configuration memory
CMD_USR_MEM_R      = 0x42  # Read user memory  
CMD_USR_MEM_W      = 0x43  # Write user memory
CMD_SET_MEM_R      = 0x44  # Read settings memory
CMD_SET_MEM_W      = 0x45  # Write settings memory
CMD_BOOT_MEM_R     = 0x46  # Read bootloader memory
CMD_BOOT_MEM_W     = 0x47  # Write bootloader memory

# Miscellaneous Commands
CMD_BLINKLED       = 0x50  # Blink the LED
CMD_RESET          = 0x51  # Reset the device
CMD_STATUS         = 0x52  # Read the device status
CMD_NETWORK_CONF   = 0x54  # Read device network configuration
CMD_FIRMWARE       = 0x60  # Enter bootloader for firmware upgrade

class E_DIO24:
    
  def __init__(self, device):
    self.device = device        # inherit values from mccEthernetDevice


  #################################
  #     Digital I/O Commands      #
  #################################

  def DIn(self):
    # This command reads the current state of the DIO pins.  A 0 in a
    # bit position indicates the correspoing pin is reading a low
    # state, and a 1 indicates a high state.
    dataCount = 0
    replyCount = 3
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_DIN_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise
      print('DIn: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8) | (r_buffer[MSG_INDEX_DATA+2]<<16)
    if (result == False):
      print('Error in DIn E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result, value
    
  def DOut_R(self):
    # This command reads the DIO output latch value.  The factory power
    # on default is all zeros. A 0 in a bit position indicates the
    # corresponding pin driver is low, a 1 indicates it is high.
    
    dataCount = 0
    replyCount = 3
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_DOUT_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('DOut_R: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = r_buffer[MSG_INDEX_DATA] | r_buffer[MSG_INDEX_DATA+1]<<8 | r_buffer[MSG_INDEX_DATA+2]<<16
    if (result == False):
      print('Error in DOut_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result, value

  def DOut(self, mask, value):
    # This command writes the DIO latch value.  The factory power on
    # default is all ones (pins are floating). Writing a 0 to a bit will set
    # the corresponding pin driver low, writing a 1 sets it high.
    # Individual bits may be written using the port bitmask.
    
    dataCount = 6
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_DOUT_W
    s_buffer[MSG_INDEX_DATA]           = mask & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (mask>>8)  & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = (mask>>16) & 0xff
    s_buffer[MSG_INDEX_DATA+3]          = value & 0xff
    s_buffer[MSG_INDEX_DATA+4]         = (value>>8)  & 0xff
    s_buffer[MSG_INDEX_DATA+5]         = (value>>16) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('DOut: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in DOut E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  def DConfig_R(self):
    # This command reads the DIO configuration value.  A 1 in a bit
    # position indicates the corresponding pin is set to an input, a 0
    # indicates it is set to an output.  The power on devault is all 1 (input)
    
    dataCount = 0
    replyCount = 3
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_DCONF_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('DConfig_R: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int(r_buffer[MSG_INDEX_DATA] | r_buffer[MSG_INDEX_DATA+1]<<8 | r_buffer[MSG_INDEX_DATA+2]<<16)
    if (result == False):
      print('Error in DConfig_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result, value

  def DConfig_W(self, mask, value):
    # This command writes the configuration value.  A 1 in a bit
    # position sets the corresponding pin to an input, a 0 sets it to an 
    # output.  The power on default is all ones (input)
    # Individual configurations may be written using the port bitmask.
    
    dataCount = 6
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_DCONF_W
    s_buffer[MSG_INDEX_DATA]           = mask & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (mask>>8)  & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = (mask>>16) & 0xff
    s_buffer[MSG_INDEX_DATA+3]          = value & 0xff
    s_buffer[MSG_INDEX_DATA+4]         = (value>>8)  & 0xff
    s_buffer[MSG_INDEX_DATA+5]         = (value>>16) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('DConfig_W: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in DConfig_W E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  #################################
  #       Counter Commands        #
  #################################

  def counter(self):
  # This command reads the event counter on pin P2D7. Configure it as
  #   input.  The pin accepts frequency input up to 10 MHz.  The
  #   internal counter increments when the TTL levels transition from
  #   low to high.

    dataCount = 0
    replyCount = 4
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_COUNTER_R
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise
      print('counter: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value ,= unpack_from('I', r_buffer, MSG_INDEX_DATA)
    if (result == False):
      print('Error in counter E-1608.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result, value
  
  def resetCounter(self):
    # This command resets the event counter.  On a write, the
    # counter will be reset to 0.
    
    dataCount = 0
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_COUNTER_W
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise
      print('restCounter: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in restCounter_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result
  
  #################################
  #     Miscellaneous Commands    #
  #################################
  def blink(self, count=1):
    # This command will blink the device power LED "count" times

    dataCount = 1
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = CMD_BLINKLED
    s_buffer[MSG_INDEX_DATA]           = count
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('blink: timout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
        r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                                and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in blink E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  def reset(self):
    # The command resets the device.
    dataCount = 0
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = CMD_RESET
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('reset: timout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in reset E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  def status(self):
    # This command reads the device status
    # bits 0-15   Reserved

    dataCount = 0
    replyCount = 2
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = CMD_STATUS
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(16)
    except socket.timeout:
      raise
      print('status: timout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           self.status = r_buffer[MSG_INDEX_DATA] | (r_buffer[MSG_INDEX_DATA+1]<<8 & 0xff)
    if (result == False):
      print('Error in status E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result, self.status

  def networkConfig(self):
    # This command reads the current network configuration.  Returns tuple
    #  (ip_address, subnet_mask, gateway_address)
    #  
    dataCount = 0
    replyCount = 12
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = CMD_NETWORK_CONF
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise
      print('networkConfig: timout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           data = unpack_from('III', r_buffer, MSG_INDEX_DATA)
           value = (socket.inet_ntoa(pack('L', data[0])), socket.inet_ntoa(pack('L', data[1])), socket.inet_ntoa(pack('L', data[2])))
    if (result == False):
      print('Error in networkConfig E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result, value

  def firmwareUpgrade(self):
    # This command causes the device to reset and enter the bootloader
    # for a firmware upgrade.  It erases a portion of the program memory so
    # the device must have firmware downloaded through the bootloder before
    # it can be used again.
    
    dataCount = 2
    replyCount = 0
    result = False
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer
    
    s_buffer[MSG_INDEX_COMMAND]        = CMD_FIRMWARE
    s_buffer[MSG_INDEX_DATA]           = 0xad                     # key
    s_buffer[MSG_INDEX_DATA+1]         = 0xad                     # key
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(64)
    except socket.timeout:
      raise
      print('firmwareUpgrade: timout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in firmwareUpgrade E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  #################################
  #       Memory  Commands        #
  #################################

  def configMemory_R(self, address, count):
    # This command reads the nonvolatile configuration memory.  The configuration memory is
    # 16 bytes (address 0 - 0xff)

    if (count > 16):
      return False

    dataCount = 4
    replyCount = count
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_CONFIG_MEM_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise
      print('configMemory_R: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder = 'little')
    if (result == False):
      print('Error in configMemory_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return value

  def configMemory_W(self, address, count, data):
     # This command writes the nonvolatile configuration memory.  The
     # config memory is 16 bytes (address 0 - 0xf) The config memory
     # should only be written during factory setup and has an additional
     # lock mechanism to prevent inadvertent writes.  To enable writes
     # to the config memory, first write the unlock code 0xAA55 to
     # address 0x10.  Writes to the entire meemory range are then
     # possible.  Write any other value to address 0x10 to lock the
     # memory after writing.  The amount of data to be writeen is
     #inferred from the frame count - 2.
     #
     # address: the start address for writing (0-0xf)
     # data:    the data to be written (frame count -2)

    if (count > 16):
      return False

    dataCount = count + 2
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_CONF_MEM_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise
      print('configMemory_W: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in configMemory_W E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  def UserMemory_R(self, address, count):
    # This command reads the nonvolatile user memory.  The user memory is
    # 3827 bytes (address 0 - 0xeef)
    #
    # address: the start address for reading (0-0xeef)
    # count:   the number of bytes to read (max 1024 due to protocol)

    if (count > 1024 or address > 0xeef):
      return False

    dataCount = 4
    replyCount = count
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_USR_MEM_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1050)
    except socket.timeout:
      raise
      print('UserMemory_R: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder='little')
    if (result == False):
      print('Error in UserMemory_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return value

  def UserMemory_W(self, address, count, data):
    # This command writes the nonvolatile user memory.  The user memory
    # is 3824 bytes (address 0 - 0xeef). The amount of data to be
    # written is inferred from the frame count - 2.  The maximum that
    # can be writtenin one transfer is 1024 bytes.

    if (count > 512 or address > 0xeef):
      return False

    dataCount = count + 2
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_USR_MEM_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise
      print('UserMemory_W: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in UserMemory_W E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  def SettingsMemory_R(self, address, count):
    # This command reads the nonvolatile settings memory.  The settings memory is
    # 256 bytes (address 0 - 0xff)
    #
    # address: the start address for reading (0-0xff)
    # count:   the number of bytes to read (max 256 due to protocol)

    if (count > 256 or address > 0xff):
      return False

    dataCount = 4
    replyCount = count
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_SET_MEM_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise
      print('SettingsMemory_R: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder='little')
    if (result == False):
      print('Error in SettingsMemory_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return value

  def SettingsMemory_W(self, address, count, data):
    # This command writes to the nonvolatile settings memory.  The settings memory
    # is 256 bytes (address 0 - 0xff).  The amount of data to be
    # written is inferred from the frame count  - 2.  The maximum that
    # can be written in one transfer is 512 bytes.  The settings will
    # be implemented after a device reset.

    if (count > 256 or address > 0xff):
      return False

    dataCount = count + 2
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_SET_MEM_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise
      print('SettingsMemory_W: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in SettingsMemory_W E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

  def BootloaderMemory_R(self, address, count):
    # This command reads the bootloader stored in nonvolatile FLASH
    # memory.  The bootloader is located in program FLASH memory in two
    # physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
    # code and 0x1FC00000 - 0x1FC01FFF for C startup code and
    # interrupts.  Reads may be performed at any time.
    #
    # address: the start address for reading (see above)
    # count:   the number of bytes to read (max 1024)

    if (count > 1024):
      return False

    dataCount = 4
    replyCount = count
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_BOOT_MEM_R
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    s_buffer[MSG_INDEX_DATA+2]         = count & 0xff
    s_buffer[MSG_INDEX_DATA+3]         = (count>>8) & 0xff
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1050)
    except socket.timeout:
      raise
      print('BootloaderMemory_R: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
           value = int.from_bytes(r_buffer[MSG_INDEX_DATA:MSG_INDEX_DATA+replyCount], byteorder='little')
    if (result == False):
      print('Error in BootloaderMemory_R E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return value

  def BootloaderMemory_W(self, address, count, data):
    # This command writes the bootloader stored in nonvolatile FLASH
    # memory.  The bootloader is located in program FLASH memory in two
    # physical address ranges: 0x1D000000 - 0x1D007FFF for bootloader
    # code and 0x1FC00000 - 0x1FC01FFF for C startup code and
    # interrupts.  Writes outside these ranges are ignored.  The
    # bootloader memory is write protected and must be unlocked in
    # order to write the memory.  The unlock proceedure is to write the
    # unlock code 0xAA55 to address 0xFFFFFFFE.  Writes to the entire
    # memory range are then possible.  Write any other value to address
    # 0xFFFFFFFE to lock the memory after writing.
    #
    # The FLASH memory must be erased prior to programming.  A bulk
    # erase is perfomred by writing 0xAA55 to address 0x80000000 after
    # unlocking the memory for write.  The bulk erase will require
    # approximately 150ms to complete.  Once the erase is complete, the
    # memory may be written; however, the device will not be able to
    # boot unless it has a valid bootloader so the device shold not be
    # reset until the bootloader is completely written and verified
    # using readBootloaderMemory_E1608().
    #
    # The writes are perfomred on 4-byte boundaries internally and it
    # is recommended that the output data be sent in the same manner.
    # The amount of data to be written is inferred frolm the frame
    # count - 2. The maximum count value is 1024.

    if (count > 1024):
      return False

    dataCount = count + 2
    replyCount = 0
    s_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+dataCount)  # send buffer
    r_buffer = bytearray(MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+replyCount) # reply buffer

    s_buffer[MSG_INDEX_COMMAND]        = CMD_BOOT_MEM_W
    s_buffer[MSG_INDEX_DATA]           = address & 0xff
    s_buffer[MSG_INDEX_DATA+1]         = (address>>8) & 0xff
    for i in range(count):
      s_buffer[MSG_INDEX_DATA+2+i] = data[i]
    s_buffer[MSG_INDEX_START]          = MSG_START
    s_buffer[MSG_INDEX_FRAME]          = self.device.frameID
    self.device.frameID += 1                                      # increment frame ID with every send
    s_buffer[MSG_INDEX_STATUS]         = 0
    s_buffer[MSG_INDEX_COUNT_LOW]      = (dataCount & 0xff)
    s_buffer[MSG_INDEX_COUNT_HIGH]     = ((dataCount>>8) & 0xff)
    s_buffer[MSG_INDEX_DATA+dataCount] = 0xff - self.device.calcChecksum(s_buffer, MSG_INDEX_DATA+dataCount)
    self.device.sock.settimeout(.1)
    self.device.sock.send(s_buffer)

    try:
      r_buffer = self.device.sock.recv(1024)
    except socket.timeout:
      raise
      print('BootloaderMemory_W: timeout error.\n')
      return
    if len(r_buffer) == MSG_HEADER_SIZE + MSG_CHECKSUM_SIZE + replyCount:
      if r_buffer[MSG_INDEX_START] == s_buffer[0]                               and \
         r_buffer[MSG_INDEX_COMMAND] == s_buffer[MSG_INDEX_COMMAND] | MSG_REPLY and \
         r_buffer[MSG_INDEX_FRAME] == s_buffer[2]                               and \
         r_buffer[MSG_INDEX_STATUS] == MSG_SUCCESS                              and \
         r_buffer[MSG_INDEX_COUNT_LOW] == replyCount & 0xff                     and \
         r_buffer[MSG_INDEX_COUNT_HIGH] == (replyCount >> 8) & 0xff             and \
         r_buffer[MSG_INDEX_DATA+replyCount] + self.device.calcChecksum(r_buffer,(MSG_HEADER_SIZE+replyCount)) == 0xff :
           result = True
    if (result == False):
      print('Error in BootloaderMemory_W E-DIO24.  Status =', r_buffer[MSG_INDEX_STATUS])
    return result

