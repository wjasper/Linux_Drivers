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

from E_DIO24 import *
import time
import sys


def main():
  if len(sys.argv) == 2:
    device = []
    device.append(mccEthernetDevice(EDIO24_PID,sys.argv[1]))  
  else:
    # Find a E-DIO24 on the subnet
    device = mccDiscover(EDIO24_PID)

  print('Number of devices found = ', len(device))
  if (len(device) > 0):
    for i in range(len(device)):
      device[i].printDeviceInfo()
  else:
    exit(0)

  # Open the device
  device[0].mccOpenDevice()

  # initalize the class
  edio24 = E_DIO24(device[0])

  while True:
    print("\nE-DIO24 Testing")
    print("----------------")
    print("Hit 'b' to blink.")
    print("Hit 'c' to test counter.")
    print("Hit 'd' to test digital IO.")
    print("Hit 'C' to configure the DIO for input or output")
    print("Hit 'r' to reset the device")
    print("Hit 'n' to get networking information")
    print("Hit 'R' to read System Memory Map")
    print("Hit 'W' to write System Memory Map")
    print("Hit 's' to get Status")
    print("Hit 'e' to exit")

    ch = input('\n')
    if ch == 'b':
      count = input('Enter number of times to blink: ')
      edio24.Blink(int(count))
    elif ch == 'c':
      print('Testing Counter on E-DIO24')
      print('connect P2D6 to P2D7')
      mask = 0xc00000
      edio24.DConfig_W(mask, 0x800000) # set pins P2D6 output, counter pin P2D7 input
      edio24.ResetCounter()            # reset the counter
      for i in range(20):
        edio24.DOut(mask, 0x400000)    # set P2D6 HI
        time.sleep(.05)
        edio24.DOut(mask, 0x000000)    # set P2D6 LOW
        time.sleep(.05)
        count = edio24.Counter()
        print('Counter = ', count)
    elif ch == 'C':
      mask = int(input('Enter mask (which bits to configure): '),16)
      value = int(input('Enter the configuration value: '),16)
      if (edio24.DConfig_W(mask, value) == False):
        print('error in edio24.config_W')
    elif ch == 'd':
      print('Testing Digital I/O ...')
      print('connect pins PO[0-3] <--> PO[4-7] and P1[0-3] <--> P1[4-7]')
      mask = 0x00ffff
      val = 0x00f0f0
      if (edio24.DConfig_W(mask, val) == False):
        print('error in edio24.config_W')
      val2 = edio24.DConfig_R() & 0xffffff
      print('Digital Port tristate Register = ', hex(val2))
      value = int(input('Enter a number [0-0xff] '),16)
      value = (value & 0xf) | ((value << 4) & 0xf00)
      edio24.DOut(0xf0f, value)
      out = edio24.DOut_R() 
      out = (out & 0xf) | ((out >> 4) & 0xf0)
      inpt = edio24.DIn()
      inpt = (inpt & 0xf) | ((inpt >> 4) & 0xf0)
      print('The number you entered = ', hex(inpt), '  Latched value = ', hex(out))
    elif ch == 'r':
      edio24.Reset()
    elif ch == 'R':
      print('Reading Settings Memory.')
      for i in range(0x1f):
        value = edio24.SettingsMemory_R(i, 1)
        print('address: ',hex(i), '    value =', value)
    elif ch == 's':
      status = edio24.Status()
      print('status = ', hex(status))
    elif ch == 'n':
      value = edio24.NetworkConfig()
      print('Network configuration values: ')
      print('  IP address = ', value[0])
      print('  subnet mask = ', value[1])
      print('  gateway = ', value[2])
      print('  MAC:', \
            hex(edio24.device.MAC>>40&0xff)[2:].zfill(2)+':'+\
            hex(edio24.device.MAC>>32&0xff)[2:].zfill(2)+':'+\
            hex(edio24.device.MAC>>24&0xff)[2:].zfill(2)+':'+\
            hex(edio24.device.MAC>>16&0xff)[2:].zfill(2)+':'+\
            hex(edio24.device.MAC>>8&0xff)[2:].zfill(2)+':'+\
            hex(edio24.device.MAC&0xff)[2:].zfill(2))
    elif ch == 'e':
      edio24.device.sock.close()
      exit(0)
    else:
      pass

if __name__ == "__main__":
  main()
