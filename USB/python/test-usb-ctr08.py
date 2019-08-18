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

import time
import sys
import fcntl
import os
from usb_ctr import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    ctr08 = usb_ctr08()
    print("USB-CTR08 device found.")
  except:
    print('No USB-CTR08 device found.')
    return

  print("wMaxPacketSize = ", ctr08.wMaxPacketSize)

  while True:
    print("\nUSB-CTR08 Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("hit 'd' to test digital I/O")
    print("Hit 's' to get serial number")
    print("Hit 'S' to get the status")
    print("Hit 'e' to exit.")
    print("Hit 'P' to read the counter parameters")
    print("Hit 'L' to read the scan list")
    print("Hit 'M' to read memory")
    print("Hit 'v' for version.")

    ch = input('\n')
    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      ctr08.BlinkLED(count)
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      ctr08.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(ctr08.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        ctr08.DLatchW(value)
        value2 = ctr08.DLatchR()
        value3 = ctr08.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 's':
      print('Serial Number: ', ctr08.GetSerialNumber())
    elif ch == 'S':
      status = ctr08.Status()
      print('Status = ', hex(status))
      if status & ctr08.PACER_RUNNING:
        print("USB-CTR08: Pacer running.")
      if status & ctr08.SCAN_OVERRUN:
        print("USB-CTR08: Scan overrun.")
      if status & ctr08.SCAN_DONE:
        print("USB-CTR08: Scan done.")
      if status & ctr08.FPGA_CONFIGURED:
        print("USB-CTR08: FPGA configured.")
      if status & ctr08.FPGA_CONFIG_MODE:
        print("USB-CTR08: FPGA config mode.")
    elif ch == 'e':
      ctr08.udev.close()
      exit(0)
    elif ch == 'v':
      version = ctr08.FPGAVersion()
      print("FPGA version:", version)
    elif ch == 'P':
      counter = int(input('Enter counter: '))
      ctr08.CounterParamsR(counter)
      print("Counter:", ctr08.counterParameters[counter].counter,                       \
            "\tMode Options:", hex(ctr08.counterParameters[counter].modeOptions),       \
            "\tCounter Options:", hex(ctr08.counterParameters[counter].counterOptions), \
            "\tGate Options:", hex(ctr08.counterParameters[counter].gateOptions),       \
            "\tOutput Options:", hex(ctr08.counterParameters[counter].outputOptions),   \
            "\tdebounce:", hex(ctr08.counterParameters[counter].debounce))
    elif ch == 'L':
      ctr08.ScanConfigR()
      print("Scan list: ", ctr08.scanList) 
    elif ch == 'M':
      address = int(input("Enter memory address: "),16)
      length = int(input("Enter number of bytes to read: "))
      ctr08.MemAddressW(address)
      print("address = ", hex(ctr08.MemAddressR()))
      print("data: ", ctr08.MemoryR(length))
      
            
            
      
if __name__ == "__main__":
  main()


  
