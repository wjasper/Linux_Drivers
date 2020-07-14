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

import time
import sys
import fcntl
import os
import math
from usb_dio32HS import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    dio32HS = usb_dio32HS()
    print("USB-DIO32HS device found.")
  except:
    print("No USB-DIO32HS device found.")
    return

  print("\nwMaxPacketSize = ", dio32HS.wMaxPacketSize)

  while True:
    print("\nUSB-DIO32HS Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'o' to test DIO Out Scan")
    print("Hit 'p' for pattern triggering")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")
    print("Hit 'v' to get version numbers")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      dio32HS.BlinkLED(count)
    elif ch == 'd':
      print('Testing Digital I/O ...')
      dio32HS.DTristateW(dio32HS.PORTA, 0x0)     # port A all output
      dio32HS.DTristateW(dio32HS.PORTB, 0xffff)  # port B all input
      print("Digital Port A Tristate Register = %#x" % dio32HS.DTristateR(dio32HS.PORTA));
      print("Digital Port B Tristate Register = %#x" % dio32HS.DTristateR(dio32HS.PORTB));
      while True:
        value1 = int(input('Enter 2 byte hex number [0-0xffff]: '),16)
        dio32HS.DLatchW(dio32HS.PORTA, [value1, 0x0])
        value2 = dio32HS.DLatchR(dio32HS.PORTA)
        value3 = dio32HS.DPort(dio32HS.PORTA)
        print('The number you entered %#x  Latched value = %#x  Port value = %#x' % (value1, value2, value3))
        for i in range(16):
          print('Bit %d = %d' % (i, (value2>>i)&0x1))
        if toContinue() != True:
          break
    elif ch == 'e':
      dio32HS.udev.close()
      exit(0)
    elif ch == 'r':
      dio32HS.Reset()
    elif ch == 'S':
      print('Status =', hex(dio32HS.Status()))
      dio32HS.printStatus()
    elif ch == 's':
      print("Serial No: %s" % dio32HS.getSerialNumber())
    elif ch == 'v':
      print("FPGA version %s" % (dio32HS.FPGAVersion()))
      

if __name__ == "__main__":
  main()

