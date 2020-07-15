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
    print("Hit 'M' for information.")
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
    elif ch == 'o':
      dio32HS.DTristateW(dio32HS.PORTA, 0x0)                # port A all output
      dio32HS.OutScanStop()
      dio32HS.OutScanClearFIFO()
      print('Test of DIO Output Scan.  Connect P0D0 to scope')
      frequency = float(input('Enter frequency [Hz]: '))
      count = 6000
      outData = [0]*count               # make a square wave
      for i in range(count,2):
        outData[i] = 0
        outData[i+1] = 1
      options = 0
      for i in range(10*int(frequency)):
        dio32HS.OutScanStart(dio32HS.PORT0, count, 0, frequency, options)
        dio32HS.OutScanWrite(outData)
        time.sleep(count/frequency)
      dio32HS.OutScanStop()
      dio32HS.OutScanClearFIFO()
    elif ch == 'p':
      print('Test of Patten Triggering.  Connect Port A to Port B')
      pattern = int(input('Enter bit pattern to trigger [0-0xffff]: '), 16)
      dio32HS.DTristateW(dio32HS.PORTA, 0x0)                # port A all output
      dio32HS.DTristateW(dio32HS.PORTB, 0xffff)             # port B all input
      dio32HS.DLatchW(dio32HS.PORTA, [0x0, 0x0])            # write 0 to output port
      options = 0x1                                         # Trigger on Port 1 when equal to pattern
      dio32HS.PatternDetectConfig(pattern, 0xffff, options) # Configure Pattern Detection trigger
      print('Pattern = %#x' % (pattern))
      count = 2                                             # total number of scans
      options = 0x2                                         # use pattern detection trigger
      frequency = 10000                                     # sample at 10 kHz
      dio32HS.InScanStart(dio32HS.PORT1, count, 0, frequency, options)
      for i in range(0xffff):
        dio32HS.DLatchW(dio32HS.PORTA, [i,0x0])             # write a trial number
        if not dio32HS.Status() & dio32HS.IN_SCAN_RUNNING:
          print('Pattern Detected! pattern = %#x' % i)
          break
      data = dio32HS.InScanRead()
      print('data = [%#x, %#x]' % (data[0], data[1]))
      dio32HS.InScanStop()
      dio32HS.InScanClearFIFO()
    elif ch == 'e':
      dio32HS.udev.close()
      exit(0)
    elif ch == 'r':
      dio32HS.Reset()
    elif ch == 'S':
      print('Status =', hex(dio32HS.Status()))
      dio32HS.printStatus()
    elif ch == 'M':
      print("Manufacturer: %s" % dio32HS.getManufacturer())
      print("Product: %s" % dio32HS.getProduct())
      print("Serial No: %s" % dio32HS.getSerialNumber())
    elif ch == 's':
      print("Serial No: %s" % dio32HS.getSerialNumber())
    elif ch == 'v':
      print("FPGA version %s" % (dio32HS.FPGAVersion()))
      

if __name__ == "__main__":
  main()

