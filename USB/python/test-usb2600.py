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
from usb_2600 import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb2600 = usb_2633()
    print("USB-2633 device found.")
  except:
    try:
      usb2600 = usb_2637()
      print("USB-2637 device found.")
    except:
      try:
        usb2600 = usb_2623()
        print("USB-usb-2623 device found.")
      except:
        print("No USB-2623 device found.")
        return

  # print out the calibration tables
  print('\nCalibration Analog Input Table:')
  for gain in range(usb2600.NGAIN):
    print('  Range =',gain, \
          'Slope =',format(usb2600.table_AIn[gain].slope,'.5f'),\
          'Intercept =',format(usb2600.table_AIn[gain].intercept,'5f'))

  # print last known calibration date:
  mdate = usb2600.CalDate()
  print('\nAnalog Input MFG Calibration date: ', mdate)

  if usb2600.productID == usb2600.USB_2637_PID:
    print('\nCalibration Analog Output Table:')
    for channel in range(usb2600.NCHAN_AO):
      print('  Channel =',channel, \
          'Slope =',format(usb2600.table_AOut[channel].slope,'.5f'),\
          'Intercept =',format(usb2600.table_AOut[channel].intercept,'5f'))
    # print last known calibration date:
    mdate = usb2600.CalDate()
    print('\nAnalog Output MFG Calibration date: ', mdate)

    
  print("\nwMaxPacketSize = ", usb2600.wMaxPacketSize)

  while True:
    print("\nUSB-2633/2637 Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' to test continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    if usb2600.productID == usb2600.USB_2637_PID:
      print("Hit 'o' to test Analog Output")
      print("Hit 'O' to test Analog Output Scan")
    print("Hit 'M' for information.")
    print("Hit 't' to test timers")
    print("Hit 'T' to get temperature")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")
    print("Hit 'v' to get version numbers")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb2600.BlinkLED(count)
    elif ch == 'c':
      counter = int(input('Enter counter [0-3]: '))
      if counter == 0:                    
        usb2600.CounterInit(usb2600.COUNTER0)
        print("Connect DIO0 to CTR0")
      elif counter == 1:
        usb2600.CounterInit(usb2600.COUNTER1)
        print("Connect DIO0 to CTR1")
      usb2600.DTristateW(0xf0)
      toContinue()
      for i in range(100):
        usb2600.DLatchW(0x0)
        usb2600.DLatchW(0x1)
      print("Count = %d.  Should read 100" % (usb2600.Counter(counter)))
    elif ch == 'M':
      print("Manufacturer: %s" % usb2600.getManufacturer())
      print("Product: %s" % usb2600.getProduct())
      print("Serial No: %s" % usb2600.getSerialNumber())
    elif ch == 'e':
      usb2600.udev.close()
      exit(0)
    elif ch == 'r':
      usb2600.Reset()
    elif ch == 'S':
      print('Status =', hex(usb2600.Status()))
      usb2600.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb2600.getSerialNumber())
    elif ch == 'T':
      print("Internal temperature = %.2f deg C or %.2f deg " % (usb2600.Temperature(), usb2600.Temperature()*9./5. + 32.))
    elif ch == 'v':
      print("FPGA version %s" % (usb2600.FPGAVersion()))


if __name__ == "__main__":
  main()
