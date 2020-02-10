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
from usb_1208FS_Plus import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb1208FS_Plus = usb_1208FS_Plus()
    print("USB-1208FS_Plus device found.")
  except:
    print('No USB-1208FS-Plus device found.')
    return

# print out the calibration tables
  for chan in range(4):
    for gain in range(8):
      print('Calibration Table (Differential): Chan =',chan,' Range = ',gain, \
            'Slope = ',format(usb1208FS_Plus.table_AIn[chan][gain].slope,'.5f'),\
            'Intercept = ',format(usb1208FS_Plus.table_AIn[chan][gain].intercept,'5f'))
  print('')
  for chan in range(8):
    print('Calibration Table (Single Ended): Chan =',chan, \
      'Slope = ',format(usb1208FS_Plus.table_AInSE[chan].slope,'.5f'),\
            'Intercept = ',format(usb1208FS_Plus.table_AInSE[chan].intercept,'5f'))
      
# print last known calibration date:
  mdate = usb1208FS_Plus.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1208FS_Plus.wMaxPacketSize)

  while True:
    print("\nUSB-1208FS-Plus Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' for continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'g' to test analog input scan.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' for information.")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb1208FS_Plus.BlinkLED(count)
    elif ch == 'c':
      usb1208FS_Plus.ResetCounter()
      usb1208FS_Plus.DTristateW(0x0, usb1208FS_Plus.PORTA)
      print('Connect DIO0 to CTR0')
      usb1208FS_Plus.DLatchW(0x0, usb1208FS_Plus.PORTA)
      toContinue()
      for i in range(100):
        usb1208FS_Plus.DLatchW(0x1, usb1208FS_Plus.PORTA)
        usb1208FS_Plus.DLatchW(0x0, usb1208FS_Plus.PORTA)
      count = usb1208FS_Plus.Counter()
      print("Count = ", count, "    Should read 100.")
    elif ch == 'I':
      print("Manufacturer: %s" % usb1208FS_Plus.getManufacturer())
      print("Product: %s" % usb1208FS_Plus.getProduct())
      print("Serial No: %s" % usb1208FS_Plus.getSerialNumber())
    elif ch == 'd':
      print('Testing read/write digital port.')
      print('Connect Port A to Port B')
      usb1208FS_Plus.DTristateW(0, usb1208FS_Plus.PORTA)     # Port A Output
      usb1208FS_Plus.DTristateW(0xff, usb1208FS_Plus.PORTB)  # Port B Input
      while True:
        value = int(input('Enter a hex number [0-0xff]: '),16) & 0xff
        usb1208FS_Plus.DLatchW(value, usb1208FS_Plus.PORTA)
        valueR = usb1208FS_Plus.DPortR(usb1208FS_Plus.PORTB)
        print("The number you entered: ", hex(value), "  Latched value: ", hex(valueR))
        if toContinue() != True:
          break
    elif ch == 'e':
      usb1208FS_Plus.udev.close()
      exit(0)
    elif ch == 'S':
      print(hex(usb1208FS_Plus.Status()))
      usb1208FS_Plus.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb1208FS_Plus.getSerialNumber())
    elif ch == 'i':
      print('Connect pin 1 <-> pin 21')
      mode = int(input('Enter 1 for Differential, 0 for Single Ended: '))
      chan = 0
      print('\t\t1. +/- 20.V')
      print('\t\t2. +/- 10.V')
      print('\t\t3. +/- 5V')
      print('\t\t4. +/- 4V')
      print('\t\t5. +/- 2.5V')
      print('\t\t6. +/- 2V')
      print('\t\t7. +/- 1.25V')
      print('\t\t8. +/- 1V')
      gain = int(input('Select gain [1-8]: '))
      if gain == 1:
        gain = usb1208FS_Plus.BP_20V
      elif gain == 2:
        gain = usb1208FS_Plus.BP_10V
      elif gain == 3:
        gain = usb1208FS_Plus.BP_5V
      elif gain == 4:
        gain = usb1208FS_Plus.BP_4V
      elif gain == 5:
        gain = usb1208FS_Plus.BP_2_5V
      elif gain == 6:
        gain = usb1208FS.BP_Plus_2V
      elif gain == 7:
        gain = usb1208FS_Plus.BP_1_25V
      elif gain == 8:
        gain = usb1208FS_Plus.BP_1V
      usb1208FS_Plus.DTristateW(0x0, usb1208FS_Plus.PORTA)
      for i in range(20):
        usb1208FS_Plus.DPortW(0,usb1208FS_Plus.PORTA);
        time.sleep(0.01)
        value = usb1208FS_Plus.AIn(chan, mode, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1208FS_Plus.volts(gain, value),'.3f'),'V')
        usb1208FS_Plus.DPortW(1,usb1208FS_Plus.PORTA)
        time.sleep(0.01)
        value = usb1208FS_Plus.AIn(chan, mode, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1208FS_Plus.volts(gain, value),'.3f'),'V')
      
if __name__ == "__main__":
  main()
