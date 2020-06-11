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
from usb_1608FS_Plus import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb1608FS_Plus = usb_1608FS_Plus()
    print("USB-1608FS_Plus device found.")
  except:
    print('No USB-1608FS-Plus device found.')
    return

# print out the calibration tables
  for chan in range(8):
    for gain in range(8):
      print('Calibration Table (Differential): Chan =',chan,' Range = ',gain, \
            'Slope = ',format(usb1608FS_Plus.table_AIn[chan][gain].slope,'.5f'),\
            'Intercept = ',format(usb1608FS_Plus.table_AIn[chan][gain].intercept,'5f'))

# print last known calibration date:
  mdate = usb1608FS_Plus.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1608FS_Plus.wMaxPacketSize)

  while True:
    print("\nUSB-1608FS-Plus Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' for continous sampling")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' to test analog input scan.")
    print("Hit 'M' for information.")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb1608FS_Plus.BlinkLED(count)
    elif ch == 'c':
      usb1608FS_Plus.ResetCounter()
      usb1608FS_Plus.DTristateW(0xf0)
      print('Connect DIO0 to CTR0')
      usb1608FS_Plus.DLatchW(0x0)
      toContinue()
      for i in range(100):
        usb1608FS_Plus.DLatchW(0x1)
        usb1608FS_Plus.DLatchW(0x0)
      count = usb1608FS_Plus.Counter()
      print("Count = ", count, "    Should read 100.")
    elif ch == 'd':
      print("Testing Digital I/O ...")
      print("connect pins DIO[0-3] <--> DIO[4-7]")
      usb1608FS_Plus.DTristateW(0xf0)
      print("Digital Port Tristate Register = ", hex(usb1608FS_Plus.DTristateR()))
      while True:
        value = int(input('Enter a byte number [0-0xf]: '),16) & 0xf
        usb1608FS_Plus.DLatchW(value)
        value2 = usb1608FS_Plus.DLatchR()
        value3 = usb1608FS_Plus.DPort() >> 4
        print("The number you entered: ", hex(value3), "  Latched value: ", hex(value2))
        if toContinue() != True:
          break
    elif ch == 'i':
      print('Connect pin 1 <-> pin 21')
      chan = int(input('Select channel [0-7]: '))
      print('\t\t1. +/- 10.V')
      print('\t\t2. +/- 5.V')
      print('\t\t3. +/- 2.5V')
      print('\t\t4. +/- 2.0V')
      print('\t\t5. +/- 1.25V')
      print('\t\t6. +/- 1.0V')
      print('\t\t7. +/- 0.625V')
      print('\t\t8. +/- 0.3125V')
      gain = int(input('Select gain [1-8]: '))
      if gain == 1:
        gain = usb1608FS_Plus.BP_10_00V
      elif gain == 2:
        gain = usb1608FS_Plus.BP_5_00V
      elif gain == 3:
        gain = usb1608FS_Plus.BP_2_50V
      elif gain == 4:
        gain = usb1608FS_Plus.BP_2_00V
      elif gain == 5:
        gain = usb1608FS_Plus.BP_1_250V
      elif gain == 6:
        gain = usb1608FS.BP_Plus_2_00V
      elif gain == 7:
        gain = usb1608FS_Plus.BP_0_625V
      elif gain == 8:
        gain = usb1608FS_Plus.BP_0_3125V
      usb1608FS_Plus.DTristateW(0xf0)
      for i in range(20):
        usb1608FS_Plus.DPortW(0)
        time.sleep(0.01)
        value = usb1608FS_Plus.AIn(chan, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1608FS_Plus.volts(gain, value),'.3f'),'V')
        usb1608FS_Plus.DPortW(1)
        time.sleep(0.01)
        value = usb1608FS_Plus.AIn(chan, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1608FS_Plus.volts(gain, value),'.3f'),'V')
    elif ch == 'I':
      print('Testing Analog input scan')
      frequency = float(input('Enter desired frequency [Hz]: '))
      count = int(input('Enter number of scans [1-1024]: '))
      nchan = int(input('Enter number of channels [1-8]: '))
      print("\t\t1. +/- 10.0V")
      print("\t\t2. +/- 5.0V")
      print("\t\t3. +/- 2.5V")
      print("\t\t4. +/- 2.V")
      print("\t\t5. +/- 1.25V")
      print("\t\t6. +/- 1.0V")
      print("\t\t7. +/- 0.625V")
      print("\t\t8. +/- 0.3125V")
      gain = int(input("Select gain [1-8]: "))
      if gain == 1:
        gain = usb1608FS_Plus.BP_10_00V
      elif gain == 2:
        gain = usb1608FS_Plus.BP_5_00V
      elif gain == 3:
        gain = usb1608FS_Plus.BP_2_50V
      elif gain == 4:
        gain = usb1608FS_Plus.BP_2_00V
      elif gain == 5:
        gain = usb1608FS_Plus.BP_1_25V
      elif gain == 6:
        gain = usb1608FS_Plus.BP_1_00V
      elif gain == 7:
        gain = usb1608FS_Plus.BP_0_625V
      elif gain == 8:
        gain = usb1608FS_Plus.BP_0_3125V
      gains = [0]*8
      channels = 0
      for chan in range(nchan):
        gains[chan] = gain
        channels |= (0x1 << chan)
      usb1608FS_Plus.AInConfigW(gains)

      if frequency < 100:
        options = usb1608FS_Plus.IMMEDIATE_TRANSFER_MODE
      else:
        options = usb1608FS_Plus.BLOCK_TRANSFER_MODE

      usb1608FS_Plus.AInScanStop()
      usb1608FS_Plus.AInScanClearFIFO()
      
      usb1608FS_Plus.AInScanStart(count, frequency, channels, options)
      dataAIn = usb1608FS_Plus.AInScanRead(count)
      for scan in range(count):
        for channel in range(nchan):
          ii = scan*nchan + channel
          dataAIn[ii] = round(dataAIn[ii]*usb1608FS_Plus.table_AIn[channel][gain].slope + usb1608FS_Plus.table_AIn[channel][gain].intercept)
          print("Channel {0:d}  Sample[{1:d}] = ".format(channel, ii), hex(dataAIn[ii])," Volts = {0:7.4f}".format(usb1608FS_Plus.volts(gain,dataAIn[ii])))
      usb1608FS_Plus.AInScanStop()
      usb1608FS_Plus.AInScanClearFIFO()
    elif ch == 'C':
      print('Testing USB-1608FS-Plus Analog Input Scan in Continuous mode')
      nchan = int(input('Enter number of channels [1-8]: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      print('Hit any key to exit')
      if frequency < 100:
        options = usb1608FS_Plus.IMMEDIATE_TRANSFER_MODE
      else:
        options = 0x0
      channels = 0
      for i in range(nchan):
        channels |= (0x1 << i)
      usb1608FS_Plus.AInScanStop()
      usb1608FS_Plus.AInScanClearFIFO()
      usb1608FS_Plus.AInScanStart(0, frequency, channels, options)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      j = 0
      while True:
        raw_data = usb1608FS_Plus.AInScanRead(128)
        print('Scan =', j, 'samples returned =', len(raw_data))
        j += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1608FS_Plus.AInScanStop()
      usb1608FS_Plus.AInScanClearFIFO()
    elif ch == 'M':
      print("Manufacturer: %s" % usb1608FS_Plus.getManufacturer())
      print("Product: %s" % usb1608FS_Plus.getProduct())
      print("Serial No: %s" % usb1608FS_Plus.getSerialNumber())
    elif ch == 'e':
      usb1608FS_Plus.udev.close()
      exit(0)
    elif ch == 'r':
      usb1608FS_Plus.Reset()
    elif ch == 'S':
      print(hex(usb1608FS_Plus.Status()))
      usb1608FS_Plus.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb1608FS_Plus.getSerialNumber())


if __name__ == "__main__":
  main()
