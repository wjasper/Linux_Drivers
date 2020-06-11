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
import math
from usb_1408FS_Plus import *

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
    usb1408FS_Plus = usb_1408FS_Plus()
    print("USB-1408FS_Plus device found.")
  except:
    print('No USB-1408FS-Plus device found.')
    return

# print out the calibration tables
  for chan in range(4):
    for gain in range(8):
      print('Calibration Table (Differential): Chan =',chan,' Range = ',gain, \
            'Slope = ',format(usb1408FS_Plus.table_AIn[chan][gain].slope,'.5f'),\
            'Intercept = ',format(usb1408FS_Plus.table_AIn[chan][gain].intercept,'5f'))
  print('')
  for chan in range(8):
    print('Calibration Table (Single Ended): Chan =',chan, \
      'Slope = ',format(usb1408FS_Plus.table_AInSE[chan].slope,'.5f'),\
            'Intercept = ',format(usb1408FS_Plus.table_AInSE[chan].intercept,'5f'))
      
# print last known calibration date:
  mdate = usb1408FS_Plus.CalDate()
  print('\nMFG Calibration date: ', mdate)

  print("wMaxPacketSize = ", usb1408FS_Plus.wMaxPacketSize)

  while True:
    print("\nUSB-1408FS-Plus Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'd' to read/write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'i' to test analog input.")
    print("Hit 'I' to test Analog Input Scan")
    print("Hit 'C' for continous sampling")
    print("Hit 'M' for information.")
    print("Hit 'o' to test Analog Output.")
    print("Hit 'O' for Analog Output Scan")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb1408FS_Plus.BlinkLED(count)
    elif ch == 'c':
      usb1408FS_Plus.ResetCounter()
      usb1408FS_Plus.DTristateW(0x0, usb1408FS_Plus.PORTA)
      print('Connect DIO0 to CTR0')
      usb1408FS_Plus.DLatchW(0x0, usb1408FS_Plus.PORTA)
      toContinue()
      for i in range(100):
        usb1408FS_Plus.DLatchW(0x1, usb1408FS_Plus.PORTA)
        usb1408FS_Plus.DLatchW(0x0, usb1408FS_Plus.PORTA)
      count = usb1408FS_Plus.Counter()
      print("Count = ", count, "    Should read 100.")
    elif ch == 'M':
      print("Manufacturer: %s" % usb1408FS_Plus.getManufacturer())
      print("Product: %s" % usb1408FS_Plus.getProduct())
      print("Serial No: %s" % usb1408FS_Plus.getSerialNumber())
    elif ch == 'd':
      print('Testing read/write digital port.')
      print('Connect Port A to Port B')
      usb1408FS_Plus.DTristateW(0, usb1408FS_Plus.PORTA)     # Port A Output
      usb1408FS_Plus.DTristateW(0xff, usb1408FS_Plus.PORTB)  # Port B Input
      while True:
        value = int(input('Enter a hex number [0-0xff]: '),16) & 0xff
        usb1408FS_Plus.DLatchW(value, usb1408FS_Plus.PORTA)
        valueR = usb1408FS_Plus.DPortR(usb1408FS_Plus.PORTB)
        print("The number you entered: ", hex(value), "  Latched value: ", hex(valueR))
        if toContinue() != True:
          break
    elif ch == 'e':
      usb1408FS_Plus.udev.close()
      exit(0)
    elif ch == 'S':
      print(hex(usb1408FS_Plus.Status()))
      usb1408FS_Plus.printStatus()
    elif ch == 's':
      print("Serial No: %s" % usb1408FS_Plus.getSerialNumber())
    elif ch == 'i':
      print('Connect pin 1 <-> pin 21')
      mode = int(input('Enter 1 for Differential, 0 for Single Ended: '))
      chan = 0
      if mode == 1:  # differential mode
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
          gain = usb1408FS_Plus.BP_20V
        elif gain == 2:
          gain = usb1408FS_Plus.BP_10V
        elif gain == 3:
          gain = usb1408FS_Plus.BP_5V
        elif gain == 4:
          gain = usb1408FS_Plus.BP_4V
        elif gain == 5:
          gain = usb1408FS_Plus.BP_2_5V
        elif gain == 6:
          gain = usb1408FS.BP_Plus_2V
        elif gain == 7:
          gain = usb1408FS_Plus.BP_1_25V
        elif gain == 8:
          gain = usb1408FS_Plus.BP_1V
      else: # single ended
        gain = usb1408FS_Plus.BP_10V

      usb1408FS_Plus.DTristateW(0x0, usb1408FS_Plus.PORTA)

      for i in range(20):
        usb1408FS_Plus.DPortW(0,usb1408FS_Plus.PORTA)
        time.sleep(0.01)
        value = usb1408FS_Plus.AIn(chan, mode, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1408FS_Plus.volts(gain, value),'.3f'),'V')
        usb1408FS_Plus.DPortW(1,usb1408FS_Plus.PORTA)
        time.sleep(0.01)
        value = usb1408FS_Plus.AIn(chan, mode, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1408FS_Plus.volts(gain, value),'.3f'),'V')
    elif ch == 'o':
      channel = int(input('Enter output channel [0-1]: '))
      value = int(input('Enter value [0-4095]: '))
      usb1408FS_Plus.AOut(channel, value)
    elif ch == 'I':
      print("Testing USB-1408FS-Plus Analog Input Scan, Differential Mode")
      nscan = int(input("Enter number of scans: "))
      nchan = int(input('Enter number of channels [1-4]: '))
      frequency = float(input("Enter sampling frequency [Hz]: "))
      gain = int(input("Enter gain/range [0-7]: "))
      channels = 0
      gains = [0]*8
      for chan in range(nchan):
        gains[chan] = gain
        channels |= (0x1 << chan)
      usb1408FS_Plus.AInScanConfigW(gains)
      ranges = usb1408FS_Plus.AInScanConfigR()
      if frequency > 100:
        options = usb1408FS_Plus.DIFFERENTIAL_MODE
      else:
        options = usb1408FS_Plus.DIFFERENTIAL_MODE | usb1408FS_Plus.IMMEDIATE_TRANSFER_MODE;
      usb1408FS_Plus.AInScanStop()
      usb1408FS_Plus.AInScanClearFIFO()
      usb1408FS_Plus.AInScanStart(nscan, 0x0, frequency, channels, options)
      dataAIn = usb1408FS_Plus.AInScanRead(nscan)
      for scan in range(nscan):
        for channel in range(nchan):
          ii = scan*nchan + channel
          dataAIn[ii] = round(dataAIn[ii]*usb1408FS_Plus.table_AIn[channel][gain].slope + usb1408FS_Plus.table_AIn[channel][gain].intercept)
          print("Channel {0:d}  Sample[{1:d}] = ".format(channel, ii), hex(dataAIn[ii])," Volts = {0:7.4f}".format(usb1408FS_Plus.volts(gain,dataAIn[ii])))
      usb1408FS_Plus.AInScanStop()
      usb1408FS_Plus.AInScanClearFIFO()
    elif ch == 'C':
      print('Testing USB-1408FS-Plus Analog Input Scan in Continuous mode')
      nchan = int(input('Enter number of channels [1-4]: '))
      frequency = float(input('Enter sampling frequency [Hz]: '))
      print('Hit any key to exit')
      if frequency > 100:
        options = usb1408FS_Plus.DIFFERENTIAL_MODE
      else:
        options = usb1408FS_Plus.DIFFERENTIAL_MODE | usb1408FS_Plus.IMMEDIATE_TRANSFER_MODE;
      channels = 0
      for i in range(nchan):
        channels |= (0x1 << i)
      usb1408FS_Plus.AInScanStop()
      usb1408FS_Plus.AInScanClearFIFO()
      usb1408FS_Plus.AInScanStart(0, 0, frequency, channels, options)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      j = 0
      while True:
        raw_data = usb1408FS_Plus.AInScanRead(128)
        print('Scan =', j, 'samples returned =', len(raw_data))
        j += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1408FS_Plus.AInScanStop()
      usb1408FS_Plus.AInScanClearFIFO()
    elif ch == 'O':
      options = 0x3   # output channel 0 and 1 output scan
      print('Test of Analog Ouput Scan')
      print('Hook up scope to VDAC Channel 0 or 1')
      frequency = 2*float(input('Enter desired frequency of square wave [Hz]: '))
      data = [0]*256  # holds 12 bit unsigned analog outout data
      for i in range(32):
        data[4*i] = 0X0
        data[4*i+1] = 0x800
        data[4*i+2] = 0xfff
        data[4*i+3] = 0xcff;
      usb1408FS_Plus.AOutScanStop()
      usb1408FS_Plus.AOutScanStart(0, frequency, options)
      print("Hit 's <CR>' to stop")
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        try:
          usb1408FS_Plus.AOutScanWrite(data)
        except:
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb1408FS_Plus.AOutScanStop()
          print("AOutScan exception")
          break
        c = sys.stdin.readlines()
        if (len(c) != 0):
          fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
          usb1408FS_Plus.AOutScanStop()
          break

      
if __name__ == "__main__":
  main()
