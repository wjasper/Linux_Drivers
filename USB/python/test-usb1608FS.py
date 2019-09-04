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

from usb_1608FS import *
import time
import sys
import fcntl
import os

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False
  
def main():
  # initalize the class
  try:
    usb1608FS = usb_1608FS()
  except:
    print('No USB-1608FS device found.')
    return

# print out the calibration tables
  for chan in range(8):
    for gain in range(8):
      print('Calibration Table (Differential): Chan =',chan,' Range = ',gain, \
            'Slope = ',format(usb1608FS.Cal[chan][gain].slope,'.5f'),\
            'Intercept = ',format(usb1608FS.Cal[chan][gain].intercept,'5f'))

  print('wMaxPacketSize =', usb1608FS.wMaxPacketSize)

  usb1608FS.DConfig(usb1608FS.DIO_DIR_OUT)

  while True:
    print("\nUSB-1608FS Testing")
    print("----------------")
    print("Hit 'b' to blink LED.")
    print("Hit 'B' to test digital bits")
    print("Hit 'c' to test counter. ")
    print("Hit 'C' for continous sampling")
    print("Hit 'd' to read digital port.")
    print("Hit 'w' to write digital port.")
    print("Hit 'e' to exit.")
    print("Hit 'g' to test analog input scan.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'I' for information.")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")

    ch = input('\n')

    if ch == 'b':
      usb1608FS.Blink()
    elif ch == 'c':
      usb1608FS.CInit()                       # initialize the counter
      print('Connect pin 38 and 35')
      usb1608FS.DConfig(usb1608FS.DIO_DIR_OUT)
      for i in range(20):
        usb1608FS.DOut(0xff);
        time.sleep(.01)
        usb1608FS.DOut(0);
        print('Counter =', usb1608FS.CIn())   # read the current count
    elif ch == 'd':
      print('Reading Digital I/O ...')
      usb1608FS.DConfig(usb1608FS.DIO_DIR_IN)
      print('DIO Port = ', hex(usb1608FS.DIn()))
    elif ch == 'C':
      print('USB-1608FS Continuous sampling.')
      print('Hit space <CR> to exit')
      freq = float(input('Enter desired frequency [Hz]: '))
      gains = [usb1608FS.BP_10_00V]*8
      chan = 0
      options = 0x0;  # continuous execution, block transfer mode
      data = usb1608FS.AInScan(chan, chan, gains, 0, freq, options)
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      j = 0
      while True:
        raw_data = usb1608FS.AInRead()
        print('Scan =',j,'samples returned =',len(raw_data))
        j += 1
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1608FS.AInStop()
    elif ch == 'w':
      usb1608FS.DConfig(usb1608FS.DIO_DIR_OUT)
      value = int(input('Enter value to write to DIO port [0-ff]: '), 16)
      usb1608FS.DOut(value)
    elif ch == 'B':
      print('Testing Digital Bit I/O')
      print('Connect pins DIO0 through DIO3 <==> DIO4 through DIO7')
      usb1608FS.DConfigBit(0,usb1608FS.DIO_DIR_OUT)
      usb1608FS.DConfigBit(1,usb1608FS.DIO_DIR_OUT)
      usb1608FS.DConfigBit(2,usb1608FS.DIO_DIR_OUT)
      usb1608FS.DConfigBit(3,usb1608FS.DIO_DIR_OUT)
      usb1608FS.DConfigBit(4,usb1608FS.DIO_DIR_IN)
      usb1608FS.DConfigBit(5,usb1608FS.DIO_DIR_IN)
      usb1608FS.DConfigBit(6,usb1608FS.DIO_DIR_IN)
      usb1608FS.DConfigBit(7,usb1608FS.DIO_DIR_IN)
      while True:
        value = int(input('Enter number [0-f]: '),16)
        usb1608FS.DBitOut(0, value & 0x1)
        usb1608FS.DBitOut(1, value & 0x2)
        usb1608FS.DBitOut(2, value & 0x4)
        usb1608FS.DBitOut(3, value & 0x8)
        value = usb1608FS.DBitIn(4) | usb1608FS.DBitIn(5) << 1 | usb1608FS.DBitIn(6) << 2 | usb1608FS.DBitIn(7) << 3
        print('The value you read is',hex(value))
        if toContinue() != True:
          break
    elif ch == 'e':
      usb1608FS.udev.close()
      exit(0)
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
        gain = usb1608FS.BP_10_00V
      elif gain == 2:
        gain = usb1608FS.BP_5_00V
      elif gain == 3:
        gain = usb1608FS.BP_2_50V
      elif gain == 4:
        gain = usb1608FS.BP_2_00V
      elif gain == 5:
        gain = usb1608FS.BP_1_250V
      elif gain == 6:
        gain = usb1608FS.BP_2_00V
      elif gain == 7:
        gain = usb1608FS.BP_0_625V
      elif gain == 8:
        gain = usb1608FS.BP_0_3125V
      usb1608FS.DConfig(usb1608FS.DIO_DIR_OUT)
      for i in range(20):
        usb1608FS.DOut(0)
        time.sleep(0.01)
        value = usb1608FS.AIn(chan, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1608FS.volts(gain, value),'.3f'),'V')
        usb1608FS.DOut(1)
        time.sleep(0.01)
        value = usb1608FS.AIn(chan, gain);
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1608FS.volts(gain, value),'.3f'),'V')
    elif ch == 'g':
      print('Testing Analog input scan')
      freq = float(input('Enter desired frequency [Hz]: '))
      count = int(input('Enter number of scans [1-1024]: '))
      chan = int(input('Enter channel [0-7]: '))
      print("\t\t1. +/- 10.0V");
      print("\t\t2. +/- 5.0V");
      print("\t\t3. +/- 2.5V");
      print("\t\t4. +/- 2.V");
      print("\t\t5. +/- 1.25V");
      print("\t\t6. +/- 1.0V");
      print("\t\t7. +/- 0.625V");
      print("\t\t8. +/- 0.3125V");
      gain = int(input("Select gain [1-8]: "))
      if gain == 1:
        gain = usb1608FS.BP_10_00V
      elif gain == 2:
        gain = usb1608FS.BP_5_00V
      elif gain == 3:
        gain = usb1608FS.BP_2_50V
      elif gain == 4:
        gain = usb1608FS.BP_2_00V
      elif gain == 5:
        gain = usb1608FS.BP_1_25V
      elif gain == 6:
        gain = usb1608FS.BP_1_00V
      elif gain == 7:
        gain = usb1608FS.BP_0_625V
      elif gain == 8:
        gain = usb1608FS.BP_0_3125V
      gains = [0]*8
      gains[chan] = gain
      options = usb1608FS.AIN_EXECUTION
      data = usb1608FS.AInScan(chan,chan,gains,count,freq,options)
      for i in range(count):
        print('data[',i,'] = ', hex(data[i]),'\t',format(usb1608FS.volts(gain, data[i]),'.3f'),'V')
      usb1608FS.AInStop()
    elif ch == 's':
        print("Serial No: %s" % usb1608FS.getSerialNumber())
    elif ch == 'I':
      print("Manufacturer: %s" % usb1608FS.getManufacturer())
      print("Product: %s" % usb1608FS.getProduct())
      print("Serial No: %s" % usb1608FS.getSerialNumber())
    elif ch == 'S':
      usb1608FS.printStatus()

if __name__ == "__main__":
  main()
