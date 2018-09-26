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

from usb_1208FS import *
import time
import sys
import fcntl
import os
import math

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False
  
def main():
  # initalize the class
  usb1208FS = usb_1208FS()

  print('wMaxPacketSize =', usb1208FS.wMaxPacketSize)  
  usb1208FS.DConfig(usb1208FS.DIO_PORTA, usb1208FS.DIO_DIR_OUT)
  usb1208FS.DConfig(usb1208FS.DIO_PORTB, usb1208FS.DIO_DIR_IN)
  usb1208FS.DOut(usb1208FS.DIO_PORTA, 0x0)

  usb1208FS.MemRead(0x200, 4)

  while True:
    print("\nUSB-1208FS Testing")
    print("----------------")
    print("Hit 'a' to test analog output scan.")
    print("Hit 'A' for continuous output scan.")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to test counter. ")
    print("Hit 'd' to test digital I/O.")
    print("Hit 'e' to exit.")
    print("Hit 'f' to get all values")
    print("Hit 'g' to test analog input scan.")
    print("Hit 'i' to test analog input. (differential)")
    print("Hit 'j' to test analog input. (single ended)")
    print("Hit 'I' for information.")
    print("Hit 'o' to test analog output.")
    print("Hit 'r' to reset the device.")
    print("Hit 'S' to get status")
    print("Hit 's' to get serial number.")

    ch = input('\n')

    if ch == 'b':
      usb1208FS.Blink()
    elif ch == 'c':
      usb1208FS.CInit()                       # initialize the counter
      print('Connect pin 20 and 21')
      for i in range(20):
        usb1208FS.DOut(usb1208FS.DIO_PORTA, 1);
        time.sleep(.01)
        usb1208FS.DOut(usb1208FS.DIO_PORTA, 0);
        print('Counter =', usb1208FS.CIn())   # read the current count
    elif ch == 'd':
      print('Testing Digital I/O ...')
      print('Connect pins 21 through 28 <--> 32 through 39  (Port A to Port B)')
      usb1208FS.DConfig(usb1208FS.DIO_PORTA, 0x0)   # Port A output
      usb1208FS.DConfig(usb1208FS.DIO_PORTB, 0xff)  # Port B input
      usb1208FS.DOut(usb1208FS.DIO_PORTA, 0x0)
      while (True):
        try:
          num = int(input('Enter a byte number [0x0-0xff]: '),16)
          usb1208FS.DOut(usb1208FS.DIO_PORTA, num)
          value = usb1208FS.DIn(usb1208FS.DIO_PORTB)
          print('PortB: The number you entered =', hex(value))
          for i in range(8):
            value = usb1208FS.DBitIn(usb1208FS.DIO_PORTB, i)
            print('Port B Bit',i,' =', hex(value))
        except:
          pass
        if (toContinue() != True):
          break
    elif ch == 'e':
      usb1208FS.udev.close()
      exit(0)
    elif ch == 'f':
      print('Get all values')
      value = usb1208FS.GetAll()
      for i in range(4):
        print('Differential Reference Low channel[',i,']  = ',hex(value[i]))
      for i in range(4):
        print('Differential Reference High channel[',i,']  = ',hex(value[i+4]))
      for i in range(8):
        print('Single Ended Input channel[',i,'] = ', hex(value[i+8]))
      print('DIO Port A = ', hex(value[16]))
      print('DIO Port B = ', hex(value[17]))
    elif ch == 'i':
      print('Connect pin 1 <-> pin 21 and pin 2 <-> pin 3')
      chan = int(input('Select channel [0-3]: '))
      print("\t\t1. +/- 20.V");
      print("\t\t2. +/- 10.V")
      print("\t\t3. +/- 5.V")
      print("\t\t4. +/- 4.V")
      print("\t\t5. +/- 2.5V")
      print("\t\t6. +/- 2.0V")
      print("\t\t7. +/- 1.25V")
      print("\t\t8. +/- 1.0V")
      gain = int(input("Select gain [1-8]: "))
      if gain == 1:
        gain = usb1208FS.BP_20_00V
      elif gain == 2:
        gain = usb1208FS.BP_10_00V
      elif gain == 3:
        gain = usb1208FS.BP_5_00V
      elif gain == 4:
        gain = usb1208FS.BP_4_00V
      elif gain == 5:
        gain = usb1208FS.BP_2_50V
      elif gain == 6:
        gain = usb1208FS.BP_2_00V
      elif gain == 7:
        gain = usb1208FS.BP_1_25V
      elif gain == 8:
        gain = usb1208FS.BP_1_00V
      for i in range(20):
        usb1208FS.DOut(usb1208FS.DIO_PORTA, 0)
        time.sleep(0.01)
        value = usb1208FS.AIn(chan, gain)
        print('Channel: ',chan,' value =', hex(value),'\t',format(usb1208FS.volts(gain, value),'.3f'),'V')
        usb1208FS.DOut(usb1208FS.DIO_PORTA, 1)
        time.sleep(0.01)
        value = usb1208FS.AIn(chan, gain);
        print('Channel: ',chan,' value = ', hex(value),'\t',format(usb1208FS.volts(gain, value),'.3f'),'V')
    elif ch == 'g':
      print('Testing Analog input scan')
      freq = int(input('Enter desired frequency [Hz]: '))
      count = int(input('Enter number of samples [1-1024]: '))
      chan = int(input('Enter channel [0-3]: '))
      print("\t\t1. +/- 20.V");
      print("\t\t2. +/- 10.V")
      print("\t\t3. +/- 5.V")
      print("\t\t4. +/- 4.V")
      print("\t\t5. +/- 2.5V")
      print("\t\t6. +/- 2.0V")
      print("\t\t7. +/- 1.25V")
      print("\t\t8. +/- 1.0V")
      print("\t\t9. Single Ended +/- 10V")
      gain = int(input("Select gain [1-8]: "))
      if gain == 1:
        gain = usb1208FS.BP_20_00V
      elif gain == 2:
        gain = usb1208FS.BP_10_00V
      elif gain == 3:
        gain = usb1208FS.BP_5_00V
      elif gain == 4:
        gain = usb1208FS.BP_4_00V
      elif gain == 5:
        gain = usb1208FS.BP_2_50V
      elif gain == 6:
        gain = usb1208FS.BP_2_00V
      elif gain == 7:
        gain = usb1208FS.BP_1_25V
      elif gain == 8:
        gain = usb1208FS.BP_1_00V
      gains = [0]*8
      for i in range(8):
        gains[i] = gain
      options = usb1208FS.AIN_EXECUTION | usb1208FS.AIN_GAIN_QUEUE
      data = usb1208FS.AInScan(chan,chan,gains,count,freq,options)
      for i in range(count):
        print('data[',i,'] = ', hex(data[i]),'\t',format(usb1208FS.volts(gain, data[i]),'.3f'),'V')
      usb1208FS.AInStop()
    elif ch == 'j':
      print('Testing Analog Input Single Ended Mode')
      chan = int(input('Select channel [0-7]: '))
      gain = usb1208FS.SE_10_00V
      value = usb1208FS.AIn(chan, gain)
      print('Channel: ',chan,' value = ', hex(value),'\t',format(usb1208FS.volts(gain, value),'.3f'),'V')
    elif ch == 'o':
      print('Testing the analog output for the USB-1208FS')
      chan = int(input('Enter channel [0-1]: '))
      value = int(input('Enter value [0 - 0xfff]: '),16)
      usb1208FS.AOut(chan,value)
    elif ch == 'a':
      out_data = [0]*512
      print('Testing Analog Output Scan')
      frequency = int(input('Entered desired frequency [Hz]: '))
      for i in range(512):
        if i%2 == 0:
          out_data[i] = 0
        else:
          out_data[i] = 0xfff
      for j in range(5):
        usb1208FS.AOutScan(0,0,frequency,out_data,1)
      usb1208FS.AOutStop()
    elif ch == 'A':
      print('Analog output scan continuous. Hit <space> <CR> to Stop')
      frequency = 1000
      nSamples = 32*32
      out_data = [0]*1024
      for i in range(512):
        value = int(0.5*(math.sin(2*math.pi*i/128) + 1.)*0xffff)
        out_data[2*i] = value & 0xff
        out_data[2*i+1] = (value>>8) & 0xff
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      usb1208FS.AOutScan(0,0,frequency,out_data,0)
      while True:
        usb1208FS.AOutWrite(out_data, 1000)
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
      usb1208FS.AOutStop()
      try:  # clear out the input buffer
        ret = usb1208FS.udev.interruptRead(libusb1.LIBUSB_ENDPOINT_IN | 1, 10, 100)
      except:
        pass
    elif ch == 's':
        print("Serial No: %s" % usb1208FS.getSerialNumber())
    elif ch == 'I':
      print("Manufacturer: %s" % usb1208FS.getManufacturer())
      print("Product: %s" % usb1208FS.getProduct())
      print("Serial No: %s" % usb1208FS.getSerialNumber())
    elif ch == 'S':
      usb1208FS.printStatus()

if __name__ == "__main__":
  main()
