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

from usb_dioSS import *   # Solid State Relay class
import sys
import time

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  # initalize the class
  try:
     dev = usb_dio96H()       # USB-DIO96H
  except:
    try:
      dev = usb_dio96H_50()   # USB-DIO96H-50
    except:
      try:
        dev = usb_1096HFS()   # USB-1096HFS
      except:
        print('No device found')
      return

  while True :
    print("\nUSB-DIO96H/USB_DIO96H-50/USB-1096HFS Testing")
    print("----------------")
    print("Hit 'b' to blink LED")
    print("Hit 'c' to test counter")
    print("Hit 'A' to read all inputs")
    print("Hit 's' to get status")
    print("Hit 'j' for information")
    print("Hit 'n' to get serial number")
    print("Hit 'd' to test digital I/O ")
    print("Hit 'r' to reset the device.")
    print("Hit 't' to test digital bit I/O")
    print("Hit 'e' to exit")

    ch = input('\n')

    if ch == 'b':
      dev.Blink()
    elif ch == 'e':
      dev.h.close()
      exit(0)
    elif ch == 'A':
      value = dev.GetAll()
      for i in range(4):
        print('Port',i,'A =',hex(value[4*i+0]))
        print('Port',i,'B =',hex(value[4*i+1]))
        print('Port',i,'C Low =',hex(value[4*i+2]))
        print('Port',i,'C High =',hex(value[4*i+3]))
    elif ch == 'c':
      print('Test the Counter.  Connect pin P2C0 <--> CTR')
      dev.CInit()
      for i in range(20):
        dev.DOut(dev.DIO_PORT1C_LOW, 1)
        time.sleep(0.05)
        dev.DOut(dev.DIO_PORT1C_LOW, 0)
        time.sleep(0.05)
        print('Counter =',dev.CIn())
    elif ch == 'd':
      print('Testing Digital I/O')
      print('Connect pins P1A0 through P1A7 <==> P1B0 through P1B7')
      print('Connect pins P1C0 through P1C3 <==> P1C4 through P1C7')
      while True:
        value = int(input('Enter a byte number [0-0xff]: '),16)
        dev.DOut(dev.DIO_PORT0A, value)
        value2 = dev.DIn(dev.DIO_PORT0B)
        print('The number you entered =',hex(value2))
        value = int(input('Enter a nibble [0-0xf]: '),16)
        dev.DOut(dev.DIO_PORT0C_LOW, value)
        value2 = dev.DIn(dev.DIO_PORT0C_HI)
        print('The number you entered =',hex(value2))
        if (toContinue() != True):
          break
    elif ch == 't':
      print('Testing Digital Bit I/O')
      print('Connect pins P1A0 through P1A7 <==> P1B0 through P1B7')
      while True:
        pin = int(input('Enter a pin [0-7]: '))
        bit_value = int(input('Enter a bit value [0|1]: '))
        dev.DBitOut(dev.DIO_PORT0A, pin, bit_value)
        value = dev.DBitIn(dev.DIO_PORT0B, pin)
        print('The value you read is',value)
        if toContinue() != True:
          break
    elif ch == 'n':
      print("Serial No: %s" % dev.h.get_serial_number_string())
    elif ch == 'j':
      print("Manufacturer: %s" % dev.h.get_manufacturer_string())
      print("Product: %s" % dev.h.get_product_string())
      print("Serial No: %s" % dev.h.get_serial_number_string())
    elif ch == 's':
      status = dev.Status()
      print('Status =',hex(status))
      
if __name__ == "__main__":
  main()

