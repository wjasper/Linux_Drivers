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

from usb_dioLS import *
import time
import sys

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():

  usbdio = usb_DIO24H()       # USB-DIO24H

  while True :
    print("\nUSB-DIO24H Testing")
    print("----------------")
    print("Hit 'b' to blink LED")
    print("Hit 's' to set user id")
    print("Hit 'g' to get user id")
    print("Hit 'i' for information")
    print("Hit 'n' to get serial number")
    print("Hit 'c' to test counter ")
    print("Hit 'd' to test digital I/O ")
    print("Hit 'r' to reset the device.")
    print("Hit 't' to test digital bit I/O")
    print("Hit 'e' to exit")

    ch = input('\n')

    if ch == 'b':
      usbdio.Blink()
    elif ch == 'e':
      usbdio.h.close()
      exit(0)
    elif ch == 'c':
      print('Connect pin 1 and 20')
      usbdio.CInit()                       # initialize the counter
      for i in range(20):
        usbdio.DOut(usbdio.DIO_PORTC_LOW, 1);
        usbdio.DOut(usbdio.DIO_PORTC_LOW, 0);
        print('Counter =', usbdio.CIn())   # read the current count
    elif ch == 'n':
      print("Serial No: %s" % usbdio.h.get_serial_number_string())
    elif ch == 'g':
      print('User ID =', usbdio.GetID())
    elif ch == 's':
      id = int(input('Enter a user id (0-255): '))
      usbdio.SetID(id)
    elif ch == 'i':
      print("Manufacturer: %s" % usbdio.h.get_manufacturer_string())
      print("Product: %s" % usbdio.h.get_product_string())
      print("Serial No: %s" % usbdio.h.get_serial_number_string())
    elif ch == 'd':
      print('Testing Digital I/O ...')
      print('Connect pins 21 through 28 <--> 32 through 39  (Port A to Port B)')
      usbdio.DConfig(usbdio.DIO_PORTA, 0x0)   # Port A output
      usbdio.DConfig(usbdio.DIO_PORTB, 0xff)  # Port B input
      usbdio.DOut(usbdio.DIO_PORTA, 0x0)
      while (True):
        try:
          num = int(input('Enter a byte number [0x0-0xff]: '),16)
          usbdio.DOut(usbdio.DIO_PORTA, num)
          value = usbdio.DIn(usbdio.DIO_PORTB)
          print('PortB: The number you entered =', hex(value))
          for i in range(8):
            value = usbdio.DBitIn(usbdio.DIO_PORTB, i)
            print('Port B Bit',i,' =', hex(value))
        except:
          pass
        if (toContinue() != True):
          break
    elif ch == 't':
      print('Testing Digital Bit I/O ...')
      print('Connect pins 21 through 28 <--> 32 through 39  (Port A to Port B)')
      while (True):
        usbdio.DOut(usbdio.DIO_PORTA, 0x0)  # reset the pin values
        bit = int(input('Enter a bit value for output (0 | 1): '),16)
        pin = int(input('Select a pin in port A [0-7]: '),16)
        usbdio.DBitOut(usbdio.DIO_PORTA, pin, bit)
        value = usbdio.DIn(usbdio.DIO_PORTB)
        print('The number you entered 2^',pin,'= ',hex(value))

        if (toContinue() != True):
          break
      
if __name__ == "__main__":
  main()

