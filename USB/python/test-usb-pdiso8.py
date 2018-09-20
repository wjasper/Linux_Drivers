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

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  dev = usb_pdiso8()  # USB-PDISO8

  while True :
    print("\n USB-PDISO8 Testing")
    print("----------------")
    print("Hit 'b' to blink LED")
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
    elif ch == 'd':
      print('Testing Digital I/O for USB-PDISO8')
      while True:
        port = int(input('Enter a port number: 0 - Relay Port, 1 - ISO Port: '))
        if port == 0:             # Relay Port output only
          value = int(input('Enter a byte number [0-0xff]: '),16)
          dev.DOut(port, value)
        else:                     # ISO POrt input only
          print('ISO Port =', hex(dev.DIn(port)))  
        if (toContinue() != True):
          break
    elif ch == 't':
      print('Testing Digital Bit I/O')
      while(True):
        port = int(input('Enter a port number: 0 - Relay Port, 1 - ISO Port: '))
        if port == 0:            # Relay Port output only
          pin = int(input('Enter pin number [0-7]: '))
          bit_value = int(input('Enter bit value [0|1]: '))
          dev.DBitOut(port,pin,bit_value)
        else:                    # ISO POrt input only
          pin = int(input('Enter pin number [0-7]: '))
          print('Value = ',dev.DBitIn(port,pin))
        if (toContinue() != True):
          break
    elif ch == 'n':
      print("Serial No: %s" % dev.h.get_serial_number_string())
    elif ch == 'j':
      print("Manufacturer: %s" % dev.h.get_manufacturer_string())
      print("Product: %s" % dev.h.get_product_string())
      print("Serial No: %s" % dev.h.get_serial_number_string())
      
if __name__ == "__main__":
  main()
