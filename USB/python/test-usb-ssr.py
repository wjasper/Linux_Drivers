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

from usb_ssr import *
import time
import sys

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False

def main():
  dev = usb_ssr24()   # USB-SSR24
# dev = usb_ssr08()   # USB-SSR08
# dev = usb_erb24()   # USB-ERB24
# dev = usb_erb08()   # USB-ERB08
# dev = usb_pdiso8()  # USB-PDISO8

  while True :
    print("\nUSB-SSR & USB-ERB & USB-PDISO8 Testing")
    print("----------------")
    print("Hit 'b' to blink LED")
    print("Hit 'A' to read all inputs (SSR only)")
    print("Hit 's' to get status")
    print("Hit 'j' for information")
    print("Hit 'n' to get serial number")
    print("Hit 'd' to test digital I/O ")
    print("Hit 'r' to reset the device.")
    print("Hit 't' to test digital bit I/O")
    print("Hit 'T' to read temperature (ERB24 only)")
    print("Hit 'e' to exit")

    ch = input('\n')

    if ch == 'b':
      dev.Blink()
    elif ch == 'e':
      dev.h.close()
      exit(0)
    elif ch == 'A':
      print('Get all values, USB-SSR only')
      value = dev.GetAll()
      print('Input values on Port A:',hex(value[0]))
      print('Input values on Port B:',hex(value[1]))
      print('Input values on Port C low:',hex(value[2]))
      print('Input values on Port C high:',hex(value[3]))
    elif ch == 'd':
      if dev.productID == 0x8c:   # MCC USB-PDISO8
        print('Testing Digital I/O for USB-PDISO8')
        port = int(input('Enter a port number: 0 - Relay Port, 1 - ISO Port: '))
        if port == 0:             #Relay Port output only
          value = int(input('Enter a byte number [0-0xff]: '),16)
          dev.DOut(port, value)
        else:                     #ISO POrt input only
          print('ISO Port =', hex(dev.DIn()))  
          break
      elif dev.productID == 0x85 or dev.productID == 0x86:  # USB-SSR24 or UBB-SSR08
        print('Testing Digital I/O for USB-SSR24/08')
        while True:
          if (dev.productID == 0x0085): # USB SSR24
            port = int(input('Enter a port number [0-3]'))
            if port > 3:
              break
          else:                            # USB SSR08
            port = int(input('Enter a port number [2-3]')) 
            if port != 2 and port != 3:
              continue
          status = dev.Status()
          if port == 0:                    # Port A
            if status & 0x1<<0:  
              print('    Port A direction = input')
              value = dev.DIn(port)
              print('    Port A =',hex(value))
            else:
              print('    Port A direction = output')
              value = int(input('Enter a byte number [0-0xff]: '),16)
              dev.DOut(port, value)
            if status & 0x1<<4:
              print('    Port A polarity = normal')
            else:
              print('    Port A polarity = inverted')
            if status & 0x1<<8:
              print('    Port A = pull up')
            else:
              print('    Port A = pull down')
          elif port == 1:                  # Port B
            if status & 0x1<<1:  
              print('    Port B direction = input')
              value = dev.DIn(port)
              print('    Port B =',hex(value))
            else:
              print('    Port B direction = output')
              value = int(input('Enter a byte number [0-0xff]: '),16)
              dev.DOut(port, value)
            if status & 0x1<<5:
              print('    Port B polarity = normal')
            else:
              print('    Port B polarity = inverted')
            if status & 0x1<<9:
              print('    Port B = pull up')
            else:
              print('    Port B = pull down')
          elif port == 2:                  # Port C Low
            if status & 0x1<<2:  
              print('    Port C Low direction = input')
              value = dev.DIn(port)
              print('    Port C Low =',hex(value))
            else:
              print('    Port C Low direction = output')
              value = int(input('Enter a byte number [0-0xff]: '),16)
              dev.DOut(port, value)
            if status & 0x1<<6:
              print('    Port C Low polarity = normal')
            else:
              print('    Port C Low polarity = inverted')
            if status & 0x1<<10:
              print('    Port C Low = pull up')
            else:
              print('    Port C Low = pull down')
          elif port == 3:                  # Port C High
            if status & 0x1<<3:  
              print('    Port C High direction = input')
              value = dev.DIn(port)
              print('    Port C High =',hex(value))
            else:
              print('    Port C High direction = output')
              value = int(input('Enter a byte number [0-0xff]: '),16)
              dev.DOut(port, value)
            if status & 0x1<<7:
              print('    Port C High polarity = normal')
            else:
              print('    Port C High polarity = inverted')
            if status & 0x1<<11:
              print('    Port C High = pull up')
            else:
              print('    Port C High = pull down')
          if (toContinue() != True):
            break
    elif ch == 't':
      print('Testing Digital Bit I/O')
      while(True):
        if dev.productID == 0x0085:  # USB SSR24
          port = int(input('Enter a port number [0-3]: '))
          if port > 3:
            break
        else:
          port = int(input('Enter a port number [2-3]: '))
          if port != 2 and port != 3:
            pass
        print('Select the Pin in port',port,' [0-7] : ',end=' ')
        pin = int(input(''))
        status = dev.Status()
        # Port A
        if port == 0:
          if status & 0x1<<0:
            print('    Port A direction = input')
            bit_value = usbDBitIn(port, pin)
            print('    Port',port,' Pin',pin,' bit value', bit_value)
          else:
            print('    Port A direction = output')
            bit_value = int(input('Enter a bit value for output (0|1): '))
            dev.DBitOut(port, pin, bit_value)
          if status & 0x1<<4:
            print('    Port A polarity = normal')
          else:
            print('    Port A polarity = inverted')
          if status & 0x1<<8:
            print('    Port A = pull up')
          else:
            print('    Port A = pull down')
          break
        # Port B
        elif port == 1:
          if status & 0x1<<1:
            print('    Port B direction = input')
            bit_value = usbDBitIn(port, pin)
            print('    Port',port,' Pin',pin,' bit value', bit_value)
          else:
            print('    Port B direction = output')
            bit_value = int(input('Enter a bit value for output (0|1): '))
            dev.DBitOut(port, pin, bit_value)
          if status & 0x1<<5:
            print('    Port B polarity = normal')
          else:
            print('    Port B polarity = inverted')
          if status & 0x1<<9:
            print('    Port B = pull up')
          else:
            print('    Port B = pull down')
          break
        # Port C Low
        elif port == 2:
          if status & 0x1<<2:
            print('    Port C Low direction = input')
            bit_value = usbDBitIn(port, pin)
            print('    Port',port,' Pin',pin,' bit value', bit_value)
          else:
            print('    Port C Low direction = output')
            bit_value = int(input('Enter a bit value for output (0|1): '))
            dev.DBitOut(port, pin, bit_value)
          if status & 0x1<<6:
            print('    Port C Low polarity = normal')
          else:
            print('    Port C Low polarity = inverted')
          if status & 0x1<<10:
            print('    Port C Low = pull up')
          else:
            print('    Port C Low = pull down')
          break
        # Port C High
        elif port == 3:
          if status & 0x1<<3:
            print('    Port C High direction = input')
            bit_value = usbDBitIn(port, pin)
            print('    Port',port,' Pin',pin,' bit value', bit_value)
          else:
            print('    Port C High direction = output')
            bit_value = int(input('Enter a bit value for output (0|1): '))
            dev.DBitOut(port, pin, bit_value)
          if status & 0x1<<7:
            print('    Port C High polarity = normal')
          else:
            print('    Port C High  polarity = inverted')
          if status & 0x1<<11:
            print('    Port C High = pull up')
          else:
            print('    Port C High = pull down')
          break
        if (toContinue() != True):
          break;
    elif ch == 'n':
      print("Serial No: %s" % dev.h.get_serial_number_string())
    elif ch == 'j':
      print("Manufacturer: %s" % dev.h.get_manufacturer_string())
      print("Product: %s" % dev.h.get_product_string())
      print("Serial No: %s" % dev.h.get_serial_number_string())
    elif ch == 's':
      status = dev.Status()
      print('Status =',hex(status))
      if dev.productID == 0x0085:    # USB SSR24
        if status & 0x1<<0:
          print('    Port A direction = input')
        else:
          print('    Port A direction = output')
        if status & 0x1<<4:
          print('    Port A polarity = normal')
        else:
          print('    Port A polarity = inverted')
        if status & 0x1<<8:
          print('    Port A = pull up')
        else:
          print('    Port A = pull down')
        # Port B
        if status & 0x1<<1:
          print('    Port B direction = input')
        else:
          print('    Port B direction = output')
        if status & 0x1<<5:
          print('    Port B polarity = normal')
        else:
          print('    Port B polarity = inverted')
        if status & 0x1<<9:
          print('    Port B = pull up')
        else:
          print('    Port B = pull down')
      # Port C Low
      if status & 0x1<<2:
        print('    Port C Low direction = input')
      else:
        print('    Port C Low direction = output')
      if status & 0x1<<6:
        print('    Port C Low polarity = normal')
      else:
        print('    Port C Low polarity = inverted')
      if status & 0x1<<10:
        print('    Port C Low = pull up')
      else:
        print('    Port C Low = pull down')
      # Port C Hi
      if status & 0x1<<3:
        print('    Port C High direction = input')
      else:
        print('    Port C High direction = output')
      if status & 0x1<<7:
        print('    Port C High polarity = normal')
      else:
        print('    Port C High polarity = inverted')
      if status & 0x1<<11:
        print('    Port C High = pull up')
      else:
        print('    Port C High = pull down')
    elif ch == 'T':
      print('Temperature = ',dev.GetTemp())
      
if __name__ == "__main__":
  main()
