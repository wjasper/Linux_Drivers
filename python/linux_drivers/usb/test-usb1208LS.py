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

from usb_1208LS import *
import time
import sys

def toContinue():
  answer = input('Continue [yY]? ')
  if (answer == 'y' or answer == 'Y'):
    return True
  else:
    return False
  
def main():
  usb1208LS = usb_1208LS()

  while True:
    print("\nUSB-1208LS Testing")
    print("----------------")
    print("Hit 'b' to blink.")
    print("Hit 's' to set user id.")
    print("Hit 'g' to get user id.")
    print("Hit 'f' to get serial number.")
    print("Hit 'j' for information.")
    print("Hit 'c' to test counter. ")
    print("Hit 'd' to test digital I/O.")
    print("Hit 't' to test digital bit I/O.")
    print("Hit 'o' to test analog output.")
    print("Hit 'i' to test analog input.")
    print("Hin 'n' to test analog input scan.")
    print("Hit 'e' to exit.")

    ch = input('\n')

    if ch == 'b':
      usb1208LS.Blink()
    elif ch == 'e':
      usb1208LS.h.close()
      exit(0)
    elif ch == 'r':
      usb1208LS.Reset()
      exit(0)
    elif ch == 'c':
      usb1208LS.CInit()                       # initialize the counter
      print('Connect pin 20 and 21')
      for i in range(20):
        usb1208LS.DOut(usb1208LS.DIO_PORTA, 1);
        usb1208LS.DOut(usb1208LS.DIO_PORTA, 0);
        print('Counter =', usb1208LS.CIn())   # read the current count
    elif ch == 'f':
      print("Serial No: %s" % usb1208LS.h.get_serial_number_string())
    elif ch == 'g':
      print('User ID =', usb1208LS.GetID())
    elif ch == 's':
      id = int(input('Enter a user id (0-255): '))
      usb1208LS.SetID(id)
    elif ch == 'j':
      print("Manufacturer: %s" % usb1208LS.h.get_manufacturer_string())
      print("Product: %s" % usb1208LS.h.get_product_string())
      print("Serial No: %s" % usb1208LS.h.get_serial_number_string())
    elif ch == 'd':
      print('Testing Digital I/O ...')
      print('Connect pins 21 through 28 <--> 32 through 39  (Port A to Port B)')
      usb1208LS.DConfig(usb1208LS.DIO_PORTA, 0x0)   # Port A output
      usb1208LS.DConfig(usb1208LS.DIO_PORTB, 0xff)  # Port B input
      usb1208LS.DOut(usb1208LS.DIO_PORTA, 0x0)
      while (True):
        try:
          num = int(input('Enter a byte number [0x0-0xff]: '),16)
          usb1208LS.DOut(usb1208LS.DIO_PORTA, num)
          value = usb1208LS.DIn(usb1208LS.DIO_PORTB)
          print('PortB: The number you entered =', hex(value))
          for i in range(8):
            value = usb1208LS.DBitIn(usb1208LS.DIO_PORTB, i)
            print('Port B Bit',i,' =', hex(value))
        except:
          pass
        if (toContinue() != True):
          break
    elif ch == 't':
      print('Testing Digital Bit I/O ...')
      print('Connect pins 21 through 28 <--> 32 through 39  (Port A to Port B)')
      while (True):
        try:
          usb1208LS.DOut(usb1208LS.DIO_PORTA, 0x0)  # reset the pin values
          bit = int(input('Enter a bit value for output (0 | 1): '),16)
          pin = int(input('Select a pin in port A [0-7]: '),16)
          usb1208LS.DBitOut(usb1208LS.DIO_PORTA, pin, bit)
          value = usb1208LS.DIn(usb1208LS.DIO_PORTB)
          print('The number you entered 2^',pin,'= ',hex(value))
        except:
          pass
        if (toContinue() != True):
          break
    elif ch == 'i':
      print('Test Differential Analog Input')
      chan = int(input('Select channel [0-3]: '))
      if (chan < 0 or chan > 3):
        break
      print("\t\t1. +/- 20.V")
      print("\t\t2. +/- 10.V")
      print("\t\t3. +/- 5.V")
      print("\t\t4. +/- 4.V")
      print("\t\t5. +/- 2.5V")
      print("\t\t6. +/- 2.0V")
      print("\t\t7. +/- 1.25V")
      print("\t\t8. +/- 1.0V")
      gain = int(input("Select gain: [1-8] "))
      if (gain == 1):
        gain = usb1208LS.BP_20_00V
      elif (gain == 2):
        gain = usb1208LS.BP_10_00V
      elif (gain == 3):
        gain = usb1208LS.BP_5_00V
      elif (gain == 4):
        gain = usb1208LS.BP_4_00V
      elif (gain == 5):
        gain = usb1208LS.BP_2_50V
      elif (gain == 6):
        gain = usb1208LS.BP_2_00V
      elif (gain == 7):
        gain = usb1208LS.BP_1_25V
      elif (gain == 8):
        gain = usb1208LS.BP_1_00V
      else:
        break
      while True:
        value = usb1208LS.AIn(chan, gain)
        print('Channel:', chan,' value = ', hex(value), format(usb1208LS.volts(gain,value),'.2f'))
        time.sleep(1)
        if (toContinue() != True):
          break
    elif ch == 'n':
      print('Test Analog Input Scan')
      nQueue = 4    # depth of the queue: must be 1, 2, 4 or 8
      chanQueue = [0, 1, 2, 3, 0, 1, 2, 3]
      gain = usb1208LS.BP_10_00V
      gainQueue = [gain, gain, gain, gain, gain, gain, gain, gain]
      frequency = 150   # 150 Hz
      count = 96        # must be an even number
      options = usb1208LS.AIN_EXECUTION | usb1208LS.AIN_BURST_MODE
      value = usb1208LS.AInScan(count, frequency, nQueue, chanQueue, gainQueue, options)
      print('Total number of samples = ', len(value))
      for i in range(int(count/4)):
        print('scan ',i, end=' ')
        for j in range(4):
          print(format(usb1208LS.volts(gain,value[4*i+j]),'.2f'),end=' ')
        print()
    elif ch == 'o':
      print('Testing the analog output.')
      chan = int(input('Enter channel [0-1]: '))
      value = int(input('Enter value [0-0x3ff]: '),16)
      print('Output voltage =',format(value*5.0/1023.,'.3f'))
      usb1208LS.AOut(chan,value)

if __name__ == "__main__":
  main()
