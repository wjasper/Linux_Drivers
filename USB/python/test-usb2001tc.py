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
from usb_2001TC import *
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
    usb2001tc = usb_2001TC()
    print("got a device\n")
  except:
    print('No USB-2001TC device found.')
    return

  while True:
    print("\nUSB-2001-TC Testing")
    print("----------------")
    print("Hit 'a' to read AIn.")
    print("Hit 'b' to blink LED.")
    print("Hit 'c' to get calibration slope and offset")
    print("Hit 'C' to get calibration date")
    print("Hit 'd' to set device")
    print("Hit 'i' to get CJC and Analog Input readings")
    print("Hit 'I' to get information about the device")
    print("Hit 'F' to get the CJC reading in degree F")
    print("Hit 'K' to get the CJC reading in degree Kelvin")
    print("Hit 'G' to call get_all")
    print("Hit 'r' to get reset device")
    print("Hit 's' to get serial number")
    print("Hit 'S' to get status")
    print("Hit 't' to get the temperature")
    print("Hit 'T' to write temperature to file")
    print("Hit 'v' to get firmware version")
    print("Hit 'e' to exit.")

    ch = input('\n')

    if ch == 'b':
      count = int(input('Enter number of times to blink: '))
      usb2001tc.Blink(count)
    elif ch == 'a':
      print("AIn = ", usb2001tc.AIn())
    elif ch == 'c':
      print("Calibration data: Slope = ", usb2001tc.getSlope(), "   Offset = ", usb2001tc.getOffset())
    elif ch == 'C':
      print('MFG Calibration date: ', usb2001tc.getMFGCAL())
    elif ch == 'd':
      thermo = input("Input Thermocouple type [J,K,R,S,T,N,E,B]: ")
      usb2001tc.sendSensorType(thermo)
      thermo = usb2001tc.getSensorType()
      print("Sensor Type = ", thermo)
    elif ch == 'e':
      usb2001tc.udev.close()
      exit(0)
    elif ch == 'i':
      print("CJC = ", usb2001tc.getCJCDegC(), " degree C")
    elif ch == 'F':
      print("CJC = ", usb2001tc.getCJCDegF(), " degree F")
    elif ch == 'K':
      print("CJC = ", usb2001tc.getCJCDegKelvin(), " degree K")
    elif ch == 'G':
      usb2001tc.GetAll()
    elif ch == 'r':
      usb2001tc.Reset()
    elif ch == 's':
      print("Serial No: %s" % usb2001tc.getSerialNumber())
    elif ch == 'S':
      print("Status = %s" % usb2001tc.getStatus())
    elif ch == 'I':
      print("Manufacturer: %s" % usb2001tc.getManufacturer())
      print("Product: %s" % usb2001tc.getProduct())
      print("Serial No: %s" % usb2001tc.getSerialNumber())
    elif ch == 'v':
      print("Firmware version: %s" % usb2001tc.getFirmwareVersion())
    elif ch == 't':
      # put the board in the correct voltage range +/- 73.125mV
      usb2001tc.setVoltageRange(4)
      ch = input("Input Thermocouple type [J,K,R,S,T,N,E,B]: ")
      tc_type = ch

      for i in range(10):
        temperature = usb2001tc.tc_temperature(tc_type)
        print("Thermocouple type: ", ch, "Temperature = ", temperature, "C  ", temperature*9./5. + 32., "F ")
        time.sleep(1)

        
if __name__ == "__main__":
  main()
