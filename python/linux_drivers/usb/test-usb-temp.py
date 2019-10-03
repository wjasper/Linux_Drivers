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


from usb_temp import *
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

def celsius2fahr(celsius):
  return (celsius*9.0/5.0 + 32.)

def fahr2celsius(fahr):
  return (fahr - 32.)*5.0/9.0

def main():
  dev = usb_temp()   # USB-TEMP
  dev.DConfig(dev.DIO_DIR_OUT)
  dev.DOut(0x0)
  
  while True:
    print("\nTesting USB-TEMP")
    print("----------------")
    print("Hit 'b' to blink LED")
    print("Hit 'c' to calibrate");
    print("Hit 'd' to test digital I/O ")
    print("Hit 'e' to exit")
    print("Hit 'f' for burnout status");
    print("Hit 'i' for information")
    print("Hit 'R' to reset");
    print("Hit 'r' to measure temperature (RTD)");
    print("Hit 'p' read the CJC");
    print("Hit 's' to get status");
    print("Hit 'S' to measure temperature (Semiconductor)");
    print("Hit 't' to measure temperature (Thermocouple)");
    print("Hit 'T' to measure temperature (Thermistor)");    
    print("Hit 'x' to measure temperature (Thermocouple) multiple channels");
    ch = input('\n')

    if ch == 'b':
      dev.Blink()
    elif ch == 'e':
      dev.h.close()
      exit(0)
    elif ch == 'c':
      print('Calibrating')
      dev.Calibrate(0)
    elif ch == 'f':
      print('Burnout status =', hex(dev.BurnoutStatus(0xf)))
    elif ch == 'd':
      print('Testing Digital Bit I/O')
      print('Connect pins DIO0 through DIO3 <==> DIO4 through DIO7')
      dev.DConfigBit(0,dev.DIO_DIR_OUT)
      dev.DConfigBit(1,dev.DIO_DIR_OUT)
      dev.DConfigBit(2,dev.DIO_DIR_OUT)
      dev.DConfigBit(3,dev.DIO_DIR_OUT)
      dev.DConfigBit(4,dev.DIO_DIR_IN)
      dev.DConfigBit(5,dev.DIO_DIR_IN)
      dev.DConfigBit(6,dev.DIO_DIR_IN)
      dev.DConfigBit(7,dev.DIO_DIR_IN)
      while True:
        value = int(input('Enter number [0-f]: '),16)
        dev.DBitOut(0, value & 0x1)
        dev.DBitOut(1, value & 0x2)
        dev.DBitOut(2, value & 0x4)
        dev.DBitOut(3, value & 0x8)
        value = dev.DBitIn(4) | dev.DBitIn(5) << 1 | dev.DBitIn(6) << 2 | dev.DBitIn(7) << 3
        print('The value you read is',hex(value))
        if toContinue() != True:
          break
    elif ch == 's':
      print('Status = ', hex(dev.Status()))
    elif ch == 'i':
      print("Manufacturer: %s" % dev.h.get_manufacturer_string())
      print("Product: %s" % dev.h.get_product_string())
      print("Serial No: %s" % dev.h.get_serial_number_string())
    elif ch == 'p':
      print('Reading CJC')
      temperature = dev.AIn(dev.CJC0, 0)
      print('CJC 0 =', format(temperature,'.2f'),'degrees Celsius or',format(celsius2fahr(temperature),'.2f'),'degrees Fahrenheit')
      temperature = dev.AIn(dev.CJC1, 0)
      print('CJC 1 =', format(temperature,'.2f'),'degrees Celsius or',format(celsius2fahr(temperature),'.2f'),'degrees Fahrenheit')
    elif ch == 't':
      chan = int(input('Select Channel [0-7]: '))
      dev.SetItem(int(chan/2), dev.SENSOR_TYPE, dev.THERMOCOUPLE)
      dev.SetItem(int(chan/2), dev.EXCITATION, dev.EXCITATION_OFF)
      print('Conect thermocouple to channel',chan)
      t_type = input('Select Thermocouple type [JKSRBETN]: ')
      if t_type == 'J':
        t_type = dev.TC_TYPE_J
        print('Type J thermocouple selected.')
      elif t_type == 'K':
        t_type = dev.TC_TYPE_K
        print('Type K thermocouple selected.')
      if t_type == 'S':
        t_type = dev.TC_TYPE_S
        print('Type S thermocouple selected.')
      if t_type == 'R':
        t_type = dev.TC_TYPE_R
        print('Type R thermocouple selected.')
      if t_type == 'B':
        t_type = dev.TC_TYPE_B
        print('Type B thermocouple selected.')
      if t_type == 'E':
        t_type = dev.TC_TYPE_E
        print('Type E thermocouple selected.')
      if t_type == 'T':
        t_type = dev.TC_TYPE_T
        print('Type T thermocouple selected.')
      if t_type == 'N':
        t_type = dev.TC_TYPE_N
        print('Type N thermocouple selected.')
      dev.SetItem(int(chan/2), chan%2+dev.CH_0_TC, t_type)
      dev.Calibrate()
      for i in range(20):
        temperature = dev.AIn(chan,0)
        print('Channel: ',chan, format(temperature,'.2f'),'degrees Celsius or',\
               format(celsius2fahr(temperature),'.2f'),'degrees Fahrenheit')
        time.sleep(1)
    elif ch == 'S':
      print('Sampling Semiconductor TMP36')
      chan = int(input('Enter channel number [0-7]: '))
      dev.SetItem(int(chan/2), dev.SENSOR_TYPE, dev.SEMICONDUCTOR)
      print('\t\t 1. Single ended')
      print('\t\t 2. Differential.')
      connection_type = int(input('Enter connection type [1-2]: '))
      if connection_type == 1:
        dev.SetItem(int(chan/2), dev.CONNECTION_TYPE, dev.SINGLE_ENDED)
      else:
        dev.SetItem(int(chan/2), dev.CONNECTION_TYPE, dev.DIFFERENTIAL)
      offset = float(input('Enter Offset: '))
      scale = float(input('Enter slope (scale): '))
      dev.SetItem(int(chan/2), dev.EXCITATION, dev.EXCITATION_OFF)
      dev.SetItem(int(chan/2), dev.CH_0_GAIN + chan%2, 0x1)  # Set for Semiconductor
      dev.SetItem(int(chan/2), dev.CH_0_COEF_0 + chan%2, offset) # Offset
      value = dev.GetItem(int(chan/2), dev.CH_0_COEF_0 + chan%2)
      print('Offset =', format(value,'.4f'))
      dev.SetItem(int(chan/2), dev.CH_0_COEF_1 + chan%2, scale) # Scale factor
      value = dev.GetItem(int(chan/2), dev.CH_0_COEF_1 + chan%2)
      print('Scale Factor =', format(value,'.4f'))
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        temperature = dev.AIn(chan, 0)
        print('Channel',chan,format(temperature,'.2f'),'degrees Celsius or',\
              format(celsius2fahr(temperature),'.2f'),'degrees Fahrenheit.')
        time.sleep(1)
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)
    elif ch == 'r':
      print('Sampling RTD')
      chan = int(input('Enter channel number [0-7]: '))
      dev.SetItem(int(chan/2), dev.SENSOR_TYPE, dev.RTD)
      print('\t\t 1. 2 - wire with 1 sensor.')
      print('\t\t 2. 2 - wire with 2 sensor.')
      print('\t\t 2. 3 - wire.')
      print('\t\t 4. 4 - wire.')
      rtd_type = int(input('Enter RTD type [1-4]: '))
      if rtd_type == 1:
        dev.SetItem(int(chan/2), dev.CONNECTION_TYPE, dev.TWO_WIRE_ONE_SENSOR)
      elif rtd_type == 2:
        dev.SetItem(int(chan/2), dev.CONNECTION_TYPE, dev.TWO_WIRE_TWO_SENSOR)
      elif rtd_type == 3:
        dev.SetItem(int(chan/2), dev.CONNECTION_TYPE, dev.THREE_WIRE)
      elif rtd_type == 4:
        dev.SetItem(int(chan/2), dev.CONNECTION_TYPE, dev.FOUR_WIRE)
      else:
        print('Unknown value.')
        break
      R0 = 100.
      A = 0.003908
      B = -5.8019e-7
      C = -4.2735e-12
      dev.SetItem(int(chan/2), dev.EXCITATION, dev.MU_A_210)
      dev.SetItem(int(chan/2), dev.CH_0_GAIN + chan%2, 0x2)      # Set 0 - 0.5V for RTD
      dev.SetItem(int(chan/2), dev.CH_0_COEF_0 + chan%2, R0)     # R0 value
      value = dev.GetItem(int(chan/2), dev.CH_0_COEF_0 + chan%2)
      print('R0 =',format(value,'.4f'))
      dev.SetItem(int(chan/2), dev.CH_0_COEF_1 + chan%2, A)      # Callendar-Van Dusen Coefficient A
      value = dev.GetItem(int(chan/2), dev.CH_0_COEF_1 + chan%2) 
      print('A =',format(value,'.4f'))
      dev.SetItem(int(chan/2), dev.CH_0_COEF_2 + chan%2, B)      # Callendar-Van Dusen Coefficient B
      value = dev.GetItem(int(chan/2), dev.CH_0_COEF_2 + chan%2) 
      print('B =',format(value,'.4e'))
      dev.SetItem(int(chan/2), dev.CH_0_COEF_3 + chan%2, C)      # Callendar-Van Dusen Coefficient C
      value = dev.GetItem(int(chan/2), dev.CH_0_COEF_3 + chan%2) 
      print('C =',format(value,'.4e'))
      flag = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag|os.O_NONBLOCK)
      while True:
        temperature = dev.AIn(chan, 0)
        print('Channel',chan,format(temperature,'.2f'),'degrees Celsius or',\
              format(celsius2fahr(temperature),'.2f'),'degrees Fahrenheit.')
        time.sleep(1)
        c = sys.stdin.readlines()
        if (len(c) != 0):
          break
      fcntl.fcntl(sys.stdin, fcntl.F_SETFL, flag)


        
if __name__ == "__main__":
  main()
