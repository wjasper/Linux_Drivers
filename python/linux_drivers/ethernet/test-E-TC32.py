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

from E_TC32 import *
import time

def main():
  if len(sys.argv) == 2:
    device = []
    device.append(mccEthernetDevice(ETC32_PID,sys.argv[1]))  
  else:
    # Find a E-TC32 device on the subnet
    device = mccDiscover(ETC32_PID)

  print('Number of devices found = ', len(device))
  if (len(device) > 0):
    for i in range(len(device)):
      device[i].printDeviceInfo()
  else:
    exit(0)

  # Open the device
  device[0].mccOpenDevice()

  # initalize the class
  etc32 = E_TC32(device[0])

  etc32.units = CELSIUS
  etc32.wait = 0x0

  while(True):
    print("\nE-TC32 Testing")
    print("----------------")
    print("Hit 'a' to print alarm status")
    print("Hit 'b' to blink")
    print("Hit 'C' for A/D system calibration.")
    print("Hit 'd' to test digitial IO.")
    print("Hit 'e' to exit")
    print("Hit 'j' for CJC compensation")
    print("Hit 'r' to reset the device")
    print("Hit 'n' to get networking information")
    print("Hit 's' to get Status")
    print("hit 'R' to read System Memory Map")
    print("Hit 't' for temperature.")
    print("Hit 'T' for multiple temperature readings.")
    print("Hit 'v' for version and calibration date.")

    ch = input('')
    if ch == 'b':
      count = input('Enter number of times to blink: ')
      etc32.Blink(int(count))
    elif ch == 'a':
      print('Alarm Status:',etc32.alarm_status)
      etc32.ClearAlarmStatus(0xffffffff)
      print('Alarm Configuration:', etc32.alarm_config)
      print('Alarm threshold1:', etc32.alarm_threshold1)
      print('Alarm threshold2:', etc32.alarm_threshold2)
    elif ch == 'd':
      print("\n Testing Digital I/O ...")
      value = int(input('Enter a number [0-0xf]: '),16)
      etc32.DOut(value)
      value_base = etc32.DIn()
      value_latch = etc32.DOut_R()
      print('The number you entered = ',hex(value_base), '  Latched value = ', hex(value_latch))
    elif ch == 'C':
      print('Performing A/D system offset calibration')
      etc32.ADCal()
      if(etc32.Status == 1):  # EXP detected
        (date_base, date_exp) = etc32.FactoryCalDate_R()
        print('Factory Calibration Date (base): ', date_base)
        print('Factory Calibration Date (exp): ', date_exp)
        (date_base, date_exp) = etc32.FieldCalDate_R()
        print('Field Calibration Date (base): ', date_base)
        print('Field Calibration Date (exp): ', date_exp)
      else:
        date_base = etc32.FactoryCalDate_R()
        print('Factory Calibration Date (base): ', date_base)
        date_base = etc32.FieldCalDate_R()
        print('Field Calibration Date (base): ', date_base)
      print('')
      etc32.FactoryCoef_R()
      print('Printing Factory Calibration coefficients for base')
      print('ADC 0: slope_60_base: ',format(etc32.calCoefFactory[0].slope_60_base,'.6f'))
      print('ADC 0: slope_50_base: ',format(etc32.calCoefFactory[0].slope_50_base,'.6f'))
      print('ADC 0: intercept_60_base: ',format(etc32.calCoefFactory[0].intercept_60_base,'.6f'))
      print('ADC 0: intercept_50_base: ',format(etc32.calCoefFactory[0].intercept_50_base,'.6f'))
      print('ADC 1: slope_60_base: ',format(etc32.calCoefFactory[1].slope_60_base,'.6f'))
      print('ADC 1: slope_50_base: ',format(etc32.calCoefFactory[1].slope_50_base,'.6f'))
      print('ADC 1: intercept_60_base: ',format(etc32.calCoefFactory[1].intercept_60_base,'.6f'))
      print('ADC 1: intercept_50_base: ',format(etc32.calCoefFactory[1].intercept_50_base,'.6f'))
      print('')
      etc32.FieldCoef_R()
      print('Printing Field Calibration coefficients for base')
      print('ADC 0: slope_60_base: ',format(etc32.calCoefField[0].slope_60_base,'.6f'))
      print('ADC 0: slope_50_base: ',format(etc32.calCoefField[0].slope_50_base,'.6f'))
      print('ADC 0: intercept_60_base: ',format(etc32.calCoefField[0].intercept_60_base,'.6f'))
      print('ADC 0: intercept_50_base: ',format(etc32.calCoefField[0].intercept_50_base,'.6f'))
      print('ADC 1: slope_60__base: ',format(etc32.calCoefField[1].slope_60_base,'.6f'))
      print('ADC 1: slope_50__base: ',format(etc32.calCoefField[1].slope_50_base,'.6f'))
      print('ADC 1: intercept_60_base: ',format(etc32.calCoefField[1].intercept_60_base,'.6f'))
      print('ADC 1: intercept_50_base: ',format(etc32.calCoefField[1].intercept_50_base,'.6f'))
    elif ch == 'j':
      for i in range(32):
        temperature = etc32.CJC(i)
        print('Channel =',i,'  Temperature = ',format(temperature,'.2f'),'C   ', format(temperature*9./5.+32., '.2f'),'F')
      print('\nReading Multiple CJC sensor values:')
      value = etc32.CJCMultiple(0xffffffff)
      for i in range(32):
        print('Channel =',i,'  Temperature = ',format(value[i],'.2f'),'C   ', format(value[i]*9./5.+32., '.2f'),'F')
    elif ch == 't':
      chan = int(input('Enter channel number [0-31]: '))
      ttype = input('Enter thermocouple type [J,K,R,S,T,N,E,B]: ')
      if (ttype == 'j' or ttype == 'J'):
        tc_type = TC_TYPE_J
      elif (ttype == 'k' or ttype == 'K'):
        tc_type = TC_TYPE_K
      elif (ttype == 'r' or ttype == 'R'):
        tc_type = TC_TYPE_R
      elif (ttype == 's' or ttype == 'S'):
        tc_type = TC_TYPE_S
      elif (ttype == 't' or ttype == 'T'):
        tc_type = TC_TYPE_T
      elif (ttype == 'n' or ttype == 'N'):
        tc_type = TC_TYPE_N
      elif (ttype == 'e' or ttype == 'E'):
        tc_type = TC_TYPE_E
      elif (ttype == 'b' or ttype == 'B'):
        tc_type = TC_TYPE_B
      else:
        print('Unknown thermocouple type.')
        break
      etc32.TinConfig_R()
      for i in range(64):
        etc32.config_values[i] = CHAN_DISABLE
      etc32.config_values[chan] = tc_type
      etc32.TinConfig_W()
      for i in range(20):
        temperature = etc32.Tin(chan, CELSIUS, 1)
        print('Channel =',chan,'  Temperature = ',format(temperature,'.2f'),'C   ',format(temperature*9./5.+32.,'.2f'),'F')
        time.sleep(1)
    elif ch == 'e':
      etc32.device.sock.close()
      exit(0)
    elif ch == 'r':  
      etc32.Reset()
    elif ch == 's':
      # Print the calibration data and status
      if (etc32.status == 0):
        print('No EXP detected.')
      else:
        print('EXP detected.')
      etc32.TinStatus()
      print('TinStatus = ',hex(etc32.Tin_status[0]))
      etc32.OTDStatus()
      print('OTDStatus = ',hex(etc32.OTD_status[0]))
    elif ch == 'T':
      print('Read Multiple Thermocouple channels')
      ttype = input('Enter thermocouple type [J,K,R,S,T,N,E,B]: ')
      if (ttype == 'j' or ttype == 'J'):
        tc_type = TC_TYPE_J
      elif (ttype == 'k' or ttype == 'K'):
        tc_type = TC_TYPE_K
      elif (ttype == 'r' or ttype == 'R'):
        tc_type = TC_TYPE_R
      elif (ttype == 's' or ttype == 'S'):
        tc_type = TC_TYPE_S
      elif (ttype == 't' or ttype == 'T'):
        tc_type = TC_TYPE_T
      elif (ttype == 'n' or ttype == 'N'):
        tc_type = TC_TYPE_N
      elif (ttype == 'e' or ttype == 'E'):
        tc_type = TC_TYPE_E
      elif (ttype == 'b' or ttype == 'B'):
        tc_type = TC_TYPE_B
      else:
        print('Unknown thermocouple type.')
        break
      for i in range(31):
        etc32.config_values[i] = tc_type
      etc32.TinConfig_W()
      # just read the even channels
      etc32.channel_mask[0] = 0x55555555
      wait = 1
      units = CELSIUS
      value = etc32.TinMultiple(wait, units, 0x55555555)
      for i in range(16):
        print('Channel ',2*i,'  Temperature = ', format(value[i],'.2f'))
      # turn off the LED on the fron pannel
      etc32.config_measure[0] = 1   # disable the OTC which turn off the LED
      etc32.MeasureConfig_W()
      etc32.config_measure[0] = 0   # enable the OTD
      etc32.MeasureConfig_W()
    elif ch == 'n':
      value = etc32.NetworkConfig()
      print('Network configuration values: ')
      print('  IP address = ', value[0])
      print('  subnet mask = ', value[1])
      print('  gateway = ', value[2])
      print('  MAC:', \
            hex(etc32.device.MAC>>40&0xff)[2:].zfill(2)+':'+\
            hex(etc32.device.MAC>>32&0xff)[2:].zfill(2)+':'+\
            hex(etc32.device.MAC>>24&0xff)[2:].zfill(2)+':'+\
            hex(etc32.device.MAC>>16&0xff)[2:].zfill(2)+':'+\
            hex(etc32.device.MAC>>8&0xff)[2:].zfill(2)+':'+\
          hex(etc32.device.MAC&0xff)[2:].zfill(2))
    elif ch == 'v':
      etc32.Version()
      print('Communications micro firmware version:', str(etc32.version[0]>>8&0xff)+'.'+str(etc32.version[0]&0xff))
      print('Communications micro bootloader firmware version:', str(etc32.version[1]>>8&0xff)+'.'+str(etc32.version[1]&0xff))
      print('Base measurement micro firmware version:', str(etc32.version[2]>>8&0xff)+'.'+str(etc32.version[2]&0xff))
      print('Base measurement micro bootloader firmware version:', str(etc32.version[3]>>8&0xff)+'.'+str(etc32.version[3]&0xff))
      print('EXP measurement micro firmware version:', str(etc32.version[4]>>8&0xff)+'.'+str(etc32.version[4]&0xff))
      print('EXP measurement micro bootloader firmware version:', str(etc32.version[5]>>8&0xff)+'.'+str(etc32.version[5]&0xff))
    elif ch == 'R':
      print('Reading Settings Memory.')
      for i in range(0x1f):
        value = etc32.SettingsMemory_R(i, 1)
        print('address: ',hex(i), '    value =', value)
    else:
      pass

if __name__ == "__main__":
  main()
