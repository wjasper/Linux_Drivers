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

from E_TC import *
import time
import sys

def main():
  if len(sys.argv) == 2:
    device = []
    device.append(mccEthernetDevice(ETC_PID,sys.argv[1]))  
  else:
    # Find a E-TC device on the subnet
    device = mccDiscover(ETC_PID)

  print('Number of devices found = ', len(device))
  if (len(device) > 0):
    for i in range(len(device)):
      device[i].printDeviceInfo()
  else:
    exit(0)

  # Open the device
  device[0].mccOpenDevice()
  # initalize the class
  etc = E_TC(device[0])

  etc.units = CELSIUS
  etc.wait = 0x0

  while True:
    print("\nE-TC Testing")
    print("----------------")
    print("Hit 'b' to blink.")
    print("Hit 'c' to test counter.");
    print("Hit 'C' for A/D system calibration.");
    print("Hit 'd' to test digitial IO.");
    print("Hit 'e' to exit");
    print("Hit 'j' for CJC compensation.");
    print("Hit 'r' to reset the device.");
    print("Hit 'n' to get networking information.");
    print("Hit 's' for thermocouple status");
    print("Hit 'R' to read System Memory Map");
    print("Hit 't' for temperature.");
    print("Hit 'v' for version and calibration date.");
    
    ch = input('')
    if ch == 'b':
      count = input('Enter number of times to blink: ')
      etc.Blink(int(count))
    elif ch == 'c':
      print('Testing Counter on E-TC')
      print('Connect DI00 to CTR')
      etc.DConfig_W(0xf0)
      etc.ResetCounter()
      for i in range(20):
        etc.DOut(0x1)
        time.sleep(0.1)
        etc.DOut(0x0)
        time.sleep(0.1)
        count = etc.Counter()
        print('Counter = ', count)
    elif ch == 'd':
      print("\n Testing Digital I/O ...")
      print("connect pins DIO[0-3] <---> DIO[4-7]")
      etc.DConfig_W(0xf0)  # write to pins 0-3
      value = etc.DConfig_R()
      print('Digital Port Tristate Register = ',hex(value))
      value = int(input('Enter a number [0-0xf]: '),16)
      etc.DOut(value)
      value1 = etc.DIn()
      value1  = (value1>>4) & 0xf
      value2 = etc.DOut_R()
      print('The number you entered = ',hex(value1), '  Latched value = ', hex(value2))
    elif ch == 'e':
      etc.device.sock.close()
      exit(0)
    elif ch == 'C':
      print('Performing A/D system offset calibration')
      etc.ADCal()
    elif ch == 'r':  
      etc.Reset()
    elif ch == 'R':
      print('Reading Settings Memory.')
      for i in range(0x1f):
        value = etc.SettingsMemory_R(i, 1)
        print('address: ',hex(i), '    value =', value)
    elif ch == 'j':
      etc.CJC()
      print('Sensor 1: CJC Temperature = ',format(etc.CJC_value[0],'.2f'),'C  ',format(etc.CJC_value[0]*9./5.+32,'.2f'),'F')
      print('Sensor 2: CJC Temperature = ',format(etc.CJC_value[1],'.2f'),'C  ',format(etc.CJC_value[1]*9./5.+32,'.2f'),'F')
    elif ch == 's':
      etc.TinStatus()
      print('Tin status byte =', hex(etc.Tin_status))
      etc.OTDStatus()
      print('Open Thermocouple Detection status byte =', hex(etc.OTD_status))
      etc.AlarmStatus_R()
      print('Alarm Status byte =', hex(etc.alarm_status))
      etc.MeasureConfig_R()
      print('Measurement Configuration byte =', hex(etc.config_measure))
      etc.MeasureMode_R()
      print('Measurement Mode byte =', hex(etc.mode_measure))
      etc.CJCOffset_R()
      for i in range(8):
        print('CJC offsets: channel[',i,'] =',etc.CJC_offset[i])
      print('')
      etc.AlarmConfig_R()
      for i in range(8):
        print('Channel',i,':  alarm config =',etc.alarm_config[i],'  threshold1 =',etc.alarm_threshold1[i],\
              '  threshold2 =',etc.alarm_threshold2[i])
      # Print the calibration data and status
      status = etc.Status()
      print('status =', hex(status))
    elif ch == 'n':
      value = etc.NetworkConfig()
      print('Network configuration values: ')
      print('  IP address = ', value[0])
      print('  subnet mask = ', value[1])
      print('  gateway = ', value[2])
      print('  MAC:', \
            hex(etc.device.MAC>>40&0xff)[2:].zfill(2)+':'+\
            hex(etc.device.MAC>>32&0xff)[2:]+':'+\
            hex(etc.device.MAC>>24&0xff)[2:]+':'+\
            hex(etc.device.MAC>>16&0xff)[2:]+':'+\
            hex(etc.device.MAC>>8&0xff)[2:]+':'+\
            hex(etc.device.MAC&0xff)[2:])
    elif ch == 't':
      nchan = int(input('Enter number of channels [1-8]: '))
      thermo = input('Input Thermocouple type [J,K,R,S,T,N,E,B]: ')
      if (thermo == 'J' or thermo == 'j'):
        tc_type = TC_TYPE_J
      elif (thermo == 'K' or thermo == 'k'):
        tc_type = TC_TYPE_K
      elif (thermo == 'R' or thermo == 'r'):
        tc_type = TC_TYPE_R
      elif (thermo == 'S' or thermo == 's'):
        tc_type = TC_TYPE_S
      elif (thermo == 'T' or thermo == 't'):
        tc_type = TC_TYPE_T
      elif (thermo == 'N' or thermo == 'n'):
        tc_type = TC_TYPE_N
      elif (thermo == 'E' or thermo == 'e'):
        tc_type = TC_TYPE_E
      elif (thermo == 'B' or thermo == 'b'):
        tc_type = TC_TYPE_B
      else:
        print('Unknown thermocouple type.')
        break
      for i in range(8):
        etc.config_values[i] = CHAN_DISABLE
        channel = 0
      for i in range(nchan):
        etc.config_values[i] = tc_type
        channel |= (0x1 << i)    # set bit to corresponding channel
      etc.TinConfig_W()
      etc.TinConfig_R()
      temperature = etc.Tin(channel, CELSIUS, 1)
      for i in range(nchan):
        print('Channel = ',i,'  Temperature = ',temperature[i],'C  ',temperature[i]*9./5. + 32.,'F')
    elif ch == 'v':
      mdate = etc.FactoryCalDate_R()
      print('Factory Calibration date: ', mdate)
      mdate = etc.FieldCalDate_R()
      print('Field Calibration date: ', mdate)
    else:
      pass
    
if __name__ == "__main__":
  main()
