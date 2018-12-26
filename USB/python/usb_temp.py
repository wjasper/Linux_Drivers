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

import hid
import time
from struct import *

class usb_temp:

  DIO_DIR_IN      =  1
  DIO_DIR_OUT     =  0

  # Structures for Temperature */
##################################################
# NIST Thermocouple coefficients
#
# The following types are supported:
#
#    J, K, T, E, R, S, B, N
#
# Define the types of Thermocouples supported */

  CHAN_DISABLE  = 0
  TC_TYPE_J     = 1   # Type J thermocouple
  TC_TYPE_K     = 2   # Type K thermocouple
  TC_TYPE_T     = 3   # Type T thermocouple
  TC_TYPE_E     = 4   # Type E thermocouple
  TC_TYPE_R     = 5   # Type R thermocouple
  TC_TYPE_S     = 6   # Type S thermocouple
  TC_TYPE_B     = 7   # Type B thermocouple
  TC_TYPE_N     = 8   # Type N thermocouple
  
  # Channels  CH0  = 0x0  // Channel 0
  CH1  = 0x1  # Channel 1
  CH2  = 0x2  # Channel 2
  CH3  = 0x3  # Channel 3
  CH4  = 0x4  # Channel 4
  CH5  = 0x5  # Channel 5
  CH6  = 0x6  # Channel 6
  CH7  = 0x7  # Channel 7
  CJC0 = 0x80 # Cold Junction Compensator 0
  CJC1 = 0x81 # Cold Junction Compensator 1

  # Configuration Items
  ADC_0 = 0x0  # Setting for ADC 0
  ADC_1 = 0x1  # Setting for ADC 1
  ADC_2 = 0x2  # Setting for ADC 2
  ADC_3 = 0x3  # Setting for ADC 3

  # Sub Items
  SENSOR_TYPE      = 0x00   # Sensor type  Read Only
  CONNECTION_TYPE  = 0x01   # Connection type - RTD & Thermistor
  FILTER_RATE      = 0x02   # Filter update rate
  EXCITATION       = 0x03   # Currect excitation
  VREF             = 0x04   # Measured Vref value
  I_value_0        = 0x05   # Measured I value @ 10uA
  I_value_1        = 0x06   # Measured I value @ 210uA
  I_value_2        = 0x07   # Measured I value @ 10uA (3 wire connection)
  V_value_0        = 0x08   # Measured V value @ 10uA
  V_value_1        = 0x09   # Measured V value @ 210uA
  V_value_2        = 0x0a   # Measured V value @ 210uA (3 wire connection)
  CH_0_TC          = 0x10   # Thermocouple type for channel 0
  CH_1_TC          = 0x11   # Thermocouple type for channel 1
  CH_0_GAIN        = 0x12   # Channel 0 gain value
  CH_1_GAIN        = 0x13   # Channel 1 gain value
  CH_0_COEF_0      = 0x14   # Channel 0 Coefficient 0
  CH_1_COEF_0      = 0x15   # Channel 1 Coefficient 0
  CH_0_COEF_1      = 0x16   # Channle 0 Coefficient 1
  CH_1_COEF_1      = 0x17   # Channel 1 Coefficient 1
  CH_0_COEF_2      = 0x18   # Channel 0 Coefficient 2
  CH_1_COEF_2      = 0x19   # Channel 1 Coefficient 2
  CH_0_COEF_3      = 0x1a   # Channel 0 Coefficient 3
  CH_1_COEF_3      = 0x1b   # Channle 0 Coefficient 3

  # integers
  SubItemInt = (SENSOR_TYPE, CONNECTION_TYPE, FILTER_RATE, EXCITATION, CH_0_TC, CH_1_TC, CH_0_GAIN, CH_1_GAIN)
  # floats
  SubItemFloat = (VREF, I_value_0, I_value_1, I_value_2, V_value_0, V_value_1, V_value_2, \
                 CH_0_COEF_0, CH_1_COEF_0, CH_0_COEF_1, CH_1_COEF_1, CH_0_COEF_2, CH_1_COEF_2, CH_0_COEF_3, CH_1_COEF_3) 

  # Possible Values
  RTD            = 0x0
  THERMISTOR     = 0x1
  THERMOCOUPLE   = 0x2
  SEMICONDUCTOR  = 0x3
  DISABLED       = 0x4

  FREQ_500_HZ    = 0x1
  FREQ_250_HZ    = 0x2
  FREQ_125_HZ    = 0x3
  FREQ_62_5_HZ   = 0x4
  FREQ_50_HZ     = 0x5
  FREQ_39_2_HZ   = 0x6
  FREQ_33_3_HZ   = 0x7
  FREQ_19_6_HZ   = 0x8
  FREQ_16_7_HZ   = 0x9

  FREQ_16_7_HZ  = 0xa
  FREQ_12_5_HZ  = 0xb
  FREQ_10_HZ    = 0xc
  FREQ_8_33_HZ  = 0xd
  FREQ_6_25_HZ  = 0xe
  FREQ_4_17_HZ  = 0xf

  GAIN_1X       = 0x0
  GAIN_2X       = 0x1
  GAIN_4X       = 0x2
  GAIN_8X       = 0x3
  GAIN_16X      = 0x4
  GAIN_32X      = 0x5
  GAIN_64X      = 0x6
  GAIN_128X     = 0x7

  # For connection types RTD & thermistor
  TWO_WIRE_ONE_SENSOR  = 0x0
  TWO_WIRE_TWO_SENSOR  = 0x1
  THREE_WIRE           = 0x2
  FOUR_WIRE            = 0x3

  #For connection types Semiconductor
  SINGLE_ENDED         = 0x00
  DIFFERENTIAL         = 0x01  

  #Current excitation values
  EXCITATION_OFF     = 0x0
  MU_A_10            = 0x1  # 10 micro Amps
  MU_A_210           = 0x2  # 210 micro Amps

    
  # Commands and Codes for USB-TEMP-AI  HID reports
  # Digital I/O Commands
  DCONFIG          = 0x01   # Configure digital port
  DCONFIG_BIT      = 0x02   # Configure individual digital port bits
  DIN              = 0x03   # Read digital port
  DOUT             = 0x04   # Write digital port
  DBIT_IN          = 0x05   # Read digital port bit
  DBIT_OUT         = 0x06   # Write digital port bit

  # Analog Input Commands
  AIN              = 0x18   # Read analog input channel
  AIN_SCAN         = 0x11   # Read multiple input channels

  # Counter Commands
  CINIT            = 0x20   # Initialize counter
  CIN              = 0x21   # Read Counter

  # Memory Commands
  MEM_READ         = 0x30   # Read Memory
  MEM_WRITE        = 0x31   # Write Memory

  # Miscellaneous Commands
  BLINK_LED        = 0x40   # Causes LED to blink
  RESET            = 0x41   # Reset USB interface
  SET_TRIGGER      = 0x42   # Configure external trigger
  SET_SYNC         = 0x43   # Configure sync input/ouput
  GET_STATUS       = 0x44   # Retrieve device status
  SET_ITEM         = 0x49   # Set a configuration item
  GET_ITEM         = 0x4A   # Get a configuration item
  CALIBRATE        = 0x4B   # Perform a channel calibration
  BURNOUT_STATUS   = 0x4C   # Get thermocouple burnout detection status
  CAL_CONFIG       = 0x4D   # Configure Calibration Mux
  CAL_STEPS        = 0x4E   # Retrieve number of steps in calibration sequence

  # Code Update Commands
  PREPARE_DOWNLOAD = 0x50   # Prepare for program memory download
  WRITE_CODE       = 0x51   # Write program memory
  READ_CHECKSUM    = 0x52   # Return program memory checksum
  WRITE_SERIAL     = 0x53   # Write new serial number to device
  READ_CODE        = 0x55   # Read program memory

  # Alarm Commands
  CONFIG_ALARM     = 0x6b   # Configure temperature and voltage alarm
  GET_ALARM_CONFIG = 0x6c   # Read current alarm configuration

  scanIdx     = 0        # scan index
  productID   = 0        # product ID


  def __init__(self, serial=None):
    self.productID = 0x0008d           # USB-TEMP
    try:
      self.h = hid.device()
    except:
      print('Error creating hid device')

    try:
      self.h.open(0x09db, self.productID, serial)
    except:
      print('Can not open USB-TEMP')
      return

    # enable non-blocking mode
    self.h.set_nonblocking(1)

    
  #################################
  #     Digital I/O  Commands     #
  #################################

  def DConfig(self, direction):
    """
    This command set the direction of the DIO port to input or output
      direction:  0 = output,   1 = input
    """
    self.h.write([self.DCONFIG, direction])

  def DConfigBit(self, bit_num, direction):
    """
    This command sets the direction of individual DIO bits to input or output.
      bitnum:    the bit to configure (0-7)
      direction: 0 = output,   1 = input
    """
    self.h.write([self.DCONFIG_BIT, bit_num, direction])

  def DIn(self):
    """
    This command reads the current state of the digital port.  The
    return value will be the value seen at the port pins.
    """
    self.h.write([self.DIN])
    try:
      value = self.h.read(2,500)
    except:
      print('DIn: error in reading.')
    return(value[2])

  def DOut(self, value):
    """
    This command writes data to the DIO port bits that are configured as outputs.
     value:           value to write to the port
    """
    self.h.write([self.DOUT, value])

  def DBitIn(self, bit_num):
    """
    This command reads an individual digital port bit.  It will return the value
    seen at the port pin, so may be used for an input or output bit.
         bit_num:            The bit to read (0-7)
    """
    self.h.write([self.DBIT_IN, bit_num])  
    try:
      value = self.h.read(2,500)
    except:
      print('DBitIn: error in reading.')
    return(value[1])

  def DBitOut(self, bit_num, value):
    """
    This command writes an individual digital port bit.  
    
     bit_num:  The bit to read (0-7)
     value:    The value to write to the bit (0 or 1)
    """
    self.h.write([self.DBIT_OUT, bit_num, value])

  #################################
  #     Analog Input Commands     #
  #################################
  def AIn(self, channel, units):
    '''
    This command reads the value from the specified input channel.
    The return value is a 32-bit floating point value in the units
    configured for the channel.  CJC readings will always be in
    Celsius.  Channels 0-3 are temperature input channels, while
    channels 4-7 are voltage channels.

    channel: the channel to read (0-7)
    units:   the units to use for returned data
             0 - temperature, 1 - raw measurement (resistance or voltage, dependent on sensor type) 

    value:  -888.0 indicates open thermocouple on that channel
            -900.0 indicates an initialization error
            -999.0 indicates a floating point error
    '''
    self.h.write([self.AIN,channel, units])
    temperature ,= unpack_from('f', bytes(self.h.read(5,1000)), 1)
    return temperature
              
  def AInScan(self, start_chan, end_chan, units):
    '''
    This command reads multile input channels and sends the readings
      start_chan: the first channel to return (0-7)
      end_chan:   the last channel to return (0-7)
      units:      the units to use for the returned data
                  0 - temperature, 1 - raw measurement (resistance or voltage, dependent on sensor type)
    '''
    nSamples = end_chan - start_chan + 1
    self.h.write([self.AIN_SCAN, channel, units])
    data = self.h.read(nSamples+1, 1000)
    return unpack_from('f'*nSamples, data, 1)

  #################################
  #     Counter  Commands         #
  #################################

  def CInit(self):
    # This command initializes the event counter and resets the count to zero
    self.h.write([self.CINIT])

  def CIn(self):
    """
    This function reads the current value of the 32-bit event counter.
    """
    
    self.h.write([self.CIN])
    try:
      value = self.h.read(5,100)
    except:
      print('Error in CIn.')
      return 0
    return (value[1] | (value[2]<<8) | (value[3]<<16) | (value[4]<<24))

  #################################
  #     Memory  Commands          #
  #################################

  def MemRead(self, address, micro_type, count):
    """
    This command reads data from the configuration memeory (EEPROM).
    All memory may be read.

    The power-on values for adc_settings can be read from the EEPROM
    starting at address 0x000 in the isolated microcontroller.  New
    values may alo be written to the EEPROM with MemWrite, but will
    not be put into use until the device is reset.
    
       address: the start address for the read.
       micro_type:    0 = main microcontroller, 1 = isolated microcontroller
       count: the number of bytes to read (62 max for main micro, 60 max for isolated micro.)

       |-----------------------------------------------------------------------|
       | Address Range  |    Size   |                 Descripton               |
       |-----------------------------------------------------------------------|
       |  0x000 - 0x117 | 280 bytes | ADC settings                             |
       |-----------------------------------------------------------------------|
       |  0x118 - 0x167 |  80 bytes | Alarm settings                           |
       |-----------------------------------------------------------------------|
       |  0x168 - 0x17F |  24 bytes | Calibration offset values                |
       |-----------------------------------------------------------------------|
       |  0x180 - 0x197 |  24 bytes | Calibration fullscale temperature values |
       |-----------------------------------------------------------------------|
       |  0x198 - 0x217 | 128 bytes | Calibration fullscale voltage values     |
       |-----------------------------------------------------------------------|
       |  0x218 - 0x218 |   1 byte  | calibration path                         |
       |-----------------------------------------------------------------------|
       |  0x219 - 0x238 |  32 bytes | calibration reference values             |
       |-----------------------------------------------------------------------|
       |  0x239 - 0x3FF | 455 bytes | unused                                   |
       |-----------------------------------------------------------------------|
    """
    if (micro_type == 0 and count > 62):
      raise ValueError('MemRead: max count is 62 for main microcontroller')
      return
    if (micro_type == 1 and count > 60):
      raise ValueError('MemRead: max count is 60 for isolated microcontroller')
      return
    self.h.write([self.MEM_READ, address, micro_type, count])
    try:
      value = self.h.read(count+1, 100)
    except:
      print('Error in reading memory, value =', value)
    return (unpack_from('B'*count, value, 1))

  def MemWrite(self, address, micro_type, count, data):
    """ 
    This command writes data to the non-volatile EEPROM memory on the
    device.  The non-volatile memory is used to store calibration
    coefficients, system information and user data. Locations
    0x00-0xFF are available on the main microcontroller for general
    use and my be written.  The locations on the isolated
    microcontroller are used for channel configuration data (see the
    data structure in the isolated micro document.)
    
      address: the start address to write (0x00-0xFF)
      micro_type:    0 = main microcontroller, 1 = isolated microcontroller
      count: the number of bytes to read (59 max)
      data:  the data to be written (59 byts max for main micro, 59 bytes max for isolated micro.)
    """

    if (count > 59):
      raise ValueError('MemWrite: max count is 59')
      return
    self.h.write([self.MEM_WRITE, address, micro_type, count, data[0:count]], 1000)

  #################################
  #     Miscellaneous Commands    #
  #################################

  def Blink(self):
    """
    This commands causes the LED to flash several times.
    """
    self.h.write([self.BLINK_LED])

  def Reset(self):
    """
    The command causes the device to perform a soft reset. The device
    disconnects from the USB bus and resets the main and isolated
    microcontrollers.
    """
    self.h.write([self.RESET])

  def Status(self):
    ''' 
    This command returns the status of the device calibration.  If the
    response indicates a calibration error, it will return this value
    until a new calibration is started.  The total number of
    sequential steps in the calibration can be determined using the
    CalSteps command.
           
    Response:
      bits[0:5] - calibration progress
        0 = no calibration in progress or calibration complete
        Other values indicate the progress of calibration , increasing sequentially.
      bit 6 - calibration type in progress
        0 = temperature calibration,   1 = voltage calibration
      bit 7 - calibration error status
        0 = no error,   1 = error
    '''
    self.h.write([self.GET_STATUS])
    value = self.h.read(2, 100)
    return value[1]

  def SetItem(self, item, subitem, value):
    '''
    This command sets the value of a configuration item
    '''
    if subitem in self.SubItemInt:
      self.h.write([self.SET_ITEM, item, subitem, value])
    elif subitem in self.SubItemFloat:
      data  = unpack('BBBB', pack('f',value)) # convert float to 4 byte list
      self.h.write([self.SET_ITEM, item, subitem, data[0], data[1], data[2], data[3]])
    else:
      raise ValueError('SetItem: Unknown subitem')
      
  def GetItem(self, item, subitem):
    '''
    This command reads the value of a configuration item.
    '''
    self.h.write([self.GET_ITEM, item, subitem])
    if subitem in self.SubItemInt:
      value = self.h.read(2, 500)
      return value[1]
    elif subitem in self.SubItemFloat:
      value = bytes(self.h.read(5,500))
      value ,= unpack_from('f', value, 1)
      return value
    else:
      raise ValueError('GetItem: Unknown subitem')
      return

  def Calibrate(self, cal_type=0, path=0):
    '''
    This command instructs the device to perform a calibration on all
    channels.  Used after reconfiguring the channel(s).  This may take
    up to several seconds, and the completion may be determined by
    polling the status with GetStatus.  Temperature readings will not
    be updated while the calibration is ongoing, but DIO operations
    may be performed.  The device will not accept SetItem or MemWrite
    commands while calibration is being performed.  Additionally, any
    Calibrate commands with cal_type argument other than 255 (abort
    aclibration) wil be ignored.  After a calibration is aborted,
    GetStatus will indicate a calibration error until a new
    calibration is started.  Once voltage ccalibration has been
    completed successfully, the calibration path location in the
    isolated microcontrollers EEPROM will be updated to indicate which
    path was used for the most recent calibration.

    cal_type:  0 = temperature calibration
               1 = voltage calibration
               255 = abort calibration
    path:      0 = Channel Hi path (Channel Lo is referenced)
               1 = Channel Lo path (Channel Hi is referenced)

    '''
    self.h.write([self.CALIBRATE, cal_type, path])
    while (self.Status() & 0x1f):
      print('Calibration in Progress:', self.Status() & 0x1f)
      time.sleep(1.0)

  def BurnoutStatus(self, mask):
    ''' 
    This command returns the status of burnout detection for
    thermocouple channels.  The return value is a bitmap indicating
    the burnout detection status for all 8 channels.  Individual bits
    will be set if an open circuit has been detected on that channel.
    The bits will be cleared after the call using the mask that is
    passed as a parameter. If a bit is set, the corresponding bit in
    the status will be left at its current value.

    mask: the bit mask for clearing status bits after the call
    '''
    self.h.write([self.BURNOUT_STATUS])
    value = self.h.read(2, 500)
    return value[1]
  
  def PrepareDownload(self, micro):
    '''
    This command puts the device into code update mode.  The unlock code must be correct as a
    further safety device.  Call this once before sending code with WriteCode.  If not in
    code update mode, any WriteCode will be ignored.  A Reset command must be issued at
    the end of the code download in order to return the device to operation with the new code.
    
    micro:  the microcontroller receiving the update. 0 = main, 1 = isolated
    '''

    self.h.write([self.PREPARE_DOWNLOAD, 0xad, micro])

  def CalConfig(self, gain, polarity, path):
    '''
    This command configures the calibration mx for voltage channels
    4-7.  To use the calibration input, the channel must be configured
    for calibration using SetItem.

    gain:  gain setting
           [0:1]  00 = +/- 10V
                  01 = +/- 5V
                  10 = +/- 2.5V
                  11 = +/- 1.25V
    polarity:  0 = positive, 1 = negative
    path:      0 = channel high path, 1 = channel low path 
    '''
    self.h.write([self.CAL_CONFIG, gain, polarity, path])

  def CalSteps(self):
    '''
    This command returns the number of steps in the calibration
    sequence for both temperature and voltage calibration.  Both
    calibration routines start with a sequence number of 1 and this
    command returns the last number in the sequence.
    '''
    self.h.write(self.CAL_STEPS)
    value = self.h.read(3, 500)
    retrn (value[1], value[2])

  def WriteCode(self, address, count, data):
    '''
    This command writes to the program memory in the device.  This command is not accepted
    unless the device is in update mode.  This command will normally be used when downloading
    a new hex file, so it supports memory ranges that may be found in the hex file.  The
    microcontroller that is being written to is selected with the "Prepare Download" command.

    The address ranges are:

    0x000000 - 0x0075FF:  Microcontroller FLASH program memory
    0x200000 - 0x200007:  ID memory (serial number is stored here on main micro)
    0x300000 - 0x30000F:  CONFIG memory (processor configuration data)
    0xF00000 - 0xF03FFF:  EEPROM memory

    FLASH program memory: The device must receive data in 64-byte segments that begin
    on a 64-byte boundary.  The data is sent in messages containing 32 bytes.  count
    must always equal 32.

    Other memory: Any number of bytes up to the maximum (32) may be sent.

    address:  the start address for this portion of program memory
    count:    the number of bytes of data (max 32)
    data:     the program data (max 32 bytes)
    '''
    self.h.write([self.WRITE_CODE, address & 0xff, (address >> 8) & 0xff, (address >> 16) & 0xff, count, data[0:count]])

  def WrtieSerial(self, serial):
    '''
    This command sends a new serial number to the device.  The serial
    number consists of 8 bytes, typically ASCII numeric or hexadecimal
    digits (i.e. "00000001").  The new serial number will be
    programmed but not used until hardware reset.

    serial:  the 8 types of the serial number
    '''
    self.h.write([self.WRITE_SERIAL, serial[0:]])

  def ReadCode(self, address, count):
    '''
    This command reads from program memory.

      address:  the start address for the read.
      count:    the number of bytes to read (62 max)
    '''
    self.h.write([self.READ_CODE, address & 0xff, (address >> 8) & 0xff, (address >> 16) & 0xff, count])
    data = self.h.read(count+1, 1000)
    return (data[1:count])
                
    
  #################################
  #       Alarm  Commands        #
  #################################

  def ConfigureAlarm(self, number, in_options, out_options, value_1, value_2):
    '''
    This command configures a temperature alarm.  There are 8
    temperature alarms available, corresponding to the 8 available DIO
    bits.  If an alrm is enabled, its associated DIO line wil be
    configured as an output on power on and driver to their non-alarmed
    state.  The alarms are evaluated every measurement cycle.

    number:  alarm number to configure (0-7)
    in_options: bit field that controls various options for the input
	        bits 0-2:  input channel (0-7)
	        bit 3:     units (0=temperature, 1 = raw reading)
		bits 4-6:   threshold type
		    000 - alarm when reading > value_1
  		    001 - alarm when reading > value_1, reset when reading < value_2
		    010 - alarm when reading < value_1
		    011 - alarm when reading < value_1, reset when reading > value_2
		    100 - alarm when reading < value_1 or reading > value_2
		    101 - not used
		    110 - not used
		    111 - not used
		bit 7: not used, must be 0
		  
    out_options: bit field that controls various options for the output
		 bit 0:    1 - enable alarm, 0 - disable alarm
		 bit 1:    alarm level, 0 - active low alarm, 1 - active high alarm
		 bits 2-7: not used
	
    value_1: threshold value 1 for the alarm (float in Celsius)
    value_2: threshold value 2 for the alarm (float in Celsius)
    '''
    self.h.write([self.CONFIGURE_ALARM, in_options, out_options, unpack('BBBB',pack('f',value_1)),unpack('BBBB',pack('f',value_2))])

  def GetAlarmConfig(self, number):
    '''
    This command reads the current temperature alarm configuration.

    number = alarm number to retrieve (0-7)
    '''

    self.h.write([self.GET_ALARM_CONFIG, number])
    value = self.h.read(11, 1000)
    in_options = value[1]
    out_options = value[2]
    value_1 = unpack_from('f', value, 3)
    value_2 = unpack_from('f', value, 7)
    return (in_options, out_options, value_1, value_2)
  
