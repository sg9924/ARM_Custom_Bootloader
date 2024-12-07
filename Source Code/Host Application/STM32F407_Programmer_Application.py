import serial
import struct
import os
import sys
import glob


#Flash
Flash_HAL_OK                                        = 0x00
Flash_HAL_ERROR                                     = 0x01
Flash_HAL_BUSY                                      = 0x02
Flash_HAL_TIMEOUT                                   = 0x03
Flash_HAL_INV_ADDR                                  = 0x04


#Bootloader Commands
COMMAND_BL_GET_VER                                  =0x51
COMMAND_BL_GET_HELP                                 =0x52


#Packet Length Details of Bootloader Commands
COMMAND_BL_GET_VER_LEN                              =6
COMMAND_BL_GET_HELP_LEN                             =6


verbose_mode       =1
mem_write_active   =0


#----------------------------- CRC ------------------------------------------------
#conver word to byte
def word_to_byte(addr, index , lowerfirst):
    value = (addr >> (8 * (index -1)) & 0x000000FF)
    return value

#CRc calculation
def get_crc(buff, length):
    crc = 0xFFFFFFFF

    for data in buff[0:length]:
        crc = crc ^ data
        for i in range(32):
            if(crc & 0x80000000):
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
    return crc
#----------------------------------------------------------------------------------

#----------------------------- Serial Port ----------------------------------------
#1. Lists the serial ports available on the Host system
def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


#2. Serial Port Configuration
def Serial_Port_Configuration(port):
    global ser
    try:
        ser = serial.Serial(port,115200,timeout=2)
    except:
        print("\n[ERROR]: Not a valid Port!!")
        
        port = serial_ports()
        if(not port):
            print("\n[ERROR]: No Ports Detected!!")
        else:
            print("\n[INFO]: Available Ports on the System,")
            print("\n",port)
        return -1
    if ser.is_open:
        print("\n[INFO]: Port Open Success")
    else:
        print("\n[ERROR]: Port Open Failed!!")
    return 0

#3. Read Serial Port
def read_serial_port(length):
    read_value = ser.read(length)
    return read_value

#4. Close Serial Port
def Close_serial_port():
    pass

#5. Purge Serial Port
def purge_serial_port():
    ser.reset_input_buffer()

#6. Write to Serial Port
def Write_to_serial_port(value, *length):
        data = struct.pack('>B', value)
        if (verbose_mode):
            value = bytearray(data)
            print("   "+"0x{:02x}".format(value[0]),end=' ')
        if(mem_write_active and (not verbose_mode)):
                print("#",end=' ')
        ser.write(data)
#----------------------------------------------------------------------------------