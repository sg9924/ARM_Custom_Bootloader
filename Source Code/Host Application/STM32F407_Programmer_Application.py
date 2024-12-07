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