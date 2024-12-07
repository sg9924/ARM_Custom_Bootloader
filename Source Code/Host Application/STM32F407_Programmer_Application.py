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


#----------------------------- Command Processing ---------------------------------
#1. Get Bootloader Version
def process_COMMAND_BL_GET_VER(length):
    ver=read_serial_port(1)
    value = bytearray(ver)
    print("\n[BL_CMD]: Bootloader Version -> ",hex(value[0]))

#2. Get Help
def process_COMMAND_BL_GET_HELP(length):
    value = read_serial_port(length) 
    reply = bytearray(value)
    print("\n[BL_CMD]: Supported Commands are,",end=' ')
    for x in reply:
        print(hex(x),end=' ')
    print()
#----------------------------------------------------------------------------------


#--------------------------- Process Bootloader Reply -----------------------------
def read_bootloader_reply(command_code):
    len_to_follow = 0 
    ret = -2 

    ack = read_serial_port(2)

    if(len(ack)):
        a_array=bytearray(ack)
        #if ACK
        if (a_array[0] == 0xA5):

            len_to_follow=a_array[1]
            print("\n[INFO]: CRC was successfull, Length -> ",len_to_follow)

            if (command_code) == COMMAND_BL_GET_VER:
                process_COMMAND_BL_GET_VER(len_to_follow)
                
            elif(command_code) == COMMAND_BL_GET_HELP:
                process_COMMAND_BL_GET_HELP(len_to_follow)
            else:
                print("\n[ERROR]: Invalid Command Code!!\n")
                
            ret = 0
        #if NACK
        elif a_array[0] == 0x7F:
            print("\n[ERROR]: CRC Failure\n")
            ret= -1
    else:
        print("\n[ERROR]: Timeout, Bootloader not responding!!")
    return ret
#----------------------------------------------------------------------------------


#----------------------------- Decode Menu Command --------------------------------
def decode_menu_command_code(command):
    ret_value = 0
    data_buf = []
    for i in range(255):
        data_buf.append(0)
    
    if(command  == 0 ):
        print("\n[ERROR]: Exiting!!")
        raise SystemExit
    
    elif(command == 1):
        #Packet format --> |_{length to follow}[1 byte]_|_{CMD Code}[1 byte]_|_{Host CRC Value}[4 bytes]_|

        print("\n[INFO]: Command -> BL_GET_VER")

        data_buf[0] = COMMAND_BL_GET_VER_LEN-1 #length to follow
        data_buf[1] = COMMAND_BL_GET_VER       #bootloader command code

        crc32 = get_crc(data_buf,COMMAND_BL_GET_VER_LEN-4)
        crc32 = crc32 & 0xffffffff

        #CRC value
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        #send length to follow
        Write_to_serial_port(data_buf[0],1)

        #send rest of the packet (command code & CRC)
        for i in data_buf[1:COMMAND_BL_GET_VER_LEN]:
            Write_to_serial_port(i,COMMAND_BL_GET_VER_LEN-1)
        
        #get bootloader reply
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 2):
        #Packet format --> |_{length to follow}[1 byte]_|_{CMD Code}[1 byte]_|_{Host CRC Value}[4 bytes]_|

        print("\n[INFO]: Command -> BL_GET_HELP")

        data_buf[0] = COMMAND_BL_GET_HELP_LEN-1  #length to follow
        data_buf[1] = COMMAND_BL_GET_HELP        #bootloader command code

        crc32 = get_crc(data_buf,COMMAND_BL_GET_HELP_LEN-4)
        crc32 = crc32 & 0xffffffff

        #CRC value
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        #send length to follow
        Write_to_serial_port(data_buf[0],1)

        #send rest of the packet (command code & CRC)
        for i in data_buf[1:COMMAND_BL_GET_HELP_LEN]:
            Write_to_serial_port(i,COMMAND_BL_GET_HELP_LEN-1)
        
        #get bootloader reply
        ret_value = read_bootloader_reply(data_buf[1])

    else:
        print("\n[ERROR]: Please Enter valid command code\n")
        return

    if ret_value == -2 :
        print("\n[ERROR]: TimeOut, No Response from Bootloader")
        return
#----------------------------------------------------------------------------------