import serial
import struct
import os
import sys
import glob


#Flash
Flash_HAL_OK                                        =0x00
Flash_HAL_ERROR                                     =0x01
Flash_HAL_BUSY                                      =0x02
Flash_HAL_TIMEOUT                                   =0x03
Flash_HAL_INV_ADDR                                  =0x04


#Bootloader Commands
COMMAND_BL_GET_VER                                  =0x51
COMMAND_BL_GET_HELP                                 =0x52
COMMAND_BL_GET_CID                                  =0x53
COMMAND_BL_GET_RDP_STATUS                           =0x54
COMMAND_BL_GO_TO_ADDR                               =0x55
COMMAND_BL_FLASH_ERASE                              =0x56
COMMAND_BL_MEM_WRITE                                =0x57
COMMAND_BL_EN_R_W_PROTECT                           =0x58
COMMAND_BL_READ_SECTOR_P_STATUS                     =0x5A
COMMAND_BL_DIS_R_W_PROTECT                          =0x5C


#Packet Length Details of Bootloader Commands
COMMAND_BL_GET_VER_LEN                              =6
COMMAND_BL_GET_HELP_LEN                             =6
COMMAND_BL_GET_CID_LEN                              =6
COMMAND_BL_GET_RDP_STATUS_LEN                       =6
COMMAND_BL_GO_TO_ADDR_LEN                           =10
COMMAND_BL_FLASH_ERASE_LEN                          =8
COMMAND_BL_MEM_WRITE_LEN                            =11
COMMAND_BL_EN_R_W_PROTECT_LEN                       =8
COMMAND_BL_READ_SECTOR_P_STATUS_LEN                 =6
COMMAND_BL_DIS_R_W_PROTECT_LEN                      =6


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


#----------------------------- File Operations ----------------------------------------
#calculating file size
def calc_file_len():
    size = os.path.getsize("user_app.bin")
    return size

#opening the binary file
def open_the_file():
    global bin_file
    bin_file = open('user_app.bin','rb')

#reading the binary file
def read_the_file():
    pass

#closing the binary file
def close_the_file():
    bin_file.close()
#------------------------------------------------------------------------------------


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

#3. Get CID
def process_COMMAND_BL_GET_CID(length):
    value = read_serial_port(length)
    ci = (value[1] << 8)+ value[0]
    print("\n[BL_CMD]: Chip ID ->  ",hex(ci))

#4. Get Flash RDP Status
def process_COMMAND_BL_GET_RDP_STATUS(length):
    value = read_serial_port(length)
    rdp = bytearray(value)
    print("\n[BL_CMD]: RDP Status -> ",hex(rdp[0]))

#5. Got to Address
def process_COMMAND_BL_GO_TO_ADDR(length):
    addr_status=0
    value = read_serial_port(length)
    addr_status = bytearray(value)
    print("\n[BL_CMD]: Address Status -> ",hex(addr_status[0]))

#6. Flash Erase
def process_COMMAND_BL_FLASH_ERASE(length):
    erase_status=0
    value = read_serial_port(length)
    if len(value):
        erase_status = bytearray(value)
        if(erase_status[0] == Flash_HAL_OK):
            print("\n[BL_CMD]: Erase Status -> Success Code -> FLASH_HAL_OK")
        elif(erase_status[0] == Flash_HAL_ERROR):
            print("\n[BL_CMD]: Erase Status -> Fail Code -> FLASH_HAL_ERROR")
        elif(erase_status[0] == Flash_HAL_BUSY):
            print("\n[BL_CMD]: Erase Status -> Fail Code -> FLASH_HAL_BUSY")
        elif(erase_status[0] == Flash_HAL_TIMEOUT):
            print("\n[BL_CMD]: Erase Status -> Fail Code -> FLASH_HAL_TIMEOUT")
        elif(erase_status[0] == Flash_HAL_INV_ADDR):
            print("\n[BL_CMD]: Erase Status -> Fail Code -> FLASH_HAL_INV_SECTOR")
        else:
            print("\n[BL_CMD]: Erase Status -> Fail Code -> UNKNOWN_ERROR_CODE")
    else:
        print("Timeout: Bootloader is not responding")
    
#7. Memory Write
def process_COMMAND_BL_MEM_WRITE(length):
    write_status=0
    value = read_serial_port(length)
    write_status = bytearray(value)
    if(write_status[0] == Flash_HAL_OK):
        print("\n[BL_CMD]: Write Status -> FLASH_HAL_OK")
    elif(write_status[0] == Flash_HAL_ERROR):
        print("\n[BL_CMD]: Write Status -> FLASH_HAL_ERROR")
    elif(write_status[0] == Flash_HAL_BUSY):
        print("\n[BL_CMD]: Write Status -> FLASH_HAL_BUSY")
    elif(write_status[0] == Flash_HAL_TIMEOUT):
        print("\n[BL_CMD]: Write Status -> FLASH_HAL_TIMEOUT")
    elif(write_status[0] == Flash_HAL_INV_ADDR):
        print("\n[BL_CMD]: Write Status -> FLASH_HAL_INV_ADDR")
    else:
        print("\n [BL_CMD]: Write Status -> UNKNOWN_ERROR")
    print("\n")

#8. Protection Type
protection_mode= ["Write Protection", "Read/Write Protection","No protection"]
def protection_type(status,n):
    if(status & (1 << 15)):
        #PCROP is active
        if(status & (1 << n)):
            return protection_mode[1]
        else:
            return protection_mode[2]
    else:
        if(status & (1 << n)):
            return protection_mode[2]
        else:
            return protection_mode[0]

#9. Read Flash Sector Status
def process_COMMAND_BL_READ_SECTOR_STATUS(length):
    s_status=0

    value = read_serial_port(length)
    s_status = bytearray(value)
    #s_status.flash_sector_status = (uint16_t)(status[1] << 8 | status[0])
    print("\n[BL_CMD]: Sector Status -> ",s_status[0])
    if(s_status[0] & (1 << 15)):
        #PCROP is active
        print("\n[BL_CMD]: Flash protection mode -> Read/Write Protection(PCROP)\n")
    else:
        print("\n[BL_CMD]: Flash protection mode -> Write Protection\n")

    for x in range(8):
        print("\n[BL_CMD]: Sector{0}{1}".format(x,protection_type(s_status[0],x)))

#10. Disable Read Write Protect
def process_COMMAND_BL_DIS_R_W_PROTECT(length):
    status=0
    value = read_serial_port(length)
    status = bytearray(value)
    if(status[0]):
        print("\n[BL_CMD]: FAIL")
    else:
        print("\n[BL_CMD]: SUCCESS")

#11. Enable Read Write Protect
def process_COMMAND_BL_EN_R_W_PROTECT(length):
    status=0
    value = read_serial_port(length)
    status = bytearray(value)
    if(status[0]):
        print("\n[BL_CMD]: FAIL")
    else:
        print("\n[BL_CMD]: SUCCESS")
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
            elif(command_code) == COMMAND_BL_GET_CID:
                process_COMMAND_BL_GET_CID(len_to_follow)
            elif(command_code) == COMMAND_BL_GET_RDP_STATUS:
                process_COMMAND_BL_GET_RDP_STATUS(len_to_follow)
            elif(command_code) == COMMAND_BL_GO_TO_ADDR:
                process_COMMAND_BL_GO_TO_ADDR(len_to_follow)  
            elif(command_code) == COMMAND_BL_FLASH_ERASE:
                process_COMMAND_BL_FLASH_ERASE(len_to_follow)  
            elif(command_code) == COMMAND_BL_MEM_WRITE:
                process_COMMAND_BL_MEM_WRITE(len_to_follow)  
            elif(command_code) == COMMAND_BL_READ_SECTOR_P_STATUS:
                process_COMMAND_BL_READ_SECTOR_STATUS(len_to_follow) 
            elif(command_code) == COMMAND_BL_EN_R_W_PROTECT:
                process_COMMAND_BL_EN_R_W_PROTECT(len_to_follow) 
            elif(command_code) == COMMAND_BL_DIS_R_W_PROTECT:
                process_COMMAND_BL_DIS_R_W_PROTECT(len_to_follow)
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
    
    if(command == 0):
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
    
    elif(command == 3):
        print("\n[INFO]: Command -> BL_GET_CID")
        COMMAND_BL_GET_CID_LEN = 6
        data_buf[0] = COMMAND_BL_GET_CID_LEN-1 
        data_buf[1] = COMMAND_BL_GET_CID

        crc32 = get_crc(data_buf,COMMAND_BL_GET_CID_LEN-4)
        crc32 = crc32 & 0xffffffff

        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        for i in data_buf[1:COMMAND_BL_GET_CID_LEN]:
            Write_to_serial_port(i,COMMAND_BL_GET_CID_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])
    
    elif(command == 4):
        print("\n[INFO]: Command -> BL_GET_RDP_STATUS")
        data_buf[0] = COMMAND_BL_GET_RDP_STATUS_LEN-1
        data_buf[1] = COMMAND_BL_GET_RDP_STATUS

        crc32 = get_crc(data_buf,COMMAND_BL_GET_RDP_STATUS_LEN-4)
        crc32 = crc32 & 0xffffffff

        data_buf[2] = word_to_byte(crc32,1,1)
        data_buf[3] = word_to_byte(crc32,2,1)
        data_buf[4] = word_to_byte(crc32,3,1)
        data_buf[5] = word_to_byte(crc32,4,1)
        
        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_GET_RDP_STATUS_LEN]:
            Write_to_serial_port(i,COMMAND_BL_GET_RDP_STATUS_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 5):
        print("\n[INFO]: Command -> BL_GO_TO_ADDR")
        go_address  = input("\n[INPUT]: Please enter 4 bytes go address in hex:")
        go_address = int(go_address, 16)

        data_buf[0] = COMMAND_BL_GO_TO_ADDR_LEN-1 
        data_buf[1] = COMMAND_BL_GO_TO_ADDR 
        data_buf[2] = word_to_byte(go_address,1,1) 
        data_buf[3] = word_to_byte(go_address,2,1) 
        data_buf[4] = word_to_byte(go_address,3,1) 
        data_buf[5] = word_to_byte(go_address,4,1) 

        crc32 = get_crc(data_buf,COMMAND_BL_GO_TO_ADDR_LEN-4)

        data_buf[6] = word_to_byte(crc32,1,1) 
        data_buf[7] = word_to_byte(crc32,2,1) 
        data_buf[8] = word_to_byte(crc32,3,1) 
        data_buf[9] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_GO_TO_ADDR_LEN]:
            Write_to_serial_port(i,COMMAND_BL_GO_TO_ADDR_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])
    
    elif(command == 6):
        print("\n[INFO]: Command -> BL_FLASH_ERASE")
        data_buf[0] = COMMAND_BL_FLASH_ERASE_LEN-1 
        data_buf[1] = COMMAND_BL_FLASH_ERASE 
        sector_num = input("\n[INPUT]: Enter sector number(0-7 or 0xFF):")
        sector_num = int(sector_num, 16)
        if(sector_num != 0xff):
            nsec=int(input("\n[INPUT]: Enter number of sectors to erase(max 8):"))
        
        data_buf[2]= sector_num 
        data_buf[3]= nsec 

        crc32 = get_crc(data_buf,COMMAND_BL_FLASH_ERASE_LEN-4)

        data_buf[4] = word_to_byte(crc32,1,1) 
        data_buf[5] = word_to_byte(crc32,2,1) 
        data_buf[6] = word_to_byte(crc32,3,1) 
        data_buf[7] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_FLASH_ERASE_LEN]:
            Write_to_serial_port(i,COMMAND_BL_FLASH_ERASE_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 7):
        print("\n[INFO]: Command -> BL_MEM_WRITE")
        bytes_remaining=0
        t_len_of_file=0
        bytes_so_far_sent = 0
        len_to_read=0
        base_mem_address=0

        data_buf[1] = COMMAND_BL_MEM_WRITE

        #a. Get the total number of bytes in the binary file
        t_len_of_file =calc_file_len()

        #b. Open the File
        open_the_file()

        bytes_remaining = t_len_of_file - bytes_so_far_sent

        base_mem_address = input("\n[INPUT]: Enter the memory write address:")
        base_mem_address = int(base_mem_address, 16)
        global mem_write_active
        while(bytes_remaining):
            mem_write_active=1
            if(bytes_remaining >= 128):
                len_to_read = 128
            else:
                len_to_read = bytes_remaining
            #Read the file to get the bytes in the buffer
            for x in range(len_to_read):
                file_read_value = bin_file.read(1)
                file_read_value = bytearray(file_read_value)
                data_buf[7+x]= int(file_read_value[0])
            #print("\n   base mem address = \n",base_mem_address, hex(base_mem_address)) 

            #Populate the Base Memory Address
            data_buf[2] = word_to_byte(base_mem_address,1,1)
            data_buf[3] = word_to_byte(base_mem_address,2,1)
            data_buf[4] = word_to_byte(base_mem_address,3,1)
            data_buf[5] = word_to_byte(base_mem_address,4,1)

            data_buf[6] = len_to_read

            #1 byte len + 1 byte command code + 4 byte mem base address
            #1 byte payload len + len_to_read is amount of bytes read from file + 4 byte CRC
            mem_write_cmd_total_len = COMMAND_BL_MEM_WRITE_LEN+len_to_read

            #Frst field is "length to follow"
            data_buf[0] = mem_write_cmd_total_len-1

            crc32 = get_crc(data_buf,mem_write_cmd_total_len-4)

            data_buf[7+len_to_read]  = word_to_byte(crc32,1,1)
            data_buf[8+len_to_read]  = word_to_byte(crc32,2,1)
            data_buf[9+len_to_read]  = word_to_byte(crc32,3,1)
            data_buf[10+len_to_read] = word_to_byte(crc32,4,1)

            #Update the Base Memory Address for the next loop
            base_mem_address+=len_to_read

            Write_to_serial_port(data_buf[0],1)
        
            for i in data_buf[1:mem_write_cmd_total_len]:
                Write_to_serial_port(i,mem_write_cmd_total_len-1)

            bytes_so_far_sent+=len_to_read
            bytes_remaining = t_len_of_file - bytes_so_far_sent
            print("\n[INFO]: bytes_so_far_sent:{0} | bytes_remaining:{1}\n".format(bytes_so_far_sent,bytes_remaining)) 
        
            ret_value = read_bootloader_reply(data_buf[1])
        mem_write_active=0

    elif(command == 8):
        print("\n[INFO]: Command -> BL_EN_R_W_PROTECT")
        total_sector = int(input("\n[INPUT]: Enter no of sectors to be protected: "))
        sector_numbers = [0,0,0,0,0,0,0,0]
        sector_details=0
        for x in range(total_sector):
            sector_numbers[x]=int(input("\n[INFO]: Enter sector number[{0}]: ".format(x+1)))
            sector_details = sector_details | (1 << sector_numbers[x])

        #print("Sector details",sector_details)
        print("\n[INFO]: Mode -> Flash sectors Write Protection: 1")
        print("\n[INFO]: Mode -> Flash sectors Read/Write Protection: 2")
        mode=input("\n[INPUT]: Enter Sector Protection Mode(1 or 2):")
        mode = int(mode)
        if(mode != 2 and mode != 1):
            print("\n[ERROR]: Invalid option!!")
            return
        if(mode == 2):
            print("\n[ERROR]: This feature is currently not supported!!") 
            return

        data_buf[0] = COMMAND_BL_EN_R_W_PROTECT_LEN-1 
        data_buf[1] = COMMAND_BL_EN_R_W_PROTECT 
        data_buf[2] = sector_details 
        data_buf[3] = mode

        crc32 = get_crc(data_buf,COMMAND_BL_EN_R_W_PROTECT_LEN-4) 

        data_buf[4] = word_to_byte(crc32,1,1) 
        data_buf[5] = word_to_byte(crc32,2,1) 
        data_buf[6] = word_to_byte(crc32,3,1) 
        data_buf[7] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_EN_R_W_PROTECT_LEN]:
            Write_to_serial_port(i,COMMAND_BL_EN_R_W_PROTECT_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 9):
        print("\n[INFO]: Command -> COMMAND_BL_READ_SECTOR_P_STATUS")
        data_buf[0] = COMMAND_BL_READ_SECTOR_P_STATUS_LEN-1 
        data_buf[1] = COMMAND_BL_READ_SECTOR_P_STATUS 

        crc32 = get_crc(data_buf,COMMAND_BL_READ_SECTOR_P_STATUS_LEN-4)

        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_READ_SECTOR_P_STATUS_LEN]:
            Write_to_serial_port(i,COMMAND_BL_READ_SECTOR_P_STATUS_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif(command == 10):
        print("\n[INFO]: Command -> COMMAND_BL_DIS_R_W_PROTECT")
        data_buf[0] = COMMAND_BL_DIS_R_W_PROTECT_LEN-1 
        data_buf[1] = COMMAND_BL_DIS_R_W_PROTECT

        crc32 = get_crc(data_buf,COMMAND_BL_DIS_R_W_PROTECT_LEN-4) 

        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_DIS_R_W_PROTECT_LEN]:
            Write_to_serial_port(i,COMMAND_BL_DIS_R_W_PROTECT_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    else:
        print("\n[ERROR]: Please Enter valid command code\n")
        return

    if ret_value == -2 :
        print("\n[ERROR]: TimeOut, No Response from Bootloader")
        return
#----------------------------------------------------------------------------------


#--------------------------- Command Menu implementation---------------------------
name = input("Enter the Port Name of your device(Example -> COM3):")
ret = 0

#process Port name
ret=Serial_Port_Configuration(name)

#Hanling invalid port name
if(ret < 0):
    decode_menu_command_code(0)
    

while True:
    print("\n|*******************************************|")
    print("|                 Menu                      |")
    print("|         STM32F407 BootLoader v1           |")
    print("|*******************************************|")
    print("\nPlease select the bootloader command to send\n")
    print("BL_GET_VER                            --> 1")
    print("BL_GET_HLP                            --> 2")
    print("BL_GET_CID                            --> 3")
    print("BL_GET_RDP_STATUS                     --> 4")
    print("BL_GO_TO_ADDR                         --> 5")
    print("BL_FLASH_ERASE                        --> 6")
    print("BL_MEM_WRITE                          --> 7")
    print("BL_EN_R_W_PROTECT                     --> 8")
    print("BL_READ_SECTOR_P_STATUS               --> 9")
    print("BL_DIS_R_W_PROTECT                    --> 10")
    print("EXIT                                  --> 0")

    command_code = input("\n[INPUT]: Please Enter the command code,")

    if(not command_code.isdigit()):
        print("\n[ERROR]: Please enter valid command!!")
    else:
        decode_menu_command_code(int(command_code))

    input("\n[INFO]: Press any key to continue")
    purge_serial_port()
#----------------------------------------------------------------------------------