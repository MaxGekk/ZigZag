#!/usr/bin/env python2.4
import struct
import sys
import serial

#PORT="/dev/ttyS0"
PORT="/dev/ttyUSB1"

def sum(packet):
    return reduce(lambda a,b:a+b, [ord(x) for x in packet])

def toString(packet):
    return " ".join(["%02X"%ord(x) for x in packet])

def code(packet):
    result = []
    for char in packet:
        if char in ('\xEB','\xDA'):
            result += ['\xDA']
            result += [chr(0x19 ^ ord(char))]
        else:
            result += [char]
    return "".join(result)


# Q=64b, H=16b, B=8b, all unsigned

def request_attr(address, port, attr_num):
    # dst short addr, dst port, src port, msg type, body length, attr_num
    return struct.pack("<HBBBBB", address, port, 0, 0, 1, attr_num)

def set_attr(address, port, attr_num, size_code, attr_value):
    return struct.pack("<HBBB",size_code,address, port, 1, attr_num, attr_value)

#----------------------------------

data = request_attr(0xFFFF, 0x13, 0xe2)
#data = set_attr(1, 2, 3, "Q", 1)

#data = data + data[2:] + data[2:] + data[2:]
data = struct.pack("<BB",len(data)+4,0x71) + data 
data += chr(sum(data)%256)
packet = "\xeb"+ code(data)

#print ["%02X"%ord(x) for x in packet]


port = serial.Serial(port=PORT, baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0, xonxoff=0, rtscts=0)

import time

while(True):
    port.write("\x55\x00")
    time.sleep(0.0005)

#sys.stdout.write(packet)
#sys.stdout.flush()

print toString(packet)
