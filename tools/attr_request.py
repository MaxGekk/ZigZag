#!/usr/bin/env python2.4
import struct
import sys
import serial
import time

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
    return struct.pack("<HBBBB", address, port, 0, 0, attr_num)

def set_attr(address, port, attr_num, attr_value):
    return struct.pack("<HBBBBH",  address, port, 0, 1, attr_num, attr_value)

#----------------------------------

port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0, xonxoff=0, rtscts=0)

attrs = {
'FIRE_DAY_HI': 0x2C,
'FIRE_DAY_LO': 0x2D,
'FIRE_NIGHT_HI': 0x30,
'FIRE_NIGHT_LO': 0x31,
'BACKROUND_SIGNAL': 0xE6,
'FULL_RANGE_SIGNAL': 0xE7,
'CONTAMINATION_OFFSET': 0xE8,
'MAX_CONTAMINATION_OFFSET': 0xE9,
'MIN_CONTAMINATION_OFFSET': 0xEA,
}


def read_response(port):
    timeout = port.getTimeout()
    port.setTimeout(3)
    incoming = port.read(1000)
    print "<", toString(incoming)
    port.setTimeout(timeout)
    

def read_attrs():
    attr_nums = attrs.values()
    attr_nums.sort()
    for attr in attr_nums:
        data = request_attr(0x0001, 0x0A, attr)
        data = struct.pack("<BB",len(data)+4,0x71) + data 
        data += chr(sum(data)%256)
        packet = "\xeb"+ code(data)
        port.write(packet)
        print ">", toString(packet)
        read_response(port)

def foo():
    #data = set_attr(0x0001, 0x0A, attrs['BACKROUND_SIGNAL'], 0x8888)
    #data = set_attr(0x0001, 0x0A, 0x2C, 0x0123)
    data = request_attr(0x0001, 0x0A, 0xE6)
    data = struct.pack("<BB",len(data)+4,0x71) + data 
    data += chr(sum(data)%256)
    packet = "\xeb"+ code(data)
    port.write(packet)
    print ">", toString(packet)


# waiting for anything to come
incoming = port.read()

waiting = port.inWaiting()
print "<", toString(incoming + port.read(waiting))

read_attrs()
