# python module for controlling the Twis Torr 304 FS vacuum Controller
# Initially written by Alex Zahn, current version modified by Weiyang Wang

from __future__ import print_function

import time
import struct
import serial

ACK = 0x06
NACK = 0x15
UNKNOWN_WINDOW = 0x32
DATA_TYPE_ERROR = 0x33
OUT_OF_RANGE = 0x34
WIN_DISABLED = 0x35

STX = 0x02
ETX = 0x03

ON = 0x31
OFF = 0x30

WR = 0x31
RD = 0x30

def crc(byte_list):
    """Compute the CRC for a byte list, as specified by the instruction. 
    Returns two ASCII values of the computed CRC values in a list."""

    crc = 0x0

    for byte in byte_list:
        crc ^= byte

    return [ord(char) for char in '{:0.2X}'.format(crc)]


class TwisTorr304FS(object):

    def __init__(self, port=None, timeout=5, **kwargs):

        self.ser = serial.Serial(port, timeout=timeout, kwargs)
        if not self.ser.is_open:
            self.ser.open()
        # number of response not read
        self.num_res = 0

    def __del__(self):
        self.ser.close()

    def send(self, window, com='r', data='', devno=0):
        """
        @window:
            Integer between 0 and 999 inclusive, or string of three digits

        @com:
            Must be 'r' to read window or 'w' to write window

        @data:
            String to write to window. Must be None for read requests

        @devno:
            RS-485 device number, or 0 for RS-232

        """

        request = pack_request(window,com,data,devno)
        nbytes_sent = self.ser.write(request)

        if nbytes_sent != len(request):
            raise IOError('send failed')

        self.num_res += 1
        self.ser.flush()

    def receive(self):
        """ Returns the raw byte stream the device send out, as a byte list"""
        if self.num_res == 0:
            raise IOError('No message available from the controller.')

        result = []
        
        while True:
            result.append(self.ser.read())
            print result
            if result[-1] == b'\x03':
                break

        result.append(self.ser.read(2))
        return result

    def query(self, )


    @staticmethod
    def pack(window, com='r', data='', devno = 0):
        """
        @window:
            Integer between 0 and 999 inclusive, or string of three digits

        @com:
            Must be 'r' to read window or 'w' to write window

        @data:
            String to write to window. Must be None for read requests

        @devno:
            RS-485 device number, or 0 for RS-232

        """

        if com =='r' and len(data) != 0:
            raise ValueError('Cannot write data in a read request')

        if isinstance(window, int):
            window = '{:0.3d}'.format(window)

        if len(window) != 3:
            raise ValueError('Window number must be in the interval (0,999)')

        address = 0x80 + devno
        window = [ord(char) for char in window]
        com  = WR if com == 'w' else RD
        data = [ord(char) for char in data]

        bytes = [STX,address]
        bytes.extend(window)
        bytes.append(com)
        bytes.extend(data)
        bytes.append(ETX)
        bytes.extend(crc(bytes[1:]))

        return bytearray(bytes)

