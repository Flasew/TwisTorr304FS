import re
import sys
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

reply_pattern = '\%c.*?\%c\w{2}' % (STX,ETX)
reply_regex = re.compile(reply_pattern)

def crc_bytes(byte_list):
    crc = 0x0
    for byte in byte_list:
        crc ^= byte
    return [ord(char) for char in '%0.2X' % crc]


def pack_request(window, com='r', data='',devno = 0):
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

    if com =='r' and not len(data) == 0:
        raise Exception('Cannot write data in a read request')

    if isinstance(window,int):
        window = '%0.3i'%window
    print window

    if len(window) != 3:
        raise Exception('Window number must be in the interval (0,999)')

    ADDR = 0x80 + devno

    window = [ord(char) for char in window]
    print window

    COM  = 0x31 if com == 'w' else 0x30

    data = [ord(char) for char in data]


    bytes = [STX,ADDR]
    bytes.extend(window)
    bytes.append(COM)
    bytes.extend(data)
    bytes.append(ETX)

    for b in bytes:
        print "{:X}".format(b)

    crc = crc_bytes(bytes[1:])

    bytes.extend(crc)

    return bytearray(bytes)

def scan_for_replies(stream):
    """
    Return a list of replies found in stream, and return a version
    of stream with those replies removed.
    """
    replies = reply_regex.findall(stream)
    stream = reply_regex.sub('',stream)
    return replies, stream

def unpack_reply(reply):
    """
    Returns the devno value, a string containing the reply
    contents between the ADDR and message terminator, and whether
    or not the checksum failed.

    Note that no attempt is made at parsing the reply beyond this.
    The substance of the reply we return here can be formatted in
    a few possible ways, depending on the window number involved.

    We should get something human readable back at this stage, so
    it seems to make sense to let the user take care of their own
    parsing needs.
    """

    if reply == None or len(reply) == 0:
        raise Exception('Cannot unpack empty reply')

    checksum_ok = True

    byte_list = [ ord(char) for char in reply[1:-2] ]

    crc = ''.join([chr(byte) for byte in crc_bytes(byte_list)])

    if reply[-2:] != crc:
        checksum_ok = False

    addr = struct.unpack('>i','\x00\x00\x00'+reply[1])[0]

    return addr-0x80, reply[2:-3], checksum_ok



class controller(object):

    def __init__(self,port):

        self.ser = serial.Serial(port,timeout=None)

        if not self.ser.is_open:
            self.ser.open()

        self.stream = ''

    def __del__(self):
        self.ser.close()

    def send(self, window, com='r', data='',devno = 0):
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
            raise Exception('send failed')

        self.ser.flush()

    def recv(self,retries=1):

        replies = []

        while retries > 0:

            # TODO: make sure this shouldn't be out_waiting
            try:
                nbytes_waiting = 128
            except:
                nbytes_waiting = in_waiting

            if nbytes_waiting == 0:
                time.sleep(0.1)
                print 'no bytes waiting, retries remaining = %i' %retries
                sys.stdout.flush()
                retries -= 1
                continue

            self.stream += self.ser.read(nbytes_waiting)
            print self.stream
            r, self.stream = scan_for_replies(self.stream)

            replies += [unpack_reply(_r) for _r in r]

            if len(replies) > 0:
                break
            else:
                retries -= 1

        return replies

    def read(self):
        """ Returns the raw byte stream the device send out, as a byte list"""
        result = []
        
        while True:
            result.append(self.ser.read())
            print result
            if result[-1] == b'\x03':
                break

        result.append(self.ser.read(2))

        return result

