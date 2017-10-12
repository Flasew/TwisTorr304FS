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

    return [ord(char) for char in '{:X}'.format(crc)]


class TT304(object):

    def __init__(self, port=None, retries=3, 
                 rs485=False, devno=0, timeout=0, **kwargs):
        """Constructor of an TwisWorr304FS object. Instantiates an object 
        with the given devno, port, and other relative serial commands.

        Arguments:
            port {str} -- serial port address to be opened (/dev/* or COM*)
            rs485 {bool} -- if the device runs in RS485 mode
            retries {number} -- number of trails for the query and pquery
                                command before report failure. 
            devno {number} -- device number for RS485 connection, or 0 for 
                              RS232 connection.
            timeout {number} -- default serial timeout
            kwargs {list} -- other keyword argument to be passed to the 
                             serial.Serial constructor

        Raises
            ValueError -- when mode is RS232 but devno is not 0.
        """
        if not rs485 and devno != 0:
            raise ValueError("devno must be 0 for RS232 devices")

        self.devno = devno
        self.rs485 = rs485
        self.retries = retries

        self.ser = serial.Serial(port, timeout=timeout, **kwargs)

        if self.rs485:
            self.ser.rs485_mode = serial.rs485.RS485Settings()

        if not self.ser.is_open:
            self.ser.open()

    def __del__(self):
        self.ser.close()

    def pack(self, window, mode=RD, data=''):
        """Pack the a message to the device-compatible format, including the
        start byte, stop byte, crc, etc. See device manual for more details.
        
        Arguments: 
            window {int | str} -- Integer between 0 and 999 inclusive 
                                  or string of three digits
                                  XXX: make sure don't give number start
                                  with 0. Those number will be recognized as 
                                  octal number and will not work as intended.
            mode {str} -- mode for the message, WR or RD
            data {str} -- String to write to window. 
                          Must be empty for read requests

        Returns:
            bytearray -- encoded byte array

        Raises:
            ValueError -- when the argument does not make a valid message.
        """

        if mode == RD and len(data) != 0:
            raise ValueError('Cannot write data in a read request')

        if isinstance(window, int):
            window = '{:03d}'.format(window)
        if len(window) != 3:
            raise ValueError('Window number must be in the interval [0,999]')

        # Allowing data to be int mainly for logical values
        if isinstance(data, int):
            if data != ON and data != OFF:
                raise ValueError("Data must be string unless it's ON/OFF logic")
            try:
                data = chr(data)
            except:
                raise ValueError("Data must be string unless it's ON/OFF logic")

        address = 0x80 + self.devno
        window = [ord(char) for char in window]
        data = [ord(char) for char in data]

        bytes = [STX,address]
        bytes.extend(window)
        bytes.append(mode)
        bytes.extend(data)
        bytes.append(ETX)
        bytes.extend(crc(bytes[1:]))

        return bytearray(bytes)

    def send_raw(self, msg):
        """Send a message to the device. The message is expected to be already
        encoded in the proper format.

        Arguments:
            msg {bytes|bytearray} -- encoded message in a compatible format for
                                     Serial.write()

        Raises:
            IOError -- happens when number of bytes sent is less then the 
                       length of message
        """
        if self.ser.write(msg) != len(msg):
            raise IOError('send failed')
        self.ser.flush()

    def send(self, window, mode=RD, data=''):
        """Pack and send, wraps pack and send_raw commands.

        Arguments: 
            window {int | str} -- Integer between 0 and 999 inclusive 
                                  or string of three digits
            mode {str} -- mode for the message, WR or RD
            data {str} -- String to write to window. 
                          Must be empty for read requests

        Returns:
            bool -- true on success, False otherwise
        """

        try:
            self.send_raw(self.pack(window, mode, data))
            return True

        except IOError:
            print("Error sending message.")
            return False

    def receive(self):
        """ Returns the raw byte stream the device send out, as a byte list
        Due to the device using a special ETX to signal end of transmission,
        some modifications have to be made to ensure this call will not block 
        forever.
        """

        result = []
        
        while True:
            result.append(self.ser.read())
            if result[-1] == chr(ETX):
                break
            # case of a read 'block'
            elif result[-1] == '':
                raise IOError('Empty or incomplete response from the device.')

        # crc bits
        result.append(self.ser.read(2))
        if result[-1] == '':
            raise IOError('Empty or incomplete response from the device.')

        return ''.join(result)

    def query_raw(self, msg, waittime=0.2):
        """send a message and get the response. 
        
        Argument:
            msg {bytes|bytearray} -- encoded message in a compatible format for
                                     Serial.write()
            waittime {number} -- time to wait after command is sent and start 
                                 of receiving the response

        Returns:
            bytes - response from the device if send and receive succeeded, 
                    None otherwise
        """
        retries = self.retries

        while retries > 0:
            try:
                self.send_raw(msg)

            except IOError:
                print("Error sending message on trail {:d}."\
                    .format(self.retries-retries+1))
                retries -= 1
                continue
                
            # give the machine some time
            time.sleep(waittime)
            
            try:
                return self.receive()

            except IOError:
                print("Error receiving response on trail {:d}."\
                    .format(self.retries-retries+1))
                retries -= 1
                continue

        print("ERROR: All trails failed.")
        return None

    def query(self, window, mode=RD, data=''):
        """wrapper for pack a message, and send, and receive the response
        Arguments: 
            window {int | str} -- Integer between 0 and 999 inclusive 
                                  or string of three digits
            mode {str} -- mode for the message, WR or RD
            data {str} -- String to write to window. 
                          Must be empty for read requests

        Returns:
            bytes -- response from the device on success, None otherwise
        """
        return self.query_raw(self.pack(window, mode, data))

    def query_unpack(self, window, mode=RD, data=''):
        """Query a command and unpack the response"""
        return self.unpack(self.query(window, mode, data))

    @staticmethod
    def unpack(reply):
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

        crc_result = ''.join([chr(byte) for byte in crc(byte_list)])

        if reply[-2:] != crc_result:
            checksum_ok = False
            print("Warning: checksum FAILED for the command, please verify "
                  "controller status.")

        devno = struct.unpack('>i','\x00\x00\x00'+reply[1])[0]

        return reply[2:-3], devno-0x80, checksum_ok
            
    ############################################################################
    # Below are wrappers for some specific commands that are commonly used     #
    # All of them will print a warning if checksum fails, but the return format#
    # may depend on specific implementations.                                  #
    ############################################################################

    def start(self):
        """Start the pump. Details:

            Window: 000
            R/W: W
            Data: L, 1

        Return true on success, false other wise"""
        resp = self.query_unpack(0, WR, ON)

        if resp is None:
            print("Command send failed.")
            return False
        
        if ord(resp[0][0]) == ACK:
            print("Start: success")
            return True
        else:
            print("Start: failed, error code {:d}".format(ord(resp[0][0])))
            return False

    def stop(self):
        """Stop the pump. Details:

            Window: 000
            R/W: W
            Data: L, 0

        Return true on success, false other wise"""
        
        resp = self.query_unpack(0, WR, OFF)

        if resp is None:
            print("Command send failed.")
            return False

        if ord(resp[0][0]) == ACK:
            print("Stop: success")
            return True
        else:
            print("Stop: failed, error code {:d}".format(ord(resp[0][0])))
            return False

    def read_pressure(self):
        """Read the gauger pressure from the controller in it's current unit.

        Details:

            Window: 224
            R/W: R
            Data: A

        Return the value of pressure in float, or None if failed.
        """

        resp = self.query_unpack(224, RD)

        if resp is None:
            print("Command send failed.")
            return False

        return float(resp[0][4:])



    def read_pressure_unit(self, unit='mbar'):
        """Read the gauger pressure from the controller in the specified unit.
        Unit can be set to 'mbar', 'pa' or 'torr'
        Details:

            Window: 224
            R/W: R
            Data: A

        For setting reading unit:

            Window: 163,
            R/W: W
            Data: N
        
        Return the value of pressure in float, or None if failed.
        """

        # Set reading unit
        if unit.lower() == 'mbar':
            unit = '000000'
        elif unit.lower() == 'pa':
            unit = '000001'
        elif unit.lower() == 'torr':
            unit = '000002'
        else:
            print("Wrong or unsupported unit.")
            return False

        resp = self.query_unpack(163, WR, unit)

        if resp is None:
            print("Command send failed.")
            return False
            
        if ord(resp[0][0]) != ACK:
            print("Chang of pressure reading unit FAILED, error code {:d}"\
                .format(ord(resp[0][0])))
            return False

        return self.read_pressure()


