from enum import Enum, unique
from serial import Serial
from serial.tools import list_ports
import struct


class Raven:
    @unique
    class MotorChannel(Enum):
        CH1 = b"\x00"
        CH2 = b"\x01"
        CH3 = b"\x02"
        CH4 = b"\x03"
        CH5 = b"\x04"

    @unique
    class MotorMode(Enum):
        SPEED = b"\x00"
        POSITION = b"\x01"

    @unique
    class ServoChannel(Enum):
        CH1 = b"\x00"
        CH2 = b"\x01"
        CH3 = b"\x02"
        CH4 = b"\x03"

    # Message types
    @unique
    class __MessageType(Enum):
        NACK = 0
        ACK = 1
        REPLY = 2
        MOTOR_CMD = 3
        MOTOR_MODE = 4
        MOTOR_PID = 5
        SERVO_VALUE = 6
        ENCODER_VALUE = 7
        ERROR = 8

    class __RavenSerial:
        """
        /***
        * Message format
        * Start: 1 byte
        *   [7-0] start byte
        * Length: 1 byte
        *   [7-0] data length - 1 (for required header)
        * Data: at least 1 byte
        *   header: 1 byte
        *     [7] 0: read, 1: write
        *     [6-0] header type
        *   data bytes : any number of bytes
        *    ...
        * CRC: 1 byte
        *   [7-0] 8-bit CRC
        */
        """

        __START = b"\xAA"
        __ACK = 0x01
        __REPLY = 0x02
        __NUM_HDR = 9

        class CRC8Encoder:
            SMBUS_POLY = 0x07

            def __init__(self, poly=SMBUS_POLY):
                self.__table = list()
                for i in range(256):
                    remainder = i
                    for bit in range(8):
                        if remainder & 0x80:
                            remainder = (remainder << 1) ^ poly
                        else:
                            remainder = remainder << 1
                    self.__table.append(remainder & 0xFF)

            def crc(self, data, start=0):
                remainder = start
                for i in range(len(data)):
                    remainder = self.__table[data[i] ^ remainder]
                return remainder

        def __init__(self, port, baud, timeout=0.001):
            self.__crc = self.CRC8Encoder()
            self.__serial = Serial(port, baud, timeout=timeout)

        def __read(self):
            # Read serial data and return (header, message)
            crc = 0
            message = bytes()
            header = 0
            data = self.__serial.read(1)

            # Start
            if data == self.__START:
                crc = self.__crc.crc(data)
                data = self.__serial.read(1)
            else:
                return None, None

            # Header
            if len(data) == 1 and data[0] < self.__NUM_HDR:
                crc = self.__crc.crc(data, crc)
                header = data[0]
                data = self.__serial.read(1)
            else:
                return None, None

            # Length
            if len(data) == 1:
                crc = self.__crc.crc(data, crc)
                length = data[0]
                data = self.__serial.read(length)
            else:
                return None, None

            # Data:
            if len(data) == 0:
                data = self.__serial.read(1)
            elif len(data) == length:
                crc = self.__crc.crc(data, crc)
                message = data
                data = self.__serial.read(1)
            else:
                return None, None

            # CRC:
            if len(data) == 1:
                crc = self.__crc.crc(data, crc)
                if crc == 0:
                    return header, message
            return None, None

        def __make_message(self, header, rw, data):
            rw_header = header.value
            if rw:
                rw_header |= 0x80
            message = self.__START + bytes([rw_header, len(data)]) + data
            crc = self.__crc.crc(message)
            message = message + bytes([crc])
            return message, crc

        def write_value(self, header, data, retry=0):
            self.__serial.reset_input_buffer()
            message, crc = self.__make_message(header, 1, data)
            self.__serial.write(message)

            header, received_crc = self.__read()
            if header and header == self.__ACK:
                if received_crc == crc.to_bytes():
                    return True
            if retry > 0:
                return self.write_value(header, data, retry - 1)
            else:
                return False

        def read_value(self, header, data, retry=0):
            self.__serial.reset_input_buffer()
            message, __ = self.__make_message(header, 0, data)
            self.__serial.write(message)

            header, received_message = self.__read()
            if header and header == self.__REPLY:
                return received_message
            if retry > 0:
                return self.read_value(header, data, retry - 1)
            else:
                return False

    def __init__(self, port=None, timeout=1):
        if port is None:
            port = sorted(list_ports.comports())[0].device
        # Raven has fixed baud at 460800
        self.__serial = Raven.__RavenSerial(port, 460800, timeout)

    def get_motor_mode(self, motor_channel: MotorChannel, retry):
        """
        Get motor mode
        @motor_channel: Raven.MotorChannel#
        @return: Raven.MotorMode.SPEED or Raven.MotorMode.POSITION or None if fails
        """
        value = self.__serial.read_value(
            Raven.__MessageType.MOTOR_MODE, motor_channel.value, retry
        )
        if value and len(value) == 1:
            if value[0] == 0:
                return Raven.MotorMode.SPEED
            if value[0] == 1:
                return Raven.MotorMode.POSITION
        return None

    def set_motor_mode(
        self, motor_channel: MotorChannel, motor_mode: MotorMode, retry=0
    ):
        """
        Set motor mode to speed or position control
        @motor_channel: Raven.MotorChannel#
        @motor_mode: Raven.MotorMode.SPEED or Raven.MotorMode.POSITION
        @retry: number of retries if command fails
        @return: True if success
        """
        return self.__serial.write_value(
            Raven.__MessageType.MOTOR_MODE,
            motor_channel.value + motor_mode.value,
            retry,
        )

    def get_motor_command(self, motor_channel: MotorChannel, retry=0):
        """
        Get motor command
        @motor_channel: Raven.MotorChannel#
        @retry: number of retries if command fails
        @return: command (rps if motor mode is speed and revolutions if motor mode is position) or None if fails
        """
        value = self.__serial.read_value(
            Raven.__MessageType.MOTOR_CMD, motor_channel.value, retry
        )
        if value and len(value) == 4:
            return struct.unpack("f", value)[0]
        return None

    def set_motor_command(self, motor_channel: MotorChannel, value: float, retry=0):
        """
        Set motor command
        @motor_channel: Raven.MotorChannel#
        @value: command (rps if motor mode is speed and revolutions if motor mode is position)
        @retry: number of retries if command fails
        @return: True if success
        """
        return self.__serial.write_value(
            Raven.__MessageType.MOTOR_CMD,
            motor_channel.value + struct.pack("f", value),
            retry,
        )

    def get_motor_pid(self, motor_channel: MotorChannel, retry=0):
        """
        Get motor PID gain
        @motor_channel: Raven.MotorChannel#
        @retry: number of retries if command fails
        @return: (P,I,D) values or None if fails
        """
        value = self.__serial.read_value(
            Raven.__MessageType.MOTOR_PID, motor_channel.value, retry
        )
        if value and len(value) == 12:
            return struct.unpack("fff", value)
        return None

    def set_motor_pid(
        self,
        motor_channel: MotorChannel,
        p_gain: float,
        i_gain: float,
        d_gain: float,
        retry=0,
    ):
        """
        Set motor PID gain
        @motor_channel: Raven.MotorChannel#
        @p_gain: P gain value
        @i_gain: I gain value
        @d_gain: D gain value
        @retry: number of retries if command fails
        @return: True if success
        """
        return self.__serial.write_value(
            Raven.__MessageType.MOTOR_PID,
            motor_channel.value + struct.pack("fff", p_gain, i_gain, d_gain),
            retry,
        )

    __SERVO_COUNT_PER_US = 168 / 51  # From raven firmware
    __SERVO_MIN_US = 1000
    __SERVO_MAX_US = 2000
    __SERVO_RANGE = __SERVO_MAX_US - __SERVO_MIN_US

    @staticmethod
    def __deg_to_count(deg):
        us = ((deg + 90) / 180 * Raven.__SERVO_RANGE) + Raven.__SERVO_MIN_US
        return int(us * Raven.__SERVO_COUNT_PER_US)

    def set_servo_position(self, servo_channel: ServoChannel, degree: float, retry=0):
        """
        Set servo position in degree
        @servo_channel: Raven.ServoChannel#
        @degree: -90 to 90
        @retry: number of retries if command fails
        @return: True if success
        """
        if degree < -90 or degree > 90:
            return ValueError("Invalid degree")

        return self.__serial.write_value(
            Raven.__MessageType.SERVO_VALUE,
            servo_channel.value + struct.pack("H", Raven.__deg_to_count(degree)),
            retry,
        )

    def get_motor_encoder(self, motor_channel: MotorChannel, retry=0):
        """
        Get motor encoder count
        @motor_channel: Raven.MotorChannel#
        @retry: number of retries if command fails
        @return: number of encoder count or None if fails
        """
        value = self.__serial.read_value(
            Raven.__MessageType.ENCODER_VALUE, motor_channel.value, retry
        )
        if value and len(value) == 4:
            return struct.unpack("i", value)[0]
        return None


if __name__ == "__main__":
    import numpy as np
    import time

    raven = Raven("COM6")

    i = 0
    while True:
        p, i, d = np.random.rand(3) * 1000
        print(p, i, d)
        pid_set = raven.set_motor_pid(Raven.MotorChannel.CH5, p, i, d)
        pid_get = raven.get_motor_pid(Raven.MotorChannel.CH5)
        print(pid_set, pid_get)
        time.sleep(0.001)
