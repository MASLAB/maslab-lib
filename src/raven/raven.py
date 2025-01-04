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
        DISABLE = b"\x00"
        DIRECT = b"\x01"
        POSITION = b"\x02"
        SPEED = b"\x03"

    @unique
    class ServoChannel(Enum):
        CH1 = b"\x00"
        CH2 = b"\x01"
        CH3 = b"\x02"
        CH4 = b"\x03"

    # Message types
    @unique
    class __MessageType(Enum):
        SERVO_VALUE = 0
        MOTOR_MODE = 1
        MOTOR_PID = 2
        MOTOR_CMD = 3
        MOTOR_VOLTAGE = 4
        MOTOR_CURRENT = 5
        ENCODER_VALUE = 6
        MOTOR_MEAS_VOLTAGE = 7
        MOTOR_MEAS_CURRENT = 8

    # Read write option
    @unique
    class __ReadWrite(Enum):
        # First bit of message type
        WRITE = 0x00
        READ = 0x80

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

        __START = b"\0xAA"

        @unique
        class ReadWrite(Enum):
            WRITE = 0
            READ = 1

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
            data = self.__serial.read(1)

            # Start
            if data == self.__START:
                crc = self.__crc.crc(data)
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
            if len(data) == length:
                crc = self.__crc.crc(data, crc)
                message = data
                data = self.__serial.read(1)
            else:
                return None, None

            # CRC:
            if len(data) == 1:
                crc = self.__crc.crc(data, crc)
                if crc == 0:
                    return message[0], message[1:]
            return None, None

        def __make_message(self, data):
            message = self.__START + bytes([len(data)]) + data
            crc = self.__crc.crc(message)
            message = message + bytes([crc])
            return message, crc

        def send(self, data, retry=0):
            self.__serial.reset_input_buffer()
            message, crc = self.__make_message(data)
            self.__serial.write(message)

            received_crc, data = self.__read()
            if received_crc == crc.to_bytes():
                return data
            if retry > 0:
                return self.send(data, retry - 1)
            else:
                return None

    def __init__(self, port: str = None, timeout: float = 0.001):
        """
        Initialize a Raven communication instance
        @port: String of the serial port connected to Raven. Will detect the first open port if not specified.
        @timeout: Serial timeout in second. Default to 1ms
        """
        if port is None:
            port = sorted(list_ports.comports())[0].device
        # Raven has fixed baud at 460800
        self.__serial = Raven.__RavenSerial(port, 460800, timeout)

    @staticmethod
    def __make_message(message_type: __MessageType, rw: __ReadWrite, data):
        header_byte = bytes([message_type.value + rw.value])
        return header_byte + data

    def __read_value(self, message_type: __MessageType, data, retry=0):
        message = Raven.__make_message(message_type, Raven.__ReadWrite.READ, data)
        return self.__serial.send(message, retry)

    def __write_value(self, message_type: __MessageType, data, retry=0):
        message = Raven.__make_message(message_type, Raven.__ReadWrite.WRITE, data)
        return self.__serial.send(message, retry) is not None

    def get_motor_mode(self, motor_channel: MotorChannel, retry):
        """
        Get motor mode
        @motor_channel: Raven.MotorChannel#
        @return: Raven.MotorMode or None if fails
        """
        assert type(motor_channel) == Raven.MotorChannel
        value = self.__read_value(
            Raven.__MessageType.MOTOR_MODE, motor_channel.value, retry
        )
        if value and len(value) == 1:
            for mode in Raven.MotorMode:
                if value == mode:
                    return mode
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
        assert type(motor_channel) == Raven.MotorChannel
        return self.__write_value(
            Raven.__MessageType.MOTOR_MODE,
            motor_channel.value + motor_mode.value,
            retry,
        )

    def get_motor_command(self, motor_channel: MotorChannel, retry=0):
        """
        Get motor command
        @motor_channel: Raven.MotorChannel#
        @retry: number of retries if command fails
        @return: Motor command (based on set mode) or None if fails
        """
        assert type(motor_channel) == Raven.MotorChannel
        value = self.__read_value(
            Raven.__MessageType.MOTOR_CMD, motor_channel.value, retry
        )
        if value and len(value) == 4:
            return struct.unpack("f", value)[0]
        return None

    def set_motor_command(self, motor_channel: MotorChannel, value: float, retry=0):
        """
        Set motor command
        @motor_channel: Raven.MotorChannel#
        @value: command in encoder count if mode is position or encoder count/sec if mode is velocity
        @retry: number of retries if command fails
        @return: True if success
        """
        assert type(motor_channel) == Raven.MotorChannel
        return self.__write_value(
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
        assert type(motor_channel) == Raven.MotorChannel
        value = self.__read_value(
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
        return self.__write_value(
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
        assert type(servo_channel) == Raven.ServoChannel
        if degree < -90 or degree > 90:
            return ValueError("Invalid degree")

        return self.__write_value(
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
        assert type(motor_channel) == Raven.MotorChannel
        value = self.__read_value(
            Raven.__MessageType.ENCODER_VALUE, motor_channel.value, retry
        )
        if value and len(value) == 4:
            return struct.unpack("i", value)[0]
        return None


if __name__ == "__main__":
    import numpy as np
    import time

    raven = Raven()

    i = 0
    while True:
        # p, i, d = np.random.rand(3) * 1000
        # print(p, i, d)
        # pid_set = raven.set_motor_pid(Raven.MotorChannel.CH5, p, i, d)
        # pid_get = raven.get_motor_pid(Raven.MotorChannel.CH5)
        # print(pid_set, pid_get)
        # time.sleep(0.001)
        raven.get_motor_encoder(Raven.MotorChannel.CH1, 1)
        break
