from enum import Enum, unique
from serial import Serial
from serial.tools import list_ports
import struct


class Raven:
    @unique
    class MotorChannel(Enum):
        CH1 = 0
        CH2 = 1
        CH3 = 2
        CH4 = 3
        CH5 = 4

    @unique
    class MotorMode(Enum):
        DISABLED = b"\x00"
        DIRECT = b"\x01"
        POSITION = b"\x02"
        VELOCITY = b"\x03"

    @unique
    class ServoChannel(Enum):
        CH1 = 0
        CH2 = 1
        CH3 = 2
        CH4 = 3

    # Message types
    @unique
    class __MessageType(Enum):
        SERVO_VALUE = 0 << 3
        MOTOR_MODE = 1 << 3
        MOTOR_PID = 2 << 3
        MOTOR_TARGET = 3 << 3
        MOTOR_VOLTAGE = 4 << 3
        MOTOR_CURRENT = 5 << 3
        ENCODER_VALUE = 6 << 3
        MOTOR_VELOCITY_VALUE = 7 << 3  # Read-only
        MOTOR_VOLTAGE_VALUE = 8 << 3  # Read-only
        MOTOR_CURRENT_VALUE = 9 << 3  # Read-only

    @unique
    class __ReadWrite(Enum):
        WRITE = 0x00
        READ = 0x80

    class __RavenSerial:
        """
        /***
        * Message format
        * Start: 1 byte
        *   [7-0] start byte
        * Header: 1 byte
        *   [7] ack
        *   [6-2] length
        *   [1-0] PID (unused)
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

            # Header
            if len(data) == 1:
                crc = self.__crc.crc(data, crc)
                length = (data[0] >> 2) & 0b11111
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

        def __make_message(self, ack: bool, data: bytes):
            header = len(data) << 2
            if ack:
                header += 0x80  # Set first bit to ack
            message = self.__START + bytes([header]) + data
            crc = self.__crc.crc(message)
            message = message + bytes([crc])
            return message, crc

        def send(self, ack, data, retry=0):
            self.__serial.reset_input_buffer()
            message, crc = self.__make_message(ack, data)
            self.__serial.write(message)

            received_crc, received_data = self.__read()
            if received_crc == crc:
                return received_data
            if retry > 0:
                return self.send(ack, data, retry - 1)
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
    def __make_message(
        message_type: __MessageType,
        rw: __ReadWrite,
        channel: Enum = None,
        data: bytes = bytes(),
    ):
        if channel is None:
            channel_value = 0
        else:
            channel_value = channel.value
        return bytes([message_type.value + rw.value + channel_value]) + data

    def __read_value(
        self,
        message_type: __MessageType,
        channel: Enum = None,
        retry=0,
    ):
        message = Raven.__make_message(message_type, Raven.__ReadWrite.READ, channel)
        return self.__serial.send(True, message, retry)

    def __write_value(
        self,
        message_type: __MessageType,
        channel: Enum = None,
        data: bytes = bytes(),
        retry=0,
    ):
        message = Raven.__make_message(
            message_type, Raven.__ReadWrite.WRITE, channel, data
        )
        return self.__serial.send(True, message, retry) is not None

    def get_motor_mode(self, motor_channel: MotorChannel, retry=0):
        """
        Get motor mode
        @motor_channel: Raven.MotorChannel#
        @return: Raven.MotorMode or None if fails
        """
        assert type(motor_channel) == Raven.MotorChannel
        value = self.__read_value(Raven.__MessageType.MOTOR_MODE, motor_channel, retry)
        if value and len(value) == 1:
            for mode in Raven.MotorMode:
                if value == mode.value:
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
            motor_channel,
            motor_mode.value,
            retry,
        )

    def get_motor_target(self, motor_channel: MotorChannel, retry=0):
        """
        Get motor PID target
        @motor_channel: Raven.MotorChannel#
        @retry: number of retries if command fails
        @return: Motor target (based on set mode) or None if fails
        """
        assert type(motor_channel) == Raven.MotorChannel
        value = self.__read_value(
            Raven.__MessageType.MOTOR_TARGET, motor_channel, retry
        )
        if value and len(value) == 4:
            return struct.unpack("f", value)[0]
        return None

    def set_motor_target(self, motor_channel: MotorChannel, value: float, retry=0):
        """
        Set motor PID target
        @motor_channel: Raven.MotorChannel#
        @value: command in encoder count if mode is position or encoder count/sec if mode is velocity
        @retry: number of retries if command fails
        @return: True if success
        """
        assert type(motor_channel) == Raven.MotorChannel
        return self.__write_value(
            Raven.__MessageType.MOTOR_TARGET,
            motor_channel,
            struct.pack("f", value),
            retry,
        )

    def set_motor_speed_factor(
        self, motor_channel: MotorChannel, percent: float, reverse=False, retry=0
    ):
        assert type(motor_channel) == Raven.MotorChannel
        assert percent <= 100 and percent >= 0
        value = int(percent * 40.95)
        if reverse:
            value = -value
        return self.__write_value(
            Raven.__MessageType.MOTOR_VOLTAGE,
            motor_channel,
            struct.pack("h", value),
            retry,
        )

    def set_motor_torque_factor(
        self, motor_channel: MotorChannel, percent: float, retry=0
    ):
        assert type(motor_channel) == Raven.MotorChannel
        assert percent <= 100 and percent >= 0
        value = int(percent * 40.95)
        return self.__write_value(
            Raven.__MessageType.MOTOR_CURRENT,
            motor_channel,
            struct.pack("H", value),
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
        value = self.__read_value(Raven.__MessageType.MOTOR_PID, motor_channel, retry)
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
            motor_channel,
            struct.pack("fff", p_gain, i_gain, d_gain),
            retry,
        )

    __SERVO_COUNT_PER_US = 168 / 51  # From raven firmware

    @staticmethod
    def __deg_to_count(deg, min_us=1000, max_us=2000):
        us_range = max_us - min_us
        us = ((deg + 90) / 180 * us_range) + min_us
        return int(us * Raven.__SERVO_COUNT_PER_US)

    @staticmethod
    def __count_to_deg(count, min_us=1000, max_us=2000):
        us_range = max_us - min_us
        us = count / Raven.__SERVO_COUNT_PER_US
        return (us - min_us) / us_range * 180 - 90

    def get_servo_position(
        self, servo_channel: ServoChannel, min_us=1000, max_us=2000, retry=0
    ):
        """
        Get servo position in degree
        @servo_channel: Raven.ServoChannel#
        @min_us: servo's minimum pulse in microsecond
        @max_us: servo's maximum pulse in microsecond
        @retry: number of retries if command fails
        @return: servo position in degree or None if fails
        """
        assert type(servo_channel) == Raven.ServoChannel

        value = self.__read_value(
            Raven.__MessageType.SERVO_VALUE,
            servo_channel,
            retry,
        )

        if value and len(value) == 2:
            count = struct.unpack("H", value)[0]
            return Raven.__count_to_deg(count, min_us, max_us)

    def set_servo_position(
        self,
        servo_channel: ServoChannel,
        degree: float,
        min_us=1000,
        max_us=2000,
        retry=0,
    ):
        """
        Set servo position in degree
        @servo_channel: Raven.ServoChannel#
        @degree: -90 to 90
        @min_us: servo's minimum pulse in microsecond
        @max_us: servo's maximum pulse in microsecond
        @retry: number of retries if command fails
        @return: True if success
        """
        assert type(servo_channel) == Raven.ServoChannel
        if degree < -90 or degree > 90:
            return ValueError("Invalid degree")

        return self.__write_value(
            Raven.__MessageType.SERVO_VALUE,
            servo_channel,
            struct.pack("H", Raven.__deg_to_count(degree, min_us, max_us)),
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
            Raven.__MessageType.ENCODER_VALUE, motor_channel, retry
        )
        if value and len(value) == 4:
            return struct.unpack("i", value)[0]
        return None

    def set_motor_encoder(self, motor_channel: MotorChannel, value: int, retry=0):
        """
        Set motor encoder count
        @motor_channel: Raven.MotorChannel#
        @value: new value for encoder
        @retry: number of retries if command fails
        @return: number of encoder count or None if fails
        """
        assert type(motor_channel) == Raven.MotorChannel
        return self.__write_value(
            Raven.__MessageType.ENCODER_VALUE,
            motor_channel,
            struct.pack("i", int(value)),
            retry,
        )


if __name__ == "__main__":
    import numpy as np
    import time

    raven = Raven()

    i = 0
    j = 0
    reverse = True
    increase = True
    for channel in Raven.MotorChannel:
        print(raven.set_motor_mode(channel, Raven.MotorMode.DIRECT))
        print(raven.get_motor_mode(channel))
        print(raven.set_motor_torque_factor(channel, 50))
        print(raven.set_motor_encoder(channel, 0))

    while True:
        try:
            if i <= 0:
                increase = True
                reverse = not reverse
                j += 1
            elif i >= 100:
                increase = False

            if increase:
                i += 1
            else:
                i -= 1

            for channel in Raven.MotorChannel:
                raven.set_motor_speed_factor(channel, i, reverse)
                print(channel)
                print(raven.get_motor_encoder(channel))

            pos = i * 0.85
            if reverse:
                pos = -pos

            for channel in Raven.ServoChannel:
                raven.set_servo_position(channel, pos, 500, 2500)

            time.sleep(0.01)
        except KeyboardInterrupt:
            for channel in Raven.MotorChannel:
                print(raven.set_motor_mode(channel, Raven.MotorMode.DISABLED))
                print(raven.get_motor_mode(channel))
            break
