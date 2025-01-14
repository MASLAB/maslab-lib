# Communication
import busio
import digitalio

# Structure
from ctypes import BigEndianStructure, c_uint16, c_int16, c_uint8, c_int8, sizeof

# Math
import numpy as np

# Register parsing
import pathlib
import yaml

# General
import time
import warnings
from enum import Enum, unique


class ICM42688:

    @unique
    class ODR(Enum):
        (
            ODR32kHz,
            ODR16kHz,
            ODR8kHz,
            ODR4kHz,
            ODR2kHz,
            ODR1kHz,
            ODR200Hz,
            ODR100Hz,
            ODR50Hz,
            ODR25Hz,
            ODR12Hz5,
        ) = range(1, 12)
        ODR500Hz = 15

    @unique
    class GYRO_FS(Enum):
        FS2000dps = 0, 0.017453 / 16.4
        FS1000dps = 1, 0.017453 / 32.8
        FS500dps = 2, 0.017453 / 65.5
        FS250dps = 3, 0.017453 / 131
        FS125dps = 4, 0.017453 / 262
        FS62dps5 = 5, 0.017453 / 524.3
        FS31dps25 = 6, 0.017453 / 1048.6
        FS15dps625 = 7, 0.017453 / 2097.2

    @unique
    class ACCEL_FS(Enum):
        FS16g = 0, 9.80665 / 2048
        FS8g = 1, 9.80665 / 4096
        FS4g = 2, 9.80665 / 8192
        FS2g = 3, 9.80665 / 16384

    @unique
    class GYRO_MODE(Enum):
        OFF = 0
        STANDBY = 1
        LOW_NOISE = 3

    @unique
    class ACCEL_MODE(Enum):
        OFF = 0
        LOW_POWER = 2
        LOW_NOISE = 3

    @unique
    class UI_FILT_ORD(Enum):
        ORD1st = 0
        ORD2nd = 1
        ORD3rd = 2

    @unique
    class FIFO_MODE(Enum):
        BYPASS = 0
        STREAM_TO_FIFO = 1
        STOP_ON_FULL = 2

    __REG_FILE = "icm42688reg.yaml"
    __DEFAULT_ID = 0x47

    __AXES = ["X", "Y", "Z"]

    class __SpiDev:
        __REG_BANK_SEL = 0x76

        class __EmptyCS:
            def on(self):
                pass

            def off(self):
                pass

        def __init__(self, spi: busio.SPI, cs=None):
            self.__spi = spi
            self.__cs = self.__EmptyCS()
            if cs:
                self.__cs = digitalio.DigitalInOut(cs)
                self.__cs.direction = digitalio.Direction.OUTPUT
                self.__cs.pull = digitalio.Pull.UP
            self.__cs.on()
            self.__bank = 5

        def __write_reg(self, reg, data: bytearray):
            self.__cs.off()
            for i in range(len(data)):
                write_reg = [reg + i, data[i]]
                self.__spi.write(write_reg)
            self.__cs.on()

        def __read_reg(self, reg, length):
            result = bytearray(length + 1)
            read_reg = [reg | 0x80] + [0] * length
            self.__cs.off()
            self.__spi.write_readinto(read_reg, result)
            self.__cs.on()
            return result[1:]

        def __set_bank(self, bank):
            if self.__bank != bank:
                self.__bank = bank
                self.__write_reg(self.__REG_BANK_SEL, bytearray([bank]))
            new_bank = self.__get_bank()
            if new_bank != self.__bank:
                raise RuntimeError("Failed to set register bank")

        def __get_bank(self):
            return self.__read_reg(self.__REG_BANK_SEL, 1)[0]

        def write_bank_reg(self, bank, reg, data: bytearray):
            self.__set_bank(bank)
            self.__write_reg(reg, data)

        def read_bank_reg(self, bank, reg, length):
            self.__set_bank(bank)
            return self.__read_reg(reg, length)

    class Register(BigEndianStructure):
        def __init__(self, bank, address, spidev, *args, **kw):
            object.__setattr__(self, "_bank", bank)
            object.__setattr__(self, "_address", address)
            object.__setattr__(self, "_spidev", spidev)
            object.__setattr__(self, "field_names", dict())
            super().__init__(*args, **kw)
            field_names = object.__getattribute__(self, "field_names")
            for field in self._fields_:
                field_names[field[0]] = None
            object.__setattr__(self, "field_names", field_names)

        def __repr__(self):
            repr_string = ""
            for name in self.field_names:
                repr_string += "%s: %s,\n" % (name, object.__getattribute__(self, name))
            return repr_string

    class ReadRegister(Register):
        def read(self):
            result = self._spidev.read_bank_reg(self._bank, self._address, sizeof(self))
            read_values = self.__class__.from_buffer_copy(result)
            for name in self.field_names:
                value = getattr(read_values, name)
                super(ICM42688.Register, self).__setattr__(name, value)

        def __setattr__(self, name, value):
            warnings.warn("Writing to read-only register", RuntimeWarning)

    class WriteRegister(Register):
        def write(self):
            self._spidev.write_bank_reg(self._bank, self._address, bytearray(self))

        def __getattribute__(self, name):
            if name in object.__getattribute__(self, "field_names"):
                warnings.warn("Reading write-only register", RuntimeWarning)
            else:
                return super(ICM42688.Register, self).__getattribute__(name)

    class ReadWriteRegister(WriteRegister, ReadRegister):
        def write(self):
            super().write()
            self.read()

        def __setattr__(self, name, value):
            return super(ICM42688.Register, self).__setattr__(name, value)

        def __getattribute__(self, name):
            return super(ICM42688.Register, self).__getattribute__(name)

    class FIFOHeader(BigEndianStructure):
        _fields_ = [
            ("HEADER_MSG", c_uint8, 1),
            ("HEADER_ACCEL", c_uint8, 1),
            ("HEADER_GYRO", c_uint8, 1),
            ("HEADER_20", c_uint8, 1),
            ("HEADER_TIMESTAMP_FSYNC", c_uint8, 2),
            ("HEADER_ODR_ACCEL", c_uint8, 1),
            ("HEADER_ODR_GYRO", c_uint8, 1),
        ]

    class FIFOPacket(BigEndianStructure):
        def __repr__(self):
            repr_string = ""
            for field in self._fields_:
                name = field[0]
                repr_string += "%s: %s,\n" % (name, object.__getattribute__(self, name))
            return repr_string

    class FIFOPacket1(FIFOPacket):
        _pack_ = 1
        _fields_ = [
            ("FIFO_HEADER", c_uint8, 8),
            ("ACCEL_DATA_X", c_int16, 16),
            ("ACCEL_DATA_Y", c_int16, 16),
            ("ACCEL_DATA_Z", c_int16, 16),
            ("TEMP_DATA", c_int8, 8),
        ]

        def __init__(self, *args, **kw):
            super().__init__(*args, **kw)

    class FIFOPacket2(FIFOPacket):
        _pack_ = 1
        _fields_ = [
            ("FIFO_HEADER", c_uint8, 8),
            ("GYRO_DATA_X", c_int16, 16),
            ("GYRO_DATA_Y", c_int16, 16),
            ("GYRO_DATA_Z", c_int16, 16),
            ("TEMP_DATA", c_int8, 8),
        ]

        def __init__(self, *args, **kw):
            super().__init__(*args, **kw)

    class FIFOPacket3(FIFOPacket):
        _pack_ = 1
        _fields_ = [
            ("FIFO_HEADER", c_uint8, 8),
            ("ACCEL_DATA_X", c_int16, 16),
            ("ACCEL_DATA_Y", c_int16, 16),
            ("ACCEL_DATA_Z", c_int16, 16),
            ("GYRO_DATA_X", c_int16, 16),
            ("GYRO_DATA_Y", c_int16, 16),
            ("GYRO_DATA_Z", c_int16, 16),
            ("TEMP_DATA", c_int8, 8),
            ("TIMESTAMP", c_uint16, 16),
        ]

        def __init__(self, *args, **kw):
            super().__init__(*args, **kw)

    class FIFOPacket4(FIFOPacket):
        _pack_ = 1
        _fields_ = [
            ("FIFO_HEADER", c_uint8, 8),
            ("ACCEL_DATA_X_UPPER", c_int16, 16),
            ("ACCEL_DATA_Y_UPPER", c_int16, 16),
            ("ACCEL_DATA_Z_UPPER", c_int16, 16),
            ("GYRO_DATA_X_UPPER", c_int16, 16),
            ("GYRO_DATA_Y_UPPER", c_int16, 16),
            ("GYRO_DATA_Z_UPPER", c_int16, 16),
            ("TEMP_DATA", c_int16, 16),
            ("TIMESTAMP", c_uint16, 16),
            ("ACCEL_DATA_X_LOWER", c_uint8, 4),
            ("GYRO_DATA_X_LOWER", c_uint8, 4),
            ("ACCEL_DATA_Y_LOWER", c_uint8, 4),
            ("GYRO_DATA_Y_LOWER", c_uint8, 4),
            ("ACCEL_DATA_Z_LOWER", c_uint8, 4),
            ("GYRO_DATA_Z_LOWER", c_uint8, 4),
        ]

        def __init__(self, *args, **kw):
            super().__init__(*args, **kw)

    __REG_TYPES = {
        "R": ReadRegister,
        "W": WriteRegister,
        "R/W": ReadWriteRegister,
    }

    def __init__(self, spi: busio.SPI, cs=None):
        self.__spidev = ICM42688.__SpiDev(spi, cs)
        self.__parse_register()

    def __create_register(
        self,
        name,
        address,
        register_type,
        bank,
        fields,
        init_values=[],
    ):
        return type(name, (register_type,), {"_fields_": fields, "_pack_": 1})(
            bank, address, self.__spidev, *init_values
        )

    def __parse_register(self):
        self.__registers = []
        file_path = pathlib.Path(__file__).parent.resolve() / ICM42688.__REG_FILE
        with open(file_path, "r") as file:
            banks = yaml.safe_load(file)

        bank_num = 0
        for bank in banks:
            if bank is not None:
                for register in bank:
                    name = register["name"]
                    address = register["address"]
                    register_type = ICM42688.__REG_TYPES[register["type"]]
                    fields, init_values = ICM42688.__parse_fields(register["fields"])
                    if register_type == ICM42688.ReadRegister:
                        init_values = []

                    new_register = self.__create_register(
                        name, address, register_type, bank_num, fields, init_values
                    )

                    setattr(
                        self,
                        name,
                        new_register,
                    )
                    self.__registers.append(new_register)
            bank_num += 1

    @staticmethod
    def __parse_fields(field_settings: dict[dict]):
        fields = list()
        init_values = list()
        for field in field_settings:
            # C struct fields
            name = field["name"]
            data_type = eval(field["type"])
            length = field["length"]
            fields.append((name, data_type, length))

        return fields, init_values

    def begin(self):
        """
        Begin communication with the sensor and initialize the sensor settings to:
        Gyro + accel UI filter: 3rd order
        Gyro + accel mode: Low Noise
        Gyro fullscale: +/-2000dps ODR: 1kHz
        Accel fullscale: +/-16g ODR: 1kHz
        Calibrate gyro offsets
        """
        # Check ID
        try:
            self.WHO_AM_I.read()
            id = self.WHO_AM_I.WHOAMI
            if id != ICM42688.__DEFAULT_ID:
                raise RuntimeError("Device ID not matched")
        except RuntimeError:
            raise RuntimeError("Device not connected")

        # Reset device
        self.reset()

        # Set filter order to 3rd order
        self.GYRO_CONFIG1.GYRO_UI_FILT_ORD = ICM42688.UI_FILT_ORD.ORD3rd.value
        self.GYRO_CONFIG1.write()
        self.ACCEL_CONFIG1.ACCEL_UI_FILT_ORD = ICM42688.UI_FILT_ORD.ORD3rd.value
        self.ACCEL_CONFIG1.write()

        # Turn on sensors
        self.PWR_MGMT0.GYRO_MODE = ICM42688.GYRO_MODE.LOW_NOISE.value
        self.PWR_MGMT0.ACCEL_MODE = ICM42688.ACCEL_MODE.LOW_NOISE.value
        self.PWR_MGMT0.write()
        time.sleep(0.1)

        # Set fullscale and ODR
        self.set_gyro_fullscale_odr(ICM42688.GYRO_FS.FS2000dps, ICM42688.ODR.ODR1kHz)
        self.set_accel_fullscale_odr(ICM42688.ACCEL_FS.FS16g, ICM42688.ODR.ODR1kHz)

        # Calibrate gyro
        self.calibrate_gyro()

    def reset(self):
        """
        Reset all settings to defaults
        """
        self.DEVICE_CONFIG.SOFT_RESET_CONFIG = 1
        self.DEVICE_CONFIG.write()
        time.sleep(0.1)
        # Read registers
        for register in self.__registers:
            try:
                register.read()
            except AttributeError:
                pass

    def __sensors_off(self):
        self.prev_gyro_mode = self.PWR_MGMT0.GYRO_MODE
        self.prev_accel_mode = self.PWR_MGMT0.ACCEL_MODE
        self.PWR_MGMT0.GYRO_MODE = ICM42688.GYRO_MODE.OFF.value
        self.PWR_MGMT0.ACCEL_MODE = ICM42688.GYRO_MODE.OFF.value
        self.PWR_MGMT0.write()
        time.sleep(0.1)

    def __sensors_resume(self):
        self.PWR_MGMT0.GYRO_MODE = self.prev_gyro_mode
        self.PWR_MGMT0.ACCEL_MODE = self.prev_accel_mode
        self.PWR_MGMT0.write()
        time.sleep(0.1)

    def set_fifo_mode(self, mode: FIFO_MODE):
        """
        Set FIFO mode
        @mode: FIFO mode (BYPASS, STREAM_TO_FIFO, STOP_ON_FULL)
        """
        # Statically enable temp, gyro, and acceleration for now
        # Need to support timestamp and high resolution later
        if mode != ICM42688.FIFO_MODE.BYPASS:
            self.FIFO_CONFIG1.FIFO_TEMP_EN = 1
            self.FIFO_CONFIG1.FIFO_GYRO_EN = 1
            self.FIFO_CONFIG1.FIFO_ACCEL_EN = 1
            self.FIFO_CONFIG1.write()
        self.FIFO_CONFIG.FIFO_MODE = mode.value
        self.FIFO_CONFIG.write()

    def get_fifo_length(self):
        self.FIFO_COUNT.read()
        return self.FIFO_COUNT.FIFO_COUNT

    def get_fifo_data(self):
        length = self.get_fifo_length()
        data = self.__spidev.read_bank_reg(0, 0x30, length)
        return self.__parse_fifo_data(data)

    def __parse_fifo_data(self, data):
        data_length = len(data)
        parsed_data = list()
        i = 0
        while i < data_length:
            accel = None
            gyro = None
            temperature = None
            time_stamp = None

            header = ICM42688.FIFOHeader.from_buffer(data, i)
            # Packet 4
            if header.HEADER_20:
                package_datum = ICM42688.FIFOPacket4.from_buffer(data, i)
                i += sizeof(package_datum)

            # Packet 3
            elif header.HEADER_TIMESTAMP_FSYNC or (
                header.HEADER_ACCEL and header.HEADER_GYRO
            ):
                package_datum = ICM42688.FIFOPacket3.from_buffer(data, i)
                accel = [
                    self.__accel_data_to_accel(
                        getattr(package_datum, "ACCEL_DATA_" + axis)
                    )
                    for axis in ICM42688.__AXES
                ]
                gyro = [
                    self.__gyro_data_to_gyro(
                        getattr(package_datum, "GYRO_DATA_" + axis)
                    )
                    for axis in ICM42688.__AXES
                ]
                temperature = ICM42688.__temp_data_8_to_temp(package_datum.TEMP_DATA)
                time_stamp = package_datum.TIMESTAMP / 1000000

                i += sizeof(package_datum)

            # Packet 2
            elif header.HEADER_GYRO:
                package_datum = ICM42688.FIFOPacket2.from_buffer(data, i)
                i += sizeof(package_datum)

            # Packet 1
            elif header.HEADER_ACCEL:
                package_datum = ICM42688.FIFOPacket1.from_buffer(data, i)
                i += sizeof(package_datum)

            else:
                i += 1
                continue

            # print(package_datum)
            # print(sizeof(package_datum))

            parsed_data.append((accel, gyro, temperature, time_stamp))

        return parsed_data

    def set_gyro_mode(self, mode: GYRO_MODE):
        """
        Set gyro operation mode
        @mode: Operation mode (OFF, STANDBY, LOW_NOISE)
        """
        self.PWR_MGMT0.GYRO_MODE = mode.value
        self.PWR_MGMT0.write()
        time.sleep(0.1)

    def set_accel_mode(self, mode: ACCEL_MODE):
        """
        Set accel operation mode
        @mode: Operation mode (OFF, LOW_POWER, LOW_NOISE)
        """
        self.PWR_MGMT0.ACCEL_MODE = mode.value
        self.PWR_MGMT0.write()
        time.sleep(0.1)

    def set_gyro_fullscale_odr(self, fullscale: GYRO_FS, odr: ODR):
        """
        Set gyro fullscale and output data rate
        @fullscale: +/- 15.625-2000dps
        @odr: 12.5Hz - 32kHz
        """
        self.gyro_fullscale = fullscale
        self.gyro_odr = odr
        self.GYRO_CONFIG0.GYRO_FS_SEL = fullscale.value[0]
        self.GYRO_CONFIG0.GYRO_ODR = odr.value
        self.GYRO_CONFIG0.write()
        time.sleep(0.1)

    def set_accel_fullscale_odr(self, fullscale: ACCEL_FS, odr: ODR):
        """
        Set accel fullscale and output data rate
        @fullscale: +/- 2-16g
        @odr: 12.5Hz - 32kHz
        """
        self.accel_fullscale = fullscale
        self.accel_odr = odr
        self.ACCEL_CONFIG0.ACCEL_FS_SEL = fullscale.value[0]
        self.ACCEL_CONFIG0.ACCEL_ODR = odr.value
        self.ACCEL_CONFIG0.write()
        time.sleep(0.1)

    def set_gyro_ui_filter_order(self, order: UI_FILT_ORD):
        """
        Set gyro filter order
        @order: 1st, 2nd, 3rd
        """
        self.__sensors_off()
        self.GYRO_CONFIG1.GYRO_UI_FILT_ORD = order.value
        self.GYRO_CONFIG1.write()
        self.__sensors_resume()

    def set_accel_ui_filter_order(self, order: UI_FILT_ORD):
        """
        Set accel filter order
        @order: 1st, 2nd, 3rd
        """
        self.__sensors_off()
        self.ACCEL_CONFIG1.ACCEL_UI_FILT_ORD = order.value
        self.ACCEL_CONFIG1.write()
        self.__sensors_resume()

    def __accel_data_to_accel(self, accel_data):
        return self.accel_fullscale.value[1] * accel_data

    def __gyro_data_to_gyro(self, gyro_data):
        return self.gyro_fullscale.value[1] * gyro_data

    @staticmethod
    def __temp_data_8_to_temp(temp_data):
        return (temp_data / 2.07) + 25

    @staticmethod
    def __temp_data_16_to_temp(temp_data):
        return (temp_data / 132.48) + 25

    def get_data(self):
        """
        Return data in order x,y,z of acceleration(m/s) and rotation(rad/s)
        """
        self.IMU_DATA.read()
        accel = [
            self.__accel_data_to_accel(getattr(self.IMU_DATA, "ACCEL_DATA_" + axis))
            for axis in ICM42688.__AXES
        ]
        gyro = [
            self.__gyro_data_to_gyro(getattr(self.IMU_DATA, "GYRO_DATA_" + axis))
            for axis in ICM42688.__AXES
        ]
        return accel, gyro

    __GYRO_OFFSET_RES = 1 / 32

    def __set_gyro_offset(self, axis: str):
        gyro_data_reg = getattr(self.IMU_DATA, "GYRO_DATA_" + axis)
        gyro = -self.__gyro_data_to_gyro(gyro_data_reg) / 0.017453
        gyro_offset = np.int16(gyro / ICM42688.__GYRO_OFFSET_RES)
        base_register_name = "GYRO_%s_OFFUSER_" % axis
        setattr(self.OFFSET_USER, base_register_name + "UPPER", gyro_offset >> 8)
        setattr(self.OFFSET_USER, base_register_name + "LOWER", gyro_offset & 0xFF)

    def calibrate_gyro(self):
        """
        Calibrate gyro offset
        """
        self.OFFSET_USER.read()
        prev_gyro_fullscale = self.gyro_fullscale
        prev_gyro_odr = self.gyro_odr
        self.set_gyro_fullscale_odr(ICM42688.GYRO_FS.FS15dps625, ICM42688.ODR.ODR1kHz)
        time.sleep(0.1)
        self.IMU_DATA.read()
        self.__sensors_off()

        self.__set_gyro_offset("X")
        self.__set_gyro_offset("Y")
        self.__set_gyro_offset("Z")

        self.OFFSET_USER.write()
        self.__sensors_resume()
        self.set_gyro_fullscale_odr(prev_gyro_fullscale, prev_gyro_odr)


if __name__ == "__main__":
    import board

    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

    while not spi.try_lock():
        pass

    spi.configure(baudrate=5000000)

    imu = ICM42688(spi)
    imu.begin()

    imu.set_gyro_fullscale_odr(ICM42688.GYRO_FS.FS15dps625, ICM42688.ODR.ODR1kHz)
    imu.set_accel_fullscale_odr(ICM42688.ACCEL_FS.FS2g, ICM42688.ODR.ODR1kHz)

    imu.set_fifo_mode(ICM42688.FIFO_MODE.STREAM_TO_FIFO)

    while True:
        for datum in imu.get_fifo_data():
            print(datum)
        time.sleep(2)
