# Communication
import busio
import digitalio

# Structure
from ctypes import BigEndianStructure, c_uint16, c_int16, c_uint8, sizeof

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
        FS2000dps = 0, 2000
        FS1000dps = 1, 1000
        FS500dps = 2, 500
        FS250dps = 3, 250
        FS125dps = 4, 125
        FS62dps5 = 5, 62.5
        FS31dps25 = 6, 31.25
        FS15dps625 = 7, 15.625

    @unique
    class ACCEL_FS(Enum):
        FS16g = 0, 16
        FS8g = 1, 8
        FS4g = 2, 4
        FS2g = 3, 2

    @unique
    class GYRO_MODE(Enum):
        OFF = 0
        STANDBY = 1
        LOWNOISE = 3

    @unique
    class ACCEL_MODE(Enum):
        OFF = 0
        LOWPOWER = 2
        LOWNOISE = 3

    @unique
    class UI_FILT_ORD(Enum):
        ORD1st = 0
        ORD2nd = 1
        ORD3rd = 2

    __REG_FILE = "icm42688reg.yaml"
    __DEFAULT_ID = 0x47

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
        return type(name, (register_type,), {"_fields_": fields})(
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
        self.PWR_MGMT0.GYRO_MODE = ICM42688.GYRO_MODE.LOWNOISE.value
        self.PWR_MGMT0.ACCEL_MODE = ICM42688.ACCEL_MODE.LOWNOISE.value
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

    def set_gyro_mode(self, mode: GYRO_MODE):
        """
        Set gyro operation mode
        @mode: Operation mode (OFF, STANDBY, LOWNOISE)
        """
        self.PWR_MGMT0.GYRO_MODE = mode.value
        self.PWR_MGMT0.write()
        time.sleep(0.1)

    def set_accel_mode(self, mode: ACCEL_MODE):
        """
        Set accel operation mode
        @mode: Operation mode (OFF, LOWPOWER, LOWNOISE)
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
        return self.accel_fullscale.value[1] / 32767.5 * accel_data * 9.80665

    def __gyro_data_to_gyro(self, gyro_data):
        return self.gyro_fullscale.value[1] / 32767.5 * gyro_data * 0.017453

    def get_data(self):
        """
        Return data in order x,y,z of acceleration(m/s) and rotation(rad/s)
        """
        self.IMU_DATA.read()
        accel = (
            self.__accel_data_to_accel(self.IMU_DATA.ACCEL_DATA_X),
            self.__accel_data_to_accel(self.IMU_DATA.ACCEL_DATA_Y),
            self.__accel_data_to_accel(self.IMU_DATA.ACCEL_DATA_Z),
        )
        gyro = (
            self.__gyro_data_to_gyro(self.IMU_DATA.GYRO_DATA_X),
            self.__gyro_data_to_gyro(self.IMU_DATA.GYRO_DATA_Y),
            self.__gyro_data_to_gyro(self.IMU_DATA.GYRO_DATA_Z),
        )
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

    while True:
        print(imu.get_data())
        time.sleep(0.01)
