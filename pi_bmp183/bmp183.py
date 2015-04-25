from time import sleep
import RPi.GPIO as GPIO
import spidev


class BMP183Error(Exception):
    pass


class REGISTER():
    AC1 = 0xAA
    AC2 = 0xAC
    AC3 = 0xAE
    AC4 = 0xB0
    AC5 = 0xB2
    AC6 = 0xB4
    B1 = 0xB6
    B2 = 0xB8
    MB = 0xBA
    MC = 0xBC
    MD = 0xBE

    CHIP_ID = 0xD0
    VERSION = 0xD1
    SOFT_RESET = 0xE0
    MEASUREMENT_CONTROL = 0xF4

    ADC_OUT_MSB = 0xF6
    ADC_OUT_LSB = 0xF7
    ADC_OUT_XLSB = 0xF8


class OSS():
    Mode = {0: (0, 0.0045, 1, 'ULTRA_LOW'),
            1: (1, 0.0075, 2, 'STANDARD'),
            2: (2, 0.0135, 4, 'HIGH'),
            3: (3, 0.0255, 8, 'ULTRA_HIGH')}


GPIO_FUNCS = {-1: 'GPIO.UNKNOWN', 0: 'GPIO.OUT', 1: 'GPIO.IN', 10: 'GPIO.BOARD', 11: 'GPIO.BCM',
              40: 'GPIO.SERIAL', 41: "GPIO.SPI", 42: "GPIO.I2C", 43: "GPIO.HARD_PWM"}


class BMP183():
    ID_VALUE = 0x55
    VERSION = 0xD1
    SOFT_RESET = 0xE0
    TEMPERATURE_COMMAND = 0x2E
    TEMPERATURE_WAIT = 0.0045
    PRESSURE_COMMAND = 0x34

    def __init__(self, cs_pin, mosi_pin=0, miso_pin=0, sck_pin=0, oss=3, gpio_mode=GPIO.BOARD):
        self._gpio_mode = gpio_mode
        self._cs_pin = cs_pin
        self._spi_mode = 'HARDWARE'

        if mosi_pin > 0:
            self._spi_mode = 'SOFTWARE'
            self._mosi_pin = mosi_pin
            self._miso_pin = miso_pin
            self._sck_pin = sck_pin

        self._oss = OSS.Mode.get(oss, OSS.Mode[3])
        self._delay = 1 / 1000000.0  # 1MHz - 1 / 7,800,000 7.8MHz
        self._logger = None
        self._UT = 0
        self._UP = 0
        self.temperature_celsius = None
        self.temperature_fahrenheit = None
        self.pressure = None

        self._initialize_interface()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.logger.info('GPIO channel function status:\n'
                         'Clock pin [{0}]: {1}\n'
                         'MOSI pin [{2}]: {3}\n'
                         'MISO pin [{4}]: {5}\n'
                         'CS pin [{6}]: {7}'
                         .format(self._sck_pin, GPIO_FUNCS[GPIO.gpio_function(self._sck_pin)],
                                 self._mosi_pin, GPIO_FUNCS[GPIO.gpio_function(self._mosi_pin)],
                                 self._miso_pin, GPIO_FUNCS[GPIO.gpio_function(self._miso_pin)],
                                 self._cs_pin, GPIO_FUNCS[GPIO.gpio_function(self._cs_pin)]))
        GPIO.cleanup()
        if exc_type is not None:
            self.logger.error('Exception in with block: {0}\n{1}\n{2}'.format(exc_type, exc_val, exc_tb))
            return False

    @property
    def logger(self):
        """
        A :class:`logging.Logger` object for this application.
        """
        if self._logger:
            return self._logger
        from pi_bmp183.logging import create_logger
        self._logger = create_logger(__name__)
        return self._logger

    def _initialize_interface(self):
        GPIO.setmode(self._gpio_mode)
        GPIO.setup(self._sck_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self._cs_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self._mosi_pin, GPIO.OUT)
        GPIO.setup(self._miso_pin, GPIO.IN)

        if not self._communications_check():
            message = 'SPI communication failed, could not retrieve chip ID value.'
            self.logger.error(message)
            raise BMP183Error(message)

        self._read_calibration_data()

    def _communications_check(self):
        return self._read_byte(REGISTER.CHIP_ID) != self.ID_VALUE

    def _read_calibration_data(self):
        self.AC1 = self._read_word(REGISTER.AC1)
        self.AC2 = self._read_word(REGISTER.AC2)
        self.AC3 = self._read_word(REGISTER.AC3)
        self.AC4 = self._read_word(REGISTER.AC4)
        self.AC5 = self._read_word(REGISTER.AC5)
        self.AC6 = self._read_word(REGISTER.AC6)
        self.B1 = self._read_word(REGISTER.B1)
        self.B2 = self._read_word(REGISTER.B2)
        self.MB = self._read_word(REGISTER.MB)
        self.MC = self._read_word(REGISTER.MC)
        self.MD = self._read_word(REGISTER.MD)

    def measure_temperature(self):
        self._write_byte(REGISTER.MEASUREMENT_CONTROL, self.TEMPERATURE_COMMAND)
        sleep(self.TEMPERATURE_WAIT)
        self._UT = self._read_word(REGISTER.ADC_OUT_MSB)
        self._calculate_temperature()

        self.logger.info('Temperature: {0}*C [{1}*F]'.format(self.temperature_celsius, self.temperature_fahrenheit))
        return self.temperature_celsius

    def measure_pressure(self):
        self.measure_temperature()
        self._write_byte(REGISTER.MEASUREMENT_CONTROL, )

    def _calculate_temperature(self):
        x1 = (self._UT - self.AC6) * self.AC5 / 2 ** 15
        x2 = self.MC * 2 ** 11 / (x1 + self.MD)
        self.B5 = x1 + x2
        self.temperature_celsius = round((self.B5 + 8) / 2 ** 4, 2)
        self.temperature_fahrenheit = round((self.temperature_celsius * (9 / 5)) + 32, 2)

    def _calculate_pressure(self):
        # ...
        self.B6 = self.B5 - 4000
        x1 = (self.B2 * (self.B6 ** 2 / 2 ** 12)) / 2 ** 11
        x2 = self.AC2 * self.B6 / 2 ** 11
        x3 = x1 + x2
        self.B3 = ((self.AC1 * 4 + x3) << self._oss[0] + 2) / 4
        x1 = self.AC3 * self.B6 / 2 ** 13
        x2 = (self.B1 * (self.B6 ** 2 / 2 ** 12)) / 2 ** 16
        x3 = ((x1 + x2) + 2) / 2 ** 2
        self.B4 = self.AC4 * (x3 + 32768) / 2 ** 15
        self.B7 = (self._UP - self.B3) * (50000 >> self._oss[0])

        if self.B7 < 0x8000000:
            p = (self.B7 * 2) / self.B4
        else:
            p = (self.B7 / self.B4) * 2

        x1 = (p / 2 ** 8) ** 2
        x1 = (x1 * 3038) / 2 ** 16
        x2 = (-7357 * p) / 2 ** 16
        self.pressure = p + (x1 + x2 + 3791) / 2 ** 4

    def _read_byte(self, register):
        GPIO.output(self._cs_pin, GPIO.LOW)
        control_byte = 0x80 | register
        self._spi_transfer(control_byte)
        value = self._spi_transfer(0x00)
        GPIO.output(self._cs_pin, GPIO.HIGH)

        return value

    def _read_word(self, register):
        GPIO.output(self._cs_pin, GPIO.LOW)
        control_byte = 0x80 | register
        self._spi_transfer(control_byte)
        value = self._spi_transfer(0x00)
        value <<= 8
        value |= self._spi_transfer(0x00)
        GPIO.output(self._cs_pin, GPIO.HIGH)

        return value

    def _write_byte(self, register, value):
        GPIO.output(self._cs_pin, GPIO.LOW)
        control_byte = register & 0x7F
        self._spi_transfer(control_byte)
        self._spi_transfer(value)
        GPIO.output(self._cs_pin, GPIO.HIGH)

    def _spi_transfer(self, data):
        value = 0
        for i in range(8):
            value <<= 1
            GPIO.output(self._sck_pin, GPIO.LOW)
            GPIO.output(self._mosi_pin, data & (1 << (7 - i)))
            GPIO.output(self._sck_pin, GPIO.HIGH)
            if GPIO.input(self._miso_pin):
                value |= 1

        return value


if __name__ == "__main__":
    pass
