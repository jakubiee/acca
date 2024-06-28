import time
import rp2
from onewire import OneWire  # type: ignore
from ds18x20 import DS18X20  # type: ignore
from machine import I2C, PWM, SoftSPI, Pin as MachinePin  # type: ignore
from typing import Literal, Optional
from typing_extensions import Annotated, Doc


class Pin:
    """
    Pin class, a simple class to emulate the machine.Pin.

    Attributes:
        OUT (int): Output mode.
        IN (int): Input mode.

    Example:
        led = Pin(2, Pin.OUT, 1)
    """

    OUT = MachinePin.OUT
    IN = MachinePin.IN

    def __init__(
        self,
        pin: Annotated[int, Doc("Pin number")],
        mode: Optional[Annotated[int, Doc("Pin mode (input or output)")]] = None,
        value: Optional[Annotated[int, Doc("Initial value of the pin")]] = None,
    ) -> None:
        """
        Initializes the Pin instance.

        Args:
            pin (int): Pin number.
            mode (int, optional): Pin mode (input or output). Defaults to None.
            value (int, optional): Initial value of the pin. Defaults to None.
        """
        self._pin = MachinePin(pin)
        if mode is not None:
            self.mode(mode)
        if value is not None:
            self.value(value)

    def mode(self, mode: Optional[int] = None) -> "Pin":
        """
        Sets or gets the mode of the pin.

        Args:
            mode (int, optional): Pin mode to set (input or output). Defaults to None.

        Returns:
            Pin: The Pin instance for method chaining.

        Raises:
            ValueError: If the mode is not valid.
        """
        if mode is None:
            return self._pin.mode()  # type: ignore
        if mode not in [MachinePin.IN, MachinePin.OUT]:
            raise ValueError("Invalid mode")
        self._pin.mode(mode)  # type: ignore
        return self

    def value(self, val: Optional[int] = None) -> "Pin":
        """
        Sets or gets the value of the pin.

        Args:
            val (int, optional): Value to set (0 or 1). Defaults to None.

        Returns:
            Pin: The Pin instance for method chaining.

        Raises:
            TypeError: If the value is not an integer.
        """
        if val is None:
            return self._pin.value()  # type: ignore
        if not isinstance(val, int):
            raise TypeError("Pin value must be an integer")
        self._pin.value(val)  # type: ignore
        return self


class I2C_ACCA:
    """
    A class to interface with I2C devices.

    Example:
        i2c = I2C_ACCA(0, Pin(8), Pin(9), 400000)
        i2c.send(b"Hello")
    """

    def __init__(self, id: int, sda_pin: Pin, scl_pin: Pin, baudrate: int) -> None:
        """
        Initializes the I2C_ACCA instance.

        Args:
            id (int): The I2C bus ID.
            sda_pin (Pin): The pin used for SDA.
            scl_pin (Pin): The pin used for SCL.
            baudrate (int): The communication baud rate.
        """
        self.id = id
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.baudrate = baudrate
        self.i2c = I2C(id, sda=sda_pin, scl=scl_pin, freq=baudrate)
        self.addr = self._get_addr()

    def _get_addr(self):
        """
        Scans for I2C devices and returns the address of the first device found.

        Returns:
            int: The address of the first I2C device found.

        Raises:
            Exception: If no device is found.
        """
        addr = self.i2c.scan()
        if not addr:
            raise Exception("No device found")
        return addr[0]

    def send(self, data, stop=True):
        """
        Sends data to the I2C device.

        Args:
            data (bytes): The data to send.
            stop (bool, optional): Whether to send a stop condition after the data. Defaults to True.

        Returns:
            int: The result of the writeto operation.

        Raises:
            Exception: If there is an error sending data.
        """
        try:
            return self.i2c.writeto(self.addr, data, stop)
        except Exception as e:
            print(f"Error sending data: {e}")
            raise

    def read(self, nbytes, stop=True):
        """
        Reads data from the I2C device.

        Args:
            nbytes (int): The number of bytes to read.
            stop (bool, optional): Whether to send a stop condition after reading. Defaults to True.

        Returns:
            bytes: The data read from the I2C device.

        Raises:
            Exception: If there is an error reading data.
        """
        try:
            data = self.i2c.readfrom(self.addr, nbytes, stop)
            return data
        except Exception as e:
            print(f"Error reading data: {e}")
            raise

    def show_parameters(self) -> None:
        """
        Prints the I2C parameters.
        """
        print(
            f"ID: {self.id}\n"
            f"Frequency: {self.baudrate}\n"
            f"SCL pin: {self.scl_pin}\n"
            f"SDA pin: {self.sda_pin}\n"
            f"Device address: {self.addr}"
        )


class SPI_ACCA:
    """
    A class to interface with SPI devices.

    Args:
        key (int): The SPI bus key.
        baudrate (int): The communication baud rate.
        sck (Pin): The pin used for SCK.
        mosi (Pin): The pin used for MOSI.
        miso (Pin): The pin used for MISO.
        cs (Pin): The pin used for CS.
    """

    def __init__(
        self, key: int, baudrate: int, sck: Pin, mosi: Pin, miso: Pin, cs: Pin
    ):
        self.key = key
        self.baudrate = baudrate
        self.sck = sck
        self.mosi = mosi
        self.miso = miso
        self.cs = cs
        self.cs.mode(Pin.OUT).value

        self.spi = SoftSPI(
            baudrate=baudrate,
            sck=self.sck,
            mosi=self.mosi,
            miso=self.miso,
            polarity=0,
            phase=0,
        )

    def show_parameters(self) -> None:
        """
        Prints the SPI parameters.
        """
        print(
            f"Key: {self.key}\n"
            f"Frequency: {self.baudrate}\n"
            f"SCK pin: {self.sck}\n"
            f"MOSI pin: {self.mosi}\n"
            f"MISO pin: {self.miso}\n"
            f"CS pin: {self.cs}"
        )


class AD8821(I2C_ACCA):
    """
    A class to interface with the AD8821 digital potentiometer via I2C.

    Example:
        ad8821 = AD8821(1, Pin(3), Pin(2), 100000)
        ad8821.set_value(0x12, 0x34)
    """

    def __init__(self, id: int = 1, sda_pin: Pin = Pin(3), scl_pin: Pin = Pin(2), baudrate: int = 100000):
        """
        Initializes the AD8821 instance.

        Args:
            id (int, optional): The I2C bus ID. Defaults to 1.
            sda_pin (Pin, optional): The pin used for SDA. Defaults to Pin(3).
            scl_pin (Pin, optional): The pin used for SCL. Defaults to Pin(2).
            baudrate (int, optional): The communication baud rate. Defaults to 100000.
        """
        super().__init__(id, sda_pin, scl_pin, baudrate)

    def set_value(self, MSB: int, LSB: int):
        """
        Sets the value of the AD8821 digital potentiometer.

        Args:
            MSB (int): The most significant byte of the value.
            LSB (int): The least significant byte of the value.

        Returns:
            int: The result of the send operation.

        Raises:
            Exception: If there is an error setting the value.
        """
        try:
            return self.send(bytearray([MSB, LSB]))
        except Exception as e:
            print(f"Failed to set value: {e}")


class AD5282(I2C_ACCA):
    """
    A class to interface with the AD5282 digital potentiometer via I2C.

    Args:
        id (int, optional): The I2C bus ID. Defaults to 1.
        sda_pin (Pin, optional): The pin used for SDA. Defaults to Pin(7).
        scl_pin (Pin, optional): The pin used for SCL. Defaults to Pin(6).
        baudrate (int, optional): The communication baud rate. Defaults to 100000.
    """

    def __init__(self, id: int = 1, sda_pin: Pin = Pin(7), scl_pin: Pin = Pin(6), baudrate: int = 100000):
        super().__init__(id, sda_pin, scl_pin, baudrate)

    def set_value(self, MSB: int, LSB: int):
        """
        Sets the value of the AD5282 digital potentiometer.

        Args:
            MSB (int): The most significant byte of the value.
            LSB (int): The least significant byte of the value.

        Returns:
            int: The result of the send operation.

        Raises:
            Exception: If there is an error setting the value.
        """
        try:
            return self.send(bytearray([MSB, LSB]))
        except Exception as e:
            print(f"Failed to set value: {e}")


class MCP4706_I2C(I2C_ACCA):
    """
    A class to interface with the MCP4706 digital-to-analog converter via I2C.

    Args:
        id (int, optional): The I2C bus ID. Defaults to 1.
        sda_pin (Pin, optional): The pin used for SDA. Defaults to Pin(19).
        scl_pin (Pin, optional): The pin used for SCL. Defaults to Pin(18).
        baudrate (int, optional): The communication baud rate. Defaults to 100000.
    """

    def __init__(self, id: int = 1, sda_pin: Pin = Pin(19), scl_pin: Pin = Pin(18), baudrate: int = 100000):
        super().__init__(id=id, sda_pin=sda_pin, scl_pin=scl_pin, baudrate=baudrate)

    def set_value(self, MSB: int, LSB: int):
        """
        Sets the value of the MCP4706 digital-to-analog converter.

        Args:
            MSB (int): The most significant byte of the value.
            LSB (int): The least significant byte of the value.

        Returns:
            int: The result of the send operation.

        Raises:
            Exception: If there is an error setting the value.
        """
        try:
            return self.send(bytearray([MSB, LSB]))
        except Exception as e:
            print(f"Failed to set value: {e}")


class PWM_ACCA(PWM):
    """
    A class to interface with PWM devices.

    Args:
        pin (Pin, optional): The pin used for PWM. Defaults to Pin(17).
        freq (int, optional): The PWM frequency. Defaults to 1000.
        duty (int, optional): The PWM duty cycle. Defaults to 32768.
    """

    def __init__(self, pin: Pin = Pin(17), freq: int = 1000, duty: int = 32768):
        self.pin = pin
        self.pwm = PWM(pin)
        self.pwm.freq(freq)
        self.pwm.duty_u16(duty)


class DS18B20_OneWire:
    """
    A class to interface with the DS18B20 temperature sensor via OneWire.

    Args:
        pin (Pin): The pin used for OneWire communication.
    """

    def __init__(self, pin: Pin):
        self.onewire = OneWire(pin)
        self.ds = DS18X20(self.onewire)
        self.addr = self.scan()

    def scan(self):
        """
        Scans for OneWire devices and returns the address of the first device found.

        Returns:
            bytes: The address of the first OneWire device found.

        Raises:
            Exception: If no device is found.
        """
        addresses = self.onewire.scan()
        if not addresses:
            raise Exception("No device found")
        return addresses[0]

    def read_temp(self, with_scratch: bool = False):
        """
        Reads the temperature from the DS18B20 sensor.

        Args:
            with_scratch (bool, optional): Whether to read the scratchpad data. Defaults to False.

        Returns:
            float or list: The temperature value or a list with temperature and scratchpad data.
        """
        self.ds.convert_temp()
        time.sleep_ms(750)  # type: ignore
        if with_scratch:
            return [self.ds.read_temp(self.addr), self.ds.read_scratch(self.addr)]
        return self.ds.read_temp(self.addr)


class MCP3461_SPI(SPI_ACCA):
    """
    A class to interface with the MCP3461 analog-to-digital converter via SPI.

    Args:
        key (int, optional): The SPI bus key. Defaults to 0.
        baudrate (int, optional): The communication baud rate. Defaults to 1000000.
        sck (Pin, optional): The pin used for SCK. Defaults to Pin(2).
        mosi (Pin, optional): The pin used for MOSI. Defaults to Pin(3).
        miso (Pin, optional): The pin used for MISO. Defaults to Pin(4).
        cs (Pin, optional): The pin used for CS. Defaults to Pin(5).
    """

    def __init__(
        self,
        key: int = 0,
        baudrate: int = 1000000,
        sck: Pin = Pin(2),
        mosi: Pin = Pin(3),
        miso: Pin = Pin(4),
        cs: Pin = Pin(5),
    ):
        super().__init__(key, baudrate, sck, mosi, miso, cs)

    def write(self, message: list):
        """
        Writes a message to the MCP3461 via SPI.

        Args:
            message (list): The message to write.
        """
        self.cs.value(0)
        self.spi.write(bytearray(message))
        self.cs.value(1)
        time.sleep_us(1)  # type: ignore

    def read(self, sample: int):
        """
        Reads data from the MCP3461 via SPI.

        Args:
            sample (int): The number of samples to read.

        Returns:
            list: The data read from the MCP3461.
        """
        self.cs.value(0)
        data = []
        for i in range(sample):
            data.append(self.spi.read(4))
            time.sleep_us(1)  # type: ignore
        return data


class AD7801_Parallel:
    """
    A class to interface with the AD7801 digital-to-analog converter via parallel interface.

    Args:
        b0-b7 (int): Pins for data bits (8 pins).
        wr_pin (int): Pin used for the WR signal.
        freq (int, optional): Frequency for the state machine. Defaults to 10000000.
    """

    def __init__(
        self,
        b0: int = 6,
        b1: int = 7,
        b2: int = 8,
        b3: int = 9,
        b4: int = 10,
        b5: int = 11,
        b6: int = 12,
        b7: int = 13,
        wr_pin: int = 14,
        freq: int = 10000000
    ):
        self.freq = freq
        self.data_pins = [
            Pin(b0, Pin.OUT),
            Pin(b1, Pin.OUT),
            Pin(b2, Pin.OUT),
            Pin(b3, Pin.OUT),
            Pin(b4, Pin.OUT),
            Pin(b5, Pin.OUT),
            Pin(b6, Pin.OUT),
            Pin(b7, Pin.OUT)
        ]
        self.wr_pin = Pin(wr_pin, Pin.OUT)

        @rp2.asm_pio(set_init=(rp2.PIO.OUT_HIGH, rp2.PIO.OUT_HIGH),
                     out_init=(rp2.PIO.OUT_LOW,) * 8, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
        def smproc():
            wrap_target()
            pull()
            out(pins, 8)
            set(pins, 0b00) [5]
            set(pins, 0b11)
            wrap()

        self.sm = rp2.StateMachine(0, smproc, freq=freq, set_base=self.wr_pin, out_base=self.data_pins[0])
        self.sm.active(1)
        self.sm.put(0)

    def set_value(self, value: int):
        """
        Sets the DAC value.

        Args:
            value (int): 8-bit value to be set on the DAC.

        Raises:
            ValueError: If the value is not between 0 and 255.
        """
        if not (0 <= value <= 255):
            raise ValueError("Value must be between 0 and 255")
        self.sm.put(value)
        time.sleep_us(1)

    def show_parameters(self):
        """
        Prints the AD7801 parameters.
        """
        print(
            f"Data pins: {[pin for pin in self.data_pins]}\n"
            f"WR pin: {self.wr_pin}\n"
            f"State Machine Frequency: {self.freq} Hz"
        )


class AD7801_I2C(I2C_ACCA):
    """
    A class to interface with the AD7801 digital-to-analog converter via I2C.

    Args:
        id (int): The I2C bus ID.
        sda_pin (Pin): The pin used for SDA.
        scl_pin (Pin): The pin used for SCL.
        baudrate (int, optional): The communication baud rate. Defaults to 400000.
    """

    def __init__(
        self,
        id: int,
        sda_pin: Pin,
        scl_pin: Pin,
        baudrate: Literal[10000, 100000, 400000] = 400000,
    ) -> None:
        super().__init__(id=id, sda_pin=sda_pin, scl_pin=scl_pin, baudrate=baudrate)

    def set_value(self, channel: int, value: int):
        """
        Sets the value of the AD7801 digital-to-analog converter.

        Args:
            channel (int): The channel to set.
            value (int): The value to set.

        Returns:
            int: The result of the send operation.

        Raises:
            Exception: If there is an error setting the value.
        """
        try:
            return self.send(bytearray([channel, value]))
        except Exception as e:
            print(f"Failed to set value: {e}")


class AD7819_I2C(I2C_ACCA):
    """
    A class to interface with the AD7819 analog-to-digital converter via I2C.

    Args:
        id (int, optional): The I2C bus ID. Defaults to 0.
        sda_pin (Pin, optional): The pin used for SDA. Defaults to Pin(8).
        scl_pin (Pin, optional): The pin used for SCL. Defaults to Pin(9).
        baudrate (int, optional): The communication baud rate. Defaults to 400000.
    """

    def __init__(
        self,
        id: int = 0,
        sda_pin: Pin = Pin(8),
        scl_pin: Pin = Pin(9),
        baudrate: int = 400000,
    ) -> None:
        super().__init__(id=id, sda_pin=sda_pin, scl_pin=scl_pin, baudrate=baudrate)
        self.set_pins()

    def set_pins(
        self, conv_pin: int = 13, rd_pin: int = 11, cs_pin: int = 10, busy_pin: int = 13
    ):
        """
        Sets the control pins for the AD7819.

        Args:
            conv_pin (int): The pin used for CONV signal.
            rd_pin (int): The pin used for RD signal.
            cs_pin (int): The pin used for CS signal.
            busy_pin (int): The pin used for BUSY signal.
        """
        self.conv_pin = Pin(conv_pin, Pin.OUT)
        self.rd_pin = Pin(rd_pin, Pin.OUT)
        self.cs_pin = Pin(cs_pin, Pin.OUT)
        self.busy_pin = Pin(busy_pin, Pin.OUT)

    def _set_high(self):
        """
        Sets the control pins to high.
        """
        self.conv_pin.value(1)
        self.rd_pin.value(1)
        self.cs_pin.value(1)
        self.busy_pin.value(1)

    def _start_conversion(self):
        """
        Starts the conversion process.
        """
        self.conv_pin.value(0)
        time.sleep_us(1)
        self.conv_pin.value(1)

    def read(self, sample):
        """
        Reads data from the AD7819.

        Args:
            sample (int): The number of samples to read.

        Returns:
            list: The data read from the AD7819.
        """
        data = []
        for i in range(sample):
            self._set_high()
            self._start_conversion()
            self.rd_pin.value(0)
            self.cs_pin.value(0)
            message = self.i2c.readfrom(self.addr, 2)
            time.sleep_us(50)
            data.append(message)
        return data
