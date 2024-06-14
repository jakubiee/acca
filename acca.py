import time
from onewire import OneWire  # type: ignore
from ds18x20 import DS18X20  # type: ignore
from machine import I2C, SoftSPI, Pin as MachinePin  # type: ignore
from typing import Literal, Optional
from typing_extensions import Annotated, Doc


class Pin:
    """
    `Pin` class, a simple class to emulate the `machine.Pin`

    ## Example
    ```python
    from acca import Pin

    led = Pin(2, Pin.OUT, 1)
    ```
    """

    OUT = MachinePin.OUT
    IN = MachinePin.IN

    def __init__(
        self,
        pin: Annotated[
            int,
            Doc(
                """
                Pin number
                """
            ),
        ],
        mode: Annotated[
            Optional[int],
            Doc(
                """
            Pin mode (input or output)
            """
            ),
        ] = None,
        value: Annotated[
            Optional[int],
            Doc(
                """
            Initial value of the pin
            """
            ),
        ] = None,
    ) -> None:
        """
        Initializes the Pin instance.

        :param pin: Pin number
        :param mode: Pin mode (input or output)
        :param value: Initial value of the pin
        """
        self._pin = MachinePin(pin)
        if mode is not None:
            self.mode(mode)
        if value is not None:
            self.value(value)

    def mode(self, mode: Optional[int] = None) -> "Pin":
        """
        Sets or gets the mode of the pin.

        :param mode: Pin mode to set (input or output)
        :return: The Pin instance for method chaining
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

        :param val: Value to set (0 or 1)
        :return: The Pin instance for method chaining
        """
        if val is None:
            return self._pin.value()  # type: ignore
        if not isinstance(val, int):
            raise TypeError("Pin value must be an integer")
        self._pin.value(val)  # type: ignore
        return self

def check_address(func):
    def wrapper(self, *args, **kwargs):
        if self.addr is None:
            raise ValueError("No I2C device address found.")
        return func(self, *args, **kwargs)
    return wrapper

class I2C_ACCA:
    def __init__(self, id: int, sda_pin: Pin, scl_pin: Pin, baudrate: int) -> None:
        self.id = id
        self.sda_pin = sda_pin
        self.scl_pin = scl_pin
        self.baudrate = baudrate
        self.i2c = I2C(id, sda=sda_pin, scl=scl_pin, freq=baudrate)
        self.addr = self.get_addr()

    def get_addr(self):
        addr = self.i2c.scan()
        if not addr:
            return None
        return addr[0]

    @check_address
    def send(self, data, stop=True):
        try:
            return self.i2c.writeto(self.addr, data, stop)
        except Exception as e:
            print(f"Error sending data: {e}")
            raise

    @check_address
    def read(self, nbytes, stop=True):
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


class DS18B20_OneWire:
    def __init__(self, pin: Pin):
        self.onewire = OneWire(pin)
        self.ds = DS18X20(self.onewire)
        self.addr = self.scan()

    def scan(self):
        addresses = self.onewire.scan()
        if not addresses:
            raise Exception("No device found")
        return addresses[0]

    def read_temp(self, with_scratch: bool = False):
        self.ds.convert_temp()
        time.sleep_ms(750)  # type: ignore
        if with_scratch:
            return [self.ds.read_temp(self.addr), self.ds.read_scratch(self.addr)]
        return self.ds.read_temp(self.addr)


class MCP3461_SPI(SPI_ACCA):
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
        self.cs.value(0)
        self.spi.write(bytearray(message))
        self.cs.value(1)
        time.sleep_us(1)  # type: ignore

    def read(self, sample: int):
        self.cs.value(0)
        data = []
        for i in range(sample):
            data.append(self.spi.read(4))
            time.sleep_us(1)  # type: ignore
        return data


class AD7801_I2C(I2C_ACCA):
    def __init__(
        self,
        id: int,
        sda_pin: Pin,
        scl_pin: Pin,
        baudrate: Literal[10000, 100000, 400000] = 400000,
    ) -> None:
        super().__init__(id=id, sda_pin=sda_pin, scl_pin=scl_pin, baudrate=baudrate)

    @check_address
    def set_value(self, channel: int, value: int):
        try:
            return self.send(bytearray([channel, value]))
        except Exception as e:
            print(f"Failed to set value: {e}")


class AD7819_I2C(I2C_ACCA):
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
        self.conv_pin = Pin(conv_pin, Pin.OUT)
        self.rd_pin = Pin(rd_pin, Pin.OUT)
        self.cs_pin = Pin(cs_pin, Pin.OUT)
        self.busy_pin = Pin(busy_pin, Pin.OUT)

    def _set_high(self):
        self.conv_pin.value(1)
        self.rd_pin.value(1)
        self.cs_pin.value(1)
        self.busy_pin.value(1)

    def _start_conversion(self):
        self.conv_pin.value(0)
        time.sleep_us(1)
        self.conv_pin.value(1)

    @check_address
    def read(self, sample):
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
