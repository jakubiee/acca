# I2C_ACCA Class

A class to interface with I2C devices.

## Attributes

- `id` (int): The I2C bus ID.
- `sda_pin` (Pin): The pin used for SDA.
- `scl_pin` (Pin): The pin used for SCL.
- `baudrate` (int): The communication baud rate.

## Methods

### `__init__`

Initializes the I2C_ACCA instance.

**Parameters**:

- `id` (int): The I2C bus ID.

- `sda_pin` (Pin): The pin used for SDA.

- `scl_pin` (Pin): The pin used for SCL.

- `baudrate` (int): The communication baud rate.

### `_get_addr`

Scans for I2C devices and returns the address of the first device found.

**Returns**:
- `int`: The address of the first I2C device found.

**Raises**:
- `Exception`: If no device is found.

### `send`

Sends data to the I2C device.

**Parameters**:

- `data`: The data to send.

- `stop` (bool): Whether to send a stop condition after the data. Default is `True`.

**Returns**:
- The result of the `writeto` operation.

**Raises**:
- `Exception`: If there is an error sending data.

### `read`

Reads data from the I2C device.

**Parameters**:

- `nbytes` (int): The number of bytes to read.

- `stop` (bool): Whether to send a stop condition after reading. Default is `True`.

**Returns**:
- The data read from the I2C device.

**Raises**:
- `Exception`: If there is an error reading data.

### `show_parameters`

Prints the I2C parameters.

**Example**:
```python
i2c = I2C_ACCA(1, Pin(21), Pin(22), 400000)
i2c.show_parameters()
```