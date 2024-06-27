# Pin Class

`Pin` class, a simple class to emulate the `machine.Pin`

## Example

```python
from acca import Pin

led = Pin(2, Pin.OUT, 1)
```

## Methods

### `__init__`

Initializes the Pin instance.

**Parameters**:

- `pin`: `int`
  - Pin number

- `mode`: `Optional[int]` (default: `None`)
  - Pin mode (input or output)

- `value`: `Optional[int]` (default: `None`)
  - Initial value of the pin

### `mode`

Sets or gets the mode of the pin.

**Parameters**:

- `mode`: `Optional[int]` (default: `None`)
  - Pin mode to set (input or output)

**Returns**:

- The `Pin` instance for method chaining

### `value`

Sets or gets the value of the pin.

**Parameters**:

- `val`: `Optional[int]` (default: `None`)
  - Value to set (0 or 1)

**Returns**:

- The `Pin` instance for method chaining
