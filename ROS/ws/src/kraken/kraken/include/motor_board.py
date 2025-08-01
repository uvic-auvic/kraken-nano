import serial

class System():
    PROPULSION = 'propulsion'
    VERTICAL = 'vertical'
    YAW = 'yaw'
    ROLL = 'roll'
    PITCH = 'pitch'
    DROPPER = 'dropper'
    GRABBER = 'grabber'
    TORPEDO = 'torpedo'
    LED = "led"

class MotorBoard:
    def led(self, indices, magnitude):
        """
        Toggle LEDs by index (0-2). 'indices' is a list of LED indices to toggle.
        'magnitude' is an int 0-2; the value byte will be 1 << magnitude (only one bit set).
        The motor_mask will have bits set for each index in 'indices'.
        """
        if not all(0 <= idx <= 2 for idx in indices):
            raise ValueError("LED indices must be between 0 and 2")
        if not (0 <= magnitude <= 2):
            raise ValueError("Magnitude must be between 0 and 2")
        motor_mask = 0
        for idx in indices:
            motor_mask |= (1 << idx)
        value_byte = 1 << magnitude
        self._send(System.LED, motor_mask, value_byte)
    """
    Handles UART communication for all robot systems and motors.
    System and motor mapping is internal and fixed for this robot.
    Each UART message is 3 bytes:
      - Byte 0: System ID
      - Byte 1: Motor bitmask (bit i set => motor i controlled)
      - Byte 2: Signed int8, direction and power/speed
    """
    # Internal mapping: System enum -> (system_id, [motor_indices])
    SYSTEMS = {
        System.PROPULSION: ('M', [0, 1]),
        System.VERTICAL: ('M', [0]),
        System.YAW: ('M', [0]),
        System.ROLL: ('M', [0]),
        System.PITCH: ('M', [0]),
        System.DROPPER: ('M', [0]),
        System.GRABBER: ('M', [0]),
        System.TORPEDO: ('M', [0]),
        System.LED: ('L', [0]),
    }

    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def _send(self, system: System, motor_mask, value):
        if system not in self.SYSTEMS:
            raise ValueError(f"Unknown system: {system}")
        system_id, _ = self.SYSTEMS[system]
        if not (0 <= system_id <= 255):
            raise ValueError("System ID must be 0-255")
        if not (0 <= motor_mask <= 255):
            raise ValueError("Motor mask must be 0-255")
        if not (-128 <= value <= 127):
            raise ValueError("Value must be -128 to 127")
        value_byte = value & 0xFF
        msg = bytes([ord(system_id), motor_mask, value_byte])
        self.serial.write(msg)

    def propulsion(self, value, motors=[0, 1]):
        motor_mask = 0
        for idx in motors:
            motor_mask |= (1 << idx)
        self._send(System.PROPULSION, motor_mask, value)

    def right(self, value):
        self._send(System.PROPULSION, 1 << 1, value)

    def forward(self, value):
        self._send(System.PROPULSION, 1 << 0, value)

    def vertical(self, value):
        self._send(System.VERTICAL, 1 << 0, value)

    def yaw(self, value):
        self._send(System.YAW, 1 << 0, value)

    def roll(self, value):
        self._send(System.ROLL, 1 << 0, value)

    def pitch(self, value):
        self._send(System.PITCH, 1 << 0, value)

    def dropper(self, value):
        self._send(System.DROPPER, 1 << 0, value)

    def grabber(self, value):
        self._send(System.GRABBER, 1 << 0, value)

    def torpedo(self, value):
        self._send(System.TORPEDO, 1 << 0, value)

    def close(self):
        if self.serial.is_open:
            self.serial.close()
