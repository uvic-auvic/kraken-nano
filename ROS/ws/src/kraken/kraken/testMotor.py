import serial
import time

# You may need to adjust the COM port to the correct USB port on your laptop

ser = serial.Serial('/dev/ttyTHS1',115200, timeout=3)  # open serial port



############################ Important ###########################

# Set bytex = ord('Y') for ASCII version of a char or bytex = 0b00011000 to set the bits of a byte directly 

###################################################################

# Change the message here
byte1 = ord('I') # Byte 1 to be sent
byte2 = ord('N') #0b00000001 # Byte 2 to be sent
byte3 = ord('I') #0b00000010 # Byte 3 to be sent


data_to_send = bytes([byte1, byte2, byte3])
ser.write(data_to_send)
