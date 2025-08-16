from serial import Serial

class MotorBoard():
        def __init__(self, port, baud=115200):
                self.ser = Serial(port, baud, timeout=3)
                self.buffer = []
                self.positive_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                self.negative_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                
        def front_motor(self, positive=True):
                if positive:
                        self.positive_mask[3] = int(1)
                else:
                        self.negative_mask[3] = int(1)
        
        def back_motor(self, positive=True):
                if positive:
                        self.positive_mask[1] = int(1)
                else:
                        self.negative_mask[1] = int(1)
        
        def left_motor(self, positive=True):
                if positive:
                        self.positive_mask[6] = int(1)
                else:
                        self.negative_mask[6] = int(1)
        
        def right_motor(self, positive=True):
                if positive:
                        self.positive_mask[0] = int(1)
                else:
                        self.negative_mask[0] = int(1)
        
        def front_left_motor(self, positive=True):
                if positive:
                        self.positive_mask[4] = int(1)
                else:
                        self.negative_mask[4] = int(1)
        
        def front_right_motor(self, positive=True):
                if positive:
                        self.positive_mask[7] = int(1)
                else:
                        self.negative_mask[7] = int(1)
        
        def back_left_motor(self, positive=True):
                if positive:
                        self.positive_mask[2] = int(1)
                else:
                        self.negative_mask[2] = int(1)
        
        def back_right_motor(self, positive=True):
                if positive:
                        self.positive_mask[7] = int(1)
                else:
                        self.negative_mask[7] = int(1)
                
        def forward(self):
                self.left_motor(True)
                self.right_motor(True)
                
        def backward(self):
                self.left_motor(False)
                self.right_motor(False)
        
        def left(self):
                self.front_motor(True)
                self.back_motor(True)
        
        def right(self):
                self.front_motor(False)
                self.back_motor(False)
        
        def down(self):
                self.front_left_motor(True)
                self.front_right_motor(True)
                self.back_left_motor(True)
                self.back_right_motor(True)

        def up(self):
                self.front_left_motor(False)
                self.front_right_motor(False)
                self.back_left_motor(False)
                self.back_right_motor(False)
        
        def yaw_ccw(self):
                self.back_motor(False)
                self.front_motor(True)
                
        def yaw_cw(self):
                self.back_motor(True)
                self.front_motor(False)
        
        def cut_motors(self):
                self.buffer.append(ord("M"))
                self.buffer.append(255)
                self.buffer.append(0)
                self.ser.write(bytes(self.buffer))
                self.buffer = []

        def flip(self):
                self.front_left_motor(False)
                self.front_right_motor(False)
                self.back_left_motor(False)
                self.back_right_motor(False)

        def roll(self):
                self.front_left_motor(True)
                self.front_right_motor(False)
                self.back_left_motor(True)
                self.back_right_motor(False)
                # self.back_motor(True)
                # self.front_motor(True)

        def init_motors(self):
                self.buffer.append(ord("I"))
                self.buffer.append(ord("N"))
                self.buffer.append(ord("I"))
                self.ser.write(bytes(self.buffer))
                self.buffer = []
                
        def send_motors(self, speed):
        
                # Make sure speed is valid
                if speed < 0 or speed > 127:
                        return 0
                        
                # Convert speed to binary
                speed_bits = bin(speed)[2:]
            
                # Pad to 7 bits
                speed_bits = speed_bits.zfill(7)
                
                # Positive and negative speeds for each mask
                positive_speed = "0" + speed_bits
            
                negative_speed = "1" + speed_bits
                
                # Convert speeds to ints
                positive_speed = int(positive_speed, 2)
                negative_speed = int(negative_speed, 2)

                # Put mask in correct order
                self.positive_mask.reverse()
                self.negative_mask.reverse()                

                # Convert masks to ints
                byte_str = "".join(map(str, self.positive_mask))
                positive_decimal = int(byte_str, 2)
                
                byte_str = "".join(map(str, self.negative_mask))
                negative_decimal = int(byte_str, 2)

                if positive_decimal:
                        self.buffer.append(ord("M"))
                        self.buffer.append(positive_decimal)
                        self.buffer.append(positive_speed)
                        self.ser.write(bytes(self.buffer))
                        
                if negative_decimal:
                        self.buffer.append(ord("M"))
                        self.buffer.append(negative_decimal)
                        self.buffer.append(negative_speed)
                        self.ser.write(bytes(self.buffer))             
                
                self.buffer = []
                self.positive_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                self.negative_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                
                return 1
