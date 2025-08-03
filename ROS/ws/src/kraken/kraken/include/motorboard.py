from serial import Serial

class MotorBoard():
        def __init__(self, port, baud=115200):
                #self.ser = Serial(port, baud, timeout=3)
                self.buffer = []
                self.positive_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                self.negative_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                
        def front_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[0] = int(on)
                else:
                        self.negative_mask[0] = int(on)
        
        def back_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[1] = int(on)
                else:
                        self.negative_mask[1] = int(on)
        
        def left_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[2] = int(on)
                else:
                        self.negative_mask[2] = int(on)
        
        def right_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[3] = int(on)
                else:
                        self.negative_mask[3] = int(on)
        
        def front_left_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[4] = int(on)
                else:
                        self.negative_mask[4] = int(on)
        
        def front_right_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[5] = int(on)
                else:
                        self.negative_mask[5] = int(on)
        
        def back_left_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[6] = int(on)
                else:
                        self.negative_mask[6] = int(on)
        
        def back_right_motor(self, on=True, positive=True):
                if positive:
                        self.positive_mask[7] = int(on)
                else:
                        self.negative_mask[7] = int(on)
                
        def forward(self):
                self.left_motor(True)
                self.right_motor(True)
        
        def left(self):
                self.front_motor(True)
                self.back_motor(True)
        
        def up(self):
                self.front_left_motor(True)
                self.front_right_motor(True)
                self.back_left_motor(True)
                self.back_right_motor(True)
        
        def yaw(self):
                self.front_motor(True)
                self.back_motor(True, False)
                self.left_motor(True, False)
                self.right_motor(True)
        
        def cut_motors(self):
                self.buffer.append("M")
                self.buffer.append(255)
                self.buffer.append(0)
                #self.ser.write(bytes(self.buffer))
                
        def send_motors(self, speed):
                if speed < -128 or speed > 127:
                        return 0
                
                byte_str = "".join(map(str, self.positive_mask))
                positive_decimal = int(byte_str, 2)
                
                byte_str = "".join(map(str, self.negative_mask))
                negative_decimal = int(byte_str, 2)
                
                if positive_decimal:
                
                        self.buffer.append("M")
                        self.buffer.append(positive_decimal)
                        self.buffer.append(speed)
                        #self.ser.write(bytes(self.buffer))
                        
                if negative_decimal:
                        self.buffer.append("M")
                        self.buffer.append(negative_decimal)
                        self.buffer.append(-speed)
                        #self.ser.write(bytes(self.buffer))             
                
                self.buffer = []
                self.positive_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                self.negative_mask = [0, 0, 0, 0, 0, 0, 0, 0]
                
                return 1
