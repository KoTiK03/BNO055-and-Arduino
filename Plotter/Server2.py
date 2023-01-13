import pyquaternion as pyquat
from time import sleep
import threading as th
import serial
COM = 'COM5'

velocity_slice = 0.001
position_slice = 0.000000001

Init_command = bytes([0xC0,0x01,0x01,0xC0])
arduino = serial.Serial(port=COM, baudrate=115200, timeout=.1)

def init_command():
   arduino.write(Init_command)

def read_arduida():
    return arduino.readline()
    
def parser():
    ret = []
    str4parse = str(read_arduida()).split()

    ##print(str4parse)
    
    if (len(str4parse)!=9):
        return -1
    str4parse[0] = str4parse[0][2:]
    str4parse[8] = str4parse[8][:-5]
    for i in range (9):
        ret.append(float(str4parse[i]))
    return ret

class Classniy:
    def __init__(self):
        self.velocity = [0.0,0.0,0.0]
        self.position = [0.0,0.0,0.0]
        
    def integrator(self,data):
        Rotate = pyquat.Quaternion(data[5],data[6],data[7],data[8]) 
        ##Vector = pyquat.Quaternion(0,data[0],data[1],data[2]) 
        ##Rotate_inversion = Rotate.inverse()
        rotated_data = Rotate.rotate([data[0],data[1],data[2]])
        print(rotated_data)
        time_dif = data[3] - data[4]
        for i in range(3):
            self.velocity[i] += rotated_data[i] * time_dif / 1000
            if (abs(self.velocity[i]) < velocity_slice):
                self.velocity[i] = 0
        for i in range(3):
            self.position[i] += self.velocity[i] *time_dif / 1000

    def get_position(self):
        return self.position
    def get_velocity(self):
        return self.velocity

##Тело
BNO = Classniy()
while True:
    data = parser()
    if data == -1:
        continue
    BNO.integrator(data)
    ###print (data)
    ###print (BNO.get_velocity())
    ###print (BNO.get_position())
    
    
    
