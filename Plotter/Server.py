from time import sleep
import threading as th
import serial
COM = 'COM5'

Init_command = bytes([0xC0,0x01,0x01,0xC0])
arduino = serial.Serial(port=COM, baudrate=115200, timeout=.1)

def init_command():
   arduino.write(Init_command)

def read_arduida():
    return arduino.readline()

def rx_thread():
    while True:
        print(read_arduida())

def try_to_init_thread():
    
    while True:
        ##print(Init_command)
        ##print(arduino.write(Init_command))
        print(read_arduida())
        sleep(0.1)
        init_command()
        sleep(0.1)

##Тело
#RX = th.Thread(target = rx_thread)
#RX.start()
Init = th.Thread(target = try_to_init_thread)
Init.start()

