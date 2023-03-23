'''
python api to control mksservo57c via RS485
Before you run the script, setup following on motor using the embedded board buttons
Direction set to counter clock wise on the motor
set to 0 origin
set to uart mode
set baudrate to 9600
set uart address to 0x01
Use a USB to RS485 controller

@Author: DP

Todo: 
1. make it more readable and use proper naming for the constant values
2. make it bug free when sending random degrees while the motor is moving
3. make it more thread safe
4. add other functionalities for velocity, speed and torque control
5. add also other motor configuration packets and settings
6. make also pack and unpack style for serial packet encoding and decoding later if possible instead of using lot of byte arrays
6. Possibly make a ROS or ROS2 binding later.
'''

import serial
import time
import threading

class MotorController:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.current_position = 0
        self.previous_position = 0
        
    def send_packet(self, packet):
        self.ser.write(packet)

    def receive_packet(self):
        response = self.ser.readline()
        if response:
            response_bytes = bytearray(response)
            return response_bytes
        else:
            return None
        
    def set_ccw_position(self, slave_address, position):
        byte_head = 0xfa
        pos_control_id = 0xfd
        pulses_for_desired_position = int(position*(3200/360)) #for 1.8 degrees and 16 sub divisions to reach 360 degrees
        print(pulses_for_desired_position)
        pulse_to_bytes = pulses_for_desired_position.to_bytes(4,byteorder='big')
        packet = bytearray()
        packet.append(byte_head)
        packet.append(slave_address)
        packet.append(pos_control_id)
        packet.append(0x00) #counter clockwise direction
        packet.append(0x05) #speed
        packet.append(0x02) #acceleration
        packet.append(pulse_to_bytes[0])
        packet.append(pulse_to_bytes[1])
        packet.append(pulse_to_bytes[2])
        packet.append(pulse_to_bytes[3])
        crc = (sum(packet) & 0xFF)
        packet.append(crc)
        self.send_packet(packet)
    
    def set_cw_position(self, slave_address, position):
        byte_head = 0xfa
        pos_control_id = 0xfd
        
        pulses_for_desired_position = int(position*(3200/360)) 
        print(pulses_for_desired_position)
        pulse_to_bytes = pulses_for_desired_position.to_bytes(4,byteorder='big')
        packet = bytearray()
        packet.append(byte_head)
        packet.append(slave_address)
        packet.append(pos_control_id)
        packet.append(0x80) #clock wise direction
        packet.append(0x05) #speed
        packet.append(0x02) #acceleration
        packet.append(pulse_to_bytes[0])
        packet.append(pulse_to_bytes[1])
        packet.append(pulse_to_bytes[2])
        packet.append(pulse_to_bytes[3])
        crc = (sum(packet) & 0xFF)
        packet.append(crc)
        self.send_packet(packet)

    def set_absolute_position(self, slave_address, set_position):
        
        if set_position > self.current_position:
            self.difference = (set_position - self.current_position)
            self.set_ccw_position(slave_address,self.difference)
            self.current_position = set_position
        if set_position < self.current_position:
            self.difference = -1*(set_position - self.current_position)
            self.set_cw_position(slave_address,self.difference)
            self.current_position = set_position

    def stop(self, slave_address):
        packet = bytearray()
        packet.append(0xfa)
        packet.append(slave_address)
        packet.append(0xf6)
        packet.append(0x00)
        packet.append(0x00)
        packet.append(0x02)
        crc = (sum(packet) & 0xFF)
        packet.append(crc)
        self.send_packet(packet)

    def get_current_position(self):
        packet = bytearray()
        packet.append(0xfa)
        packet.append(0x01)
        packet.append(0x33)
        crc = (sum(packet) & 0xFF)
        packet.append(crc)
        self.send_packet(packet)

    def close(self):
        self.ser.close()

    def map_degrees_to_int32(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def homing(self):
        self.get_current_position()
        response = self.receive_packet()
        if response:
            print('Received packet:', response)
            print(len(response))
            if len(response) == 8:
                response_packet = bytearray()
                response_packet.append(response[3])
                response_packet.append(response[4])
                response_packet.append(response[5])
                response_packet.append(response[6])
        degrees_corresponding_to_int32 = int.from_bytes(response_packet,"big")
        print(degrees_corresponding_to_int32)
        self.current_position = self.map_degrees_to_int32(degrees_corresponding_to_int32,0,3200,0,360)
        return degrees_corresponding_to_int32


def main():

    motor = MotorController('/dev/ttyUSB1')
    motor.homing()
    #print("value is: ", motor.map_degrees_to_int32(value,0,3200,0,360))
    while True:
        num = int(input("Enter an integer: "))
        if num < 0 or num > 360:
            print("enter only in the range of 0 to 360")
        else:
            motor.set_absolute_position(0x01,num)

if __name__ == '__main__':
    main()
