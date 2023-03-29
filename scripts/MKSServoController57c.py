'''
python api to control mksservo57c via RS485
Before you run the script, setup following on motor using the embedded board buttons
Direction set to counter clock wise on the motor
set to 0 origin
set to uart mode
set baudrate to 38400
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
    def __init__(self, port, slave_address, baudrate=38400, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.current_position = None
        self.degrees_corresponding_to_int32 = None
        self.get_current_position_response_thread = threading.Thread(target=self.get_current_position_response,args=(slave_address,))
        self.input_thread = threading.Thread(target=self.user_input,args=(slave_address,))
        
    def start(self):
        self.get_current_position_response_thread.start()
        self.input_thread.start()
    
    def join(self):
        self.get_current_position_response_thread.join()
        self.input_thread.join()

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
        pulse_to_bytes = pulses_for_desired_position.to_bytes(4,byteorder='big')
        packet = bytearray()
        packet.append(byte_head)
        packet.append(slave_address)
        packet.append(pos_control_id)
        packet.append(0x00) #counter clockwise direction
        packet.append(0x40) #speed
        packet.append(0x20) #acceleration
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
        pulse_to_bytes = pulses_for_desired_position.to_bytes(4,byteorder='big')
        packet = bytearray()
        packet.append(byte_head)
        packet.append(slave_address)
        packet.append(pos_control_id)
        packet.append(0x80) #clock wise direction
        packet.append(0x40) #speed
        packet.append(0x20) #acceleration
        packet.append(pulse_to_bytes[0])
        packet.append(pulse_to_bytes[1])
        packet.append(pulse_to_bytes[2])
        packet.append(pulse_to_bytes[3])
        crc = (sum(packet) & 0xFF)
        packet.append(crc)
        self.send_packet(packet)

    def set_absolute_position(self, slave_address, set_position):
        self.stop(slave_address)
        time.sleep(2)
        if set_position > self.current_position:
            self.difference = (set_position - self.current_position)
            self.set_cw_position(slave_address,self.difference)
        if set_position < self.current_position:
            self.difference = -1*(set_position - self.current_position)
            self.set_ccw_position(slave_address,self.difference)

    def stop(self, slave_address):
        frame = bytearray([0xfa,slave_address,0xf6,0x00,0x00,0x02])
        crc = (sum(frame) & 0xFF)
        packet = bytearray([0xfa,slave_address,0xf6,0x00,0x00,0x02,crc])
        self.send_packet(packet)

    def send_position_request(self, slave_address):
        frame = bytearray([0xFA,slave_address,0x33])
        crc = (sum(frame) & 0xFF)
        packet = bytearray([0xFA,0x01,0x33,crc])
        self.send_packet(packet)

    def close(self):
        self.ser.close()

    def map_degrees_to_int32(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def get_current_position_response(self,slave_address):
        while True:
            #time.sleep(0.1)
            self.send_position_request(slave_address)
            response = self.receive_packet()
            if response:
                if (len(response) == 8):
                    if (response[0] == 0xfb and response[1]==0x01 and response[2] == 0x33):
                        response_packet = bytearray([response[3],response[4],response[5],response[6]])
                        self.degrees_corresponding_to_int32 = int.from_bytes(response_packet,"big")
                        self.current_position = self.map_degrees_to_int32(self.degrees_corresponding_to_int32,0,3200,0,360)
                if (len(response) == 7):
                    if (response[0] == 0xfb and response[1]==0x01 and response[2] == 0x33):
                        response_packet = bytearray([response[3],response[4],response[5]])
                        self.degrees_corresponding_to_int32 = int.from_bytes(response_packet,"big")
                        self.current_position = self.map_degrees_to_int32(self.degrees_corresponding_to_int32,0,3200,0,360)
                if (len(response) == 6):
                    if (response[0] == 0xfb and response[1]==0x01 and response[2] == 0x33):
                        response_packet = bytearray([response[3],response[4]])
                        self.degrees_corresponding_to_int32 = int.from_bytes(response_packet,"big")
                        self.current_position = self.map_degrees_to_int32(self.degrees_corresponding_to_int32,0,3200,0,360)
                if (len(response) == 5):
                    if (response[0] == 0xfb and response[1]==0x01 and response[2] == 0x33):
                        response_packet = bytearray([response[3]])
                        self.degrees_corresponding_to_int32 = int.from_bytes(response_packet,"big")
                        self.current_position = self.map_degrees_to_int32(self.degrees_corresponding_to_int32,0,3200,0,360)
            #print(self.current_position)

    def user_input(self,slave_address):
        while True:
            num = int(input("Enter an integer: "))
            if num < 0 or num > 360:
                print("enter only in the range of 0 to 360")
            else:
                #print("Position in user input loop: ")
                #print(self.current_position)
                #print("Position in user input loop pulses: ")
                self.set_absolute_position(slave_address,num)



def main():

    motor = MotorController('/dev/tty.usbserial-0001',0x01)
    motor.start()
    motor.join()

        
if __name__ == '__main__':
    main()
