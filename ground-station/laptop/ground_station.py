import serial
import struct
import time
import os
import controller

class GroundStation:
    def __init__(self, com_port):
        self.comp_port = com_port
        self.serial_port = serial.Serial(com_port, baudrate=115200, timeout=2)
        self.controller = controller.Controller()

        time.sleep(1)

        for _ in range(5):
            if self._acknowledge():
                break

            print("Failed to receive acknowledgement packet, retrying...")
            time.sleep(2)
        else:
            print("Acknowledgement failed, terminating program...")
            os._exit(1)  # kill all threads and exit
        
        print("Serial connection active")

    def _acknowledge(self):
        self.serial_port.write(b"\x1b")
        return b"\x66" == self.serial_port.read()
    
    def test_gimbal(self):
        packet = bytearray()
        packet.append(2)

        self.serial_port.write(packet)
    
    def controller_control(self):
        while self.controller.swa == 4:
            angle_x, angle_y = self.controller.convert_to_angles()
            data = struct.pack("cff", 1, angle_x, angle_y)
            self.serial_port.write(data)

    def ping_vehicle(self):
        packet = bytearray()
        packet.append(3)

        self.serial_port.write(packet)
        result = self.serial_port.read()
        print(result)
        if result == b"\x01":
            print("Successfully pinged vehicle")
        else:
            print("Ping failed")


gs = GroundStation("/dev/tty.usbmodem101")
connected = False

while True:
    if gs.controller.swc == 1 and not connected:
        gs.ping_vehicle()
        connected = True
        print("Armed")
    elif gs.controller.swc == 0 and connected:
        connected = False
        print("Disarmed")
    elif gs.controller.swa == 4 and gs.controller.swc == 1:
        gs.controller_control()
    elif gs.controller.swa == 7 and gs.controller.swc == 1:
        gs.test_gimbal()
    

    
# SWC -> enable motors
# SWA -> 


