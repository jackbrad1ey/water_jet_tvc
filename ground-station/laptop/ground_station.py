import serial
import struct
import time
import sys
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
            sys.exit()

    def _acknowledge(self):
        self.serial_port.write(b"f")
        return b"\x66" == self.serial_port.read()
    
    def test_gimbal(self):
        packet = bytearray()
        packet.append(2)

        self.serial_port.write(packet)
    
    def controller_control(self):
        angle_x, angle_y = self.controller.convert_to_angles()
        packet = bytearray()
        packet.append(1)
        packet.append(angle_x)
        packet.append(angle_y)


    
# SWC -> enable motors
# SWA -> 