import serial
import struct
import time
import os
import controller
import zlib

class GroundStation:
    def __init__(self, com_port):
        self.comp_port = com_port
        self.serial_port = serial.Serial(com_port, baudrate=115200, timeout=2)
        self.controller = controller.Controller()

        time.sleep(1)

        for _ in range(5):
            if self._acknowledge():
                break

            print("Failed to get serial acknowledgement, retrying...")
            time.sleep(2)
        else:
            print("Acknowledgement failed, terminating program...")
            os._exit(1)  # kill all threads and exit

        print("------Serial connection active------")

    def _acknowledge(self):
        self.serial_port.write(b"\x1b")
        return b"\x66" == self.serial_port.read()

    # def _calculate_crc32(self, received_bytes):
    #     # zlib uses 0x4C11DB7 polynomial
    #     return zlib.crc32(received_bytes[0:-4])

    # def _compare_crc32(self, received_bytes):
    #     rec_CRC = received_bytes[-4:-1]
    #     calc_CRC = self.calculate_crc32(received_bytes)
    #     return calc_CRC == rec_CRC

    def test_gimbal(self):
        self.serial_port.write(b"\x02")
    
    def controller_control(self):
        packet = bytearray()
        byte_x, byte_y = self.controller.get_single_byte_values()
        # print(angle_x, angle_y)
        # data = struct.pack("<c2L", b"\x01", int(round(angle_x, 2)*100), int(round(angle_y, 2)*100))
        # print(data)
        packet.append(1)
        packet.append(1)
        packet.append(round(byte_x))
        packet.append(round(byte_y))
        self.serial_port.write(packet)
        time.sleep(0.01)

    def ping_vehicle(self):
        packet = bytearray()
        packet.append(3)

        self.serial_port.write(packet)
        result = self.serial_port.read()
        
        return result == b"\x01"


gs = GroundStation("/dev/tty.usbmodem101")
connected = False

while True:
    if gs.controller.swc == 1 and not connected:
        print("SWC toggled, attempting to ping vehicle");
        for _ in range(5):
            if gs.ping_vehicle():
                print("Successfully pinged vehicle")
                break

            print("Ping failed")
            time.sleep(2)
        else:
            print("Failed to connect to vehicle, terminating...")
            os._exit(1)

        connected = True
        print("--Vehicle connected--")
    elif gs.controller.swc == 0 and connected:
        connected = False
        print("--Vehicle disconnected--")
    elif gs.controller.swa == 4 and gs.controller.swc == 1:
        gs.controller_control()
    elif gs.controller.swa == 7 and gs.controller.swc == 1:
        gs.test_gimbal()
    

    
# SWC -> enable motors
# SWA -> 


# sensor sample 
#  realtime
#  512
#  Default, NULL, Dynamic
# LoRa
#  low
#  512
#  Default NULL Dynamic
# kalman filter
#  low (same as lora)
#  1024
#  Default NULL Dynamic