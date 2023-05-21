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
        time.sleep(0.03)

    def ping_vehicle(self):
        packet = bytearray()
        packet.append(3)

        self.serial_port.write(packet)
        result = self.serial_port.read()
        
        return result == b"\x01"

    def report_sensors(self):
        packet = bytearray()
        packet.append(4)
        self.serial_port.write(packet)
        # print(self.serial_port.read(), self.serial_port.read(), self.serial_port.read(), self.serial_port.read())
        roll_sign = -1 if int.from_bytes(self.serial_port.read(), "big") else 1
        roll = int.from_bytes(self.serial_port.read(), "big")
        pitch_sign = -1 if int.from_bytes(self.serial_port.read(), "big") else 1
        pitch = int.from_bytes(self.serial_port.read(), "big")
        print(f"Roll: {roll_sign * roll}\tPitch: {pitch_sign * pitch}")
        time.sleep(0.2)
    
    def counter_tilt(self):
        packet = bytearray()
        packet.append(5)
        self.serial_port.write(packet)

# gs = GroundStation("/dev/tty.usbmodem101")
gs = GroundStation("COM6")
connected = False
sens_report = False

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
        print("--Servos armed--")
    elif gs.controller.swc == 0 and connected:
        connected = False
        print("--Servos disarmed--")
    elif gs.controller.swa == 4 and gs.controller.swc == 1:
        gs.controller_control()
    elif gs.controller.swa == 7 and gs.controller.swc == 1:
        gs.test_gimbal()
    elif gs.controller.swd == 2 and not sens_report:
        print("-Reporting sensor data-")
        sens_report = True
    elif gs.controller.swd == 2 and sens_report:
        gs.report_sensors()
    elif sens_report and gs.controller.swd != 2:
        print("-End sensor report-")
        sens_report = False
    elif gs.controller.swb == 4:
        gs.counter_tilt()

    
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