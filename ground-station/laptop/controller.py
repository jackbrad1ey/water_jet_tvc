import hid
import threading

class Controller:
    def __init__(self):
        self.interface = hid.device()
        self.interface.open(0x1209, 0x4f54)
        self.interface.set_nonblocking(True)
        self.ljs_v = 0
        self.ljs_h = 0
        self.rjs_v = 0
        self.rjs_h = 0
        self.swc = 0
        self.swa = 0
    
        update_thread = threading.Thread(target=self._update_thread)
        update_thread.start()

    def _update_thread(self):
        while True:
            raw_data = self.interface.read(64)

            if not raw_data:  # didn't read anything
                continue

            self.swc = raw_data[0]
            self.swa = raw_data[18]
            self.swd = raw_data[0]
            self.swb = raw_data[0]
            # min 0, max 2048
            self.ljs_v = raw_data[7] + 256 * raw_data[8]
            self.ljs_h = raw_data[9] + 256 * raw_data[10]
            self.rjs_v = raw_data[5] + 256 * raw_data[6]
            self.rjs_h = raw_data[3] + 256 * raw_data[4]

    def convert_to_angles(self):
        MIN_ANG = 0
        MAX_ANG = 180
        MIN_IN = 0
        MAX_IN = 2048

        angle_x = MIN_ANG + ((MAX_ANG - MIN_ANG) / (MAX_IN - MIN_IN)) * (self.rjs_h - MIN_IN)
        angle_y = MIN_ANG + ((MAX_ANG - MIN_ANG) / (MAX_IN - MIN_IN)) * (self.rjs_v - MIN_IN)

        return angle_x, angle_y

    def get_single_byte_values(self):
        MIN_OUT = 0
        MAX_OUT = 255
        MIN_IN = 0
        MAX_IN = 2048

        byte_x = MIN_OUT + ((MAX_OUT - MIN_OUT) / (MAX_IN - MIN_IN)) * (self.rjs_h - MIN_IN)
        byte_y = MIN_OUT + ((MAX_OUT - MIN_OUT) / (MAX_IN - MIN_IN)) * (self.rjs_v - MIN_IN)

        return byte_x, byte_y


        
# SWA BACK      -> l[18] = 0
# SWA MID       -> l[18] = 4
# SWA FOR       -> l[18] = 7
# SWC BACK      -> l[0] = 0
# SWC FOR       -> l[0] = 1
# Left joy up   -> l[7] = amount and l[8] rollover
# Right joy up  -> l[5] = amount and l[6] rollover
# Left joy lr   -> l[9] = amount and l[10] rollover
# Right joy lr  -> l[3] = amount and l[4] rollover


# 255 * 8 = 2040 total
# amount = amount + 255 * amount

# for device in hid.enumerate():
#     print(device)
#     print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

# gamepad = hid.device()
# gamepad.open(0x1209, 0x4f54)
# # gamepad.set_nonblocking(True)

# # while True:
# #     report = gamepad.read(64)
# #     if report:
# #         print(report)