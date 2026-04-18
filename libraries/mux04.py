import time
from machine import I2C, Pin

SHARED_MODE = False
try:
    from libraries.ina226 import get_shared_i2c
    SHARED_MODE = True
except ImportError:
    try:
        from ina226 import get_shared_i2c
        SHARED_MODE = True
    except ImportError:
        pass

class LineSensor:
    def __init__(self, i2c_addr=0x12, i2c_bus=0, freq=400000):
        self.i2c_addr = i2c_addr
        if SHARED_MODE:
            self.i2c = get_shared_i2c(i2c_bus=i2c_bus, freq=freq)
        else:
            self.i2c = self._auto_detect_standalone_i2c(i2c_bus, freq)
        self._init_sensor()

    def _auto_detect_standalone_i2c(self, i2c_bus, freq):
        pin_configs = [(6, 7), (7, 6)]
        for sda_pin, scl_pin in pin_configs:
            try:
                temp_i2c = I2C(i2c_bus, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq)
                if self.i2c_addr in temp_i2c.scan():
                    print(f"[LineSensor 단독 개통] SDA={sda_pin}, SCL={scl_pin}")
                    return temp_i2c
            except Exception:
                pass
        print("[LineSensor 경고] 센서를 찾지 못했습니다. 배선을 확인하세요.")
        return I2C(i2c_bus, sda=Pin(6), scl=Pin(7), freq=freq)

    def _init_sensor(self):
        try:
            self.i2c.writeto_mem(self.i2c_addr, 0x01, bytes([1]))
            time.sleep(0.1)
            self.i2c.writeto_mem(self.i2c_addr, 0x01, bytes([0]))
            time.sleep(0.1)
        except OSError as e:
            print(f"센서 초기화 실패 (I2C 연결 확인 필요): {e}")

    def read_raw(self):
        try:
            data = self.i2c.readfrom_mem(self.i2c_addr, 0x30, 1)
            return data[0]
        except OSError:
            return 0

    def read_channels(self):
        raw = self.read_raw()
        return [(raw >> i) & 0x01 for i in range(7, -1, -1)]

    def get_error(self):
        channels = self.read_channels()
        weights = [-20, -15, -10, -5, 5, 10, 15, 20]
        error_sum = 0
        active_count = 0
        for i in range(8):
            if channels[i] == 1:
                error_sum += weights[i]
                active_count += 1
        if active_count == 0:
            return None 
        return error_sum / active_count