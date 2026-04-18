import network
import socket
import time
from drv8833 import DRV8833
from mux04 import LineSensor
from ina226 import INA226

# --- 1. Wi-Fi 및 네트워크 설정 ---
SSID = "Team06"
PASSWORD = "12345678"
PC_IP = "192.168.137.1"
PC_PORT = 5005

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    time.sleep(0.5)
    wlan.active(True)
    
    try: 
        wlan.config(txpower=10)
        print("TX Power 10으로 설정 완료 (전력 안정화)")
    except Exception as e: 
        print("TX Power 설정 미지원:", e)
    
    if not wlan.isconnected():
        print(f'{SSID} 연결 중...')
        wlan.connect(SSID, PASSWORD)
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1
            print(".", end="")
            
    if wlan.isconnected():
        print('\nWi-Fi 연결 성공! IP:', wlan.ifconfig()[0])
        return True
    return False

# --- 2. 하드웨어 초기화 
sensor = LineSensor()
motor = DRV8833()

ina_0x40 = INA226(address=0x40)
ina_0x41 = INA226(address=0x41)

# 주행 설정
kp = 1.5
ki = 0.01
kd = 0.2
base_speed = 80
max_speed = 100
integral_limit = 100
pid_integral = 0
pid_previous_error = 0
pid_has_previous_error = False
last_pid_time = time.ticks_ms()

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

try:
    ina_0x40.configure(avg=4, busConvTime=4, shuntConvTime=4, mode=7)
    ina_0x41.configure(avg=4, busConvTime=4, shuntConvTime=4, mode=7)
    ina_0x40.calibrate(rShuntValue=0.1, iMaxExpected=2.0)
    ina_0x41.calibrate(rShuntValue=0.1, iMaxExpected=2.0)
    print("INA226 센서 초기화 완료")
except Exception as e:
    print("INA226 설정 실패:", e)

# Wi-Fi 연결 및 소켓(UDP) 생성
wifi_status = connect_wifi()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_send_time = time.ticks_ms()

# --- 3. 메인 주행 루프 ---
try:
    print("주행 및 센서 데이터 송신 시작...")
    while True:
        # --- A. 라인 트레이싱 로직 ---
        channels = sensor.read_channels()
        weights = [-55, -30, -20, -10, 10, 20, 30, 55]
        error_sum = 0
        active_count = 0
        steering = 0
        channels = [1-c for c in channels]
        
        for i in range(8):
            if channels[i] == 1:
                error_sum += weights[i]
                active_count += 1
                
        if active_count > 0:
            current_time = time.ticks_ms()
            dt_ms = time.ticks_diff(current_time, last_pid_time)
            if dt_ms <= 0:
                dt = 0.001
            else:
                dt = dt_ms / 1000
            last_pid_time = current_time

            error = error_sum / active_count
            pid_integral += error * dt
            pid_integral = clamp(pid_integral, -integral_limit, integral_limit)

            if pid_has_previous_error:
                derivative = (error - pid_previous_error) / dt
            else:
                derivative = 0

            steering = (kp * error) + (ki * pid_integral) + (kd * derivative)
            pid_previous_error = error
            pid_has_previous_error = True

            left_speed = clamp(base_speed + steering, -max_speed, max_speed)
            right_speed = clamp(base_speed - steering, -max_speed, max_speed)
            motor.set_speed(left_speed, right_speed)
        else:
            pid_integral = 0
            pid_previous_error = 0
            pid_has_previous_error = False
            last_pid_time = time.ticks_ms()
            motor.set_speed(0, 0) 

        # --- B. INA226 데이터 읽기 및 Wi-Fi 송신 (0.5초마다) ---
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_send_time) > 500:
            try:
                v1, ma1 = ina_0x40.read_bus_voltage(), ina_0x40.read_shunt_current()
                v2, ma2 = ina_0x41.read_bus_voltage(), ina_0x41.read_shunt_current()
                
                if None not in (v1, ma1, v2, ma2):
                    if wifi_status:
                        message = f"{v1:.3f},{ma1*1000:.1f},{v2:.3f},{ma2*1000:.1f}"
                        sock.sendto(message.encode(), (PC_IP, PC_PORT))
                        print(f"전송 -> 0x40: {v1:.2f}V/{ma1*1000:.0f}mA | 0x41: {v2:.2f}V/{ma2*1000:.0f}mA")
                else:
                    print("데이터 읽기 오류: 센서 배선을 확인하세요.")
            except Exception as e:
                pass
                
            last_send_time = current_time

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n정지합니다.")
    motor.set_speed(0, 0)
    sock.close()
