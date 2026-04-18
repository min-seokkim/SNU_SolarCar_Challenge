import network
import socket
import time
from drv8833 import DRV8833
from mux04 import LineSensor
from ina226 import INA226
from myservo import myServo

# --- 1. Wi-Fi 및 네트워크 설정 ---
SSID = "Team06"
PASSWORD = "12345678"
PC_IP = "192.168.137.1"
PC_PORT = 5005

wlan = network.WLAN(network.STA_IF)

def connect_wifi():
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
solar_servo = myServo(pin=10)

ina_0x40 = INA226(address=0x40)
ina_0x41 = INA226(address=0x41)

# 주행 설정
kp = 1.5
ki = 0.01
kd = 0.2
base_speed = 80
max_speed = 100
min_speed = 45
slowdown_gain = 0.6
integral_limit = 100
fallback_timeout_ms = 700
fallback_speed = 35
fallback_turn = 55
reverse_brake_ms = 200
reverse_brake_left_speed = -30
reverse_brake_right_speed = -40
charge_stop_ms = 10000
stop_marker_ignore_ms = 500
solar_scan_start_angle = 45
solar_scan_center_angle = 90
solar_scan_end_angle = 135
solar_scan_step = 1
solar_scan_speed = 10
solar_scan_settle_ms = 80
solar_scan_drop_count_limit = 5
pid_integral = 0
pid_previous_error = 0
pid_has_previous_error = False
last_pid_time = time.ticks_ms()
last_seen_steering = 0
lost_line_start_time = None
stop_marker_armed = True
charging_until_time = None
stop_marker_ignore_until_time = None

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def scan_best_solar_angle():
    best_angle = None
    best_power = None
    drop_count = 0

    if solar_servo.current_angle < solar_scan_center_angle:
        scan_angles = range(solar_scan_start_angle, solar_scan_end_angle + 1, solar_scan_step)
        scan_direction = "45->135"
    else:
        scan_angles = range(solar_scan_end_angle, solar_scan_start_angle - 1, -solar_scan_step)
        scan_direction = "135->45"

    print(f"Solar scan start... direction={scan_direction}")
    for angle in scan_angles:
        solar_servo.myServoWriteAngle(angle, solar_scan_speed)
        time.sleep_ms(solar_scan_settle_ms)

        try:
            voltage_v = ina_0x40.read_bus_voltage()
            current_a = ina_0x40.read_shunt_current()
            power_w = voltage_v * current_a
        except Exception as e:
            print("Solar scan read error:", e)
            continue

        if best_power is None or power_w > best_power:
            best_power = power_w
            best_angle = angle
            drop_count = 0
        else:
            drop_count += 1
            if drop_count >= solar_scan_drop_count_limit:
                print("Solar peak passed. Stopping scan early.")
                break

    if best_angle is not None:
        solar_servo.myServoWriteAngle(best_angle, 15)
        print(f"Solar scan done. Best angle={best_angle}, P1={best_power:.3f}W")
    else:
        print("Solar scan failed. Keeping current angle.")

    return best_angle, best_power

try:
    ina_0x40.configure(avg=4, busConvTime=4, shuntConvTime=4, mode=7)
    ina_0x41.configure(avg=4, busConvTime=4, shuntConvTime=4, mode=7)
    ina_0x40.calibrate(rShuntValue=0.1, iMaxExpected=2.0)
    ina_0x41.calibrate(rShuntValue=0.1, iMaxExpected=2.0)
    print("INA226 센서 초기화 완료")
except Exception as e:
    print("INA226 설정 실패:", e)

# Wi-Fi 연결 및 소켓(UDP) 생성
print("하드웨어 전원 안정화 대기 중...")
time.sleep(2)
connect_wifi()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_send_time = time.ticks_ms()
motor_left_command = 0
motor_right_command = 0

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
        
        current_time = time.ticks_ms()
        stop_marker_detected = active_count == 8
        if stop_marker_ignore_until_time is not None:
            if time.ticks_diff(stop_marker_ignore_until_time, current_time) > 0:
                stop_marker_detected = False
            else:
                stop_marker_ignore_until_time = None
        skip_drive_control = False

        if not stop_marker_detected:
            stop_marker_armed = True

        if stop_marker_detected and stop_marker_armed and charging_until_time is None:
            stop_marker_armed = False
            pid_integral = 0
            pid_previous_error = 0
            pid_has_previous_error = False
            lost_line_start_time = None
            motor_left_command = reverse_brake_left_speed
            motor_right_command = reverse_brake_right_speed
            motor.set_speed(motor_left_command, motor_right_command)
            time.sleep_ms(reverse_brake_ms)
            motor_left_command = 0
            motor_right_command = 0
            motor.set_speed(motor_left_command, motor_right_command)
            print("Stop marker detected. Optimizing solar angle...")
            scan_best_solar_angle()
            current_time = time.ticks_ms()
            last_pid_time = current_time
            charging_until_time = time.ticks_add(current_time, charge_stop_ms)
            print("Charging...")

        if charging_until_time is not None:
            if time.ticks_diff(charging_until_time, current_time) > 0:
                pid_integral = 0
                pid_previous_error = 0
                pid_has_previous_error = False
                last_pid_time = current_time
                lost_line_start_time = None
                motor_left_command = 0
                motor_right_command = 0
                motor.set_speed(motor_left_command, motor_right_command)
                skip_drive_control = True
            else:
                charging_until_time = None
                stop_marker_ignore_until_time = time.ticks_add(current_time, stop_marker_ignore_ms)
                print("Charging done. Resuming...")
                
        if not skip_drive_control and active_count > 0:
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
            last_seen_steering = steering
            lost_line_start_time = None

            slowdown = min(abs(error) * slowdown_gain, base_speed - min_speed)
            drive_speed = base_speed - slowdown

            left_speed = clamp(drive_speed + steering, -max_speed, max_speed)
            right_speed = clamp(drive_speed - steering, -max_speed, max_speed)
            motor_left_command = left_speed
            motor_right_command = right_speed
            motor.set_speed(motor_left_command, motor_right_command)
        elif not skip_drive_control:
            current_time = time.ticks_ms()
            if lost_line_start_time is None:
                lost_line_start_time = current_time

            pid_integral = 0
            pid_previous_error = 0
            pid_has_previous_error = False
            last_pid_time = time.ticks_ms()

            lost_time_ms = time.ticks_diff(current_time, lost_line_start_time)
            if lost_time_ms <= fallback_timeout_ms and last_seen_steering != 0:
                if last_seen_steering > 0:
                    turn_direction = 1
                else:
                    turn_direction = -1

                left_speed = clamp(fallback_speed + fallback_turn * turn_direction, -max_speed, max_speed)
                right_speed = clamp(fallback_speed - fallback_turn * turn_direction, -max_speed, max_speed)
                motor_left_command = left_speed
                motor_right_command = right_speed
                motor.set_speed(motor_left_command, motor_right_command)
            else:
                motor_left_command = 0
                motor_right_command = 0
                motor.set_speed(motor_left_command, motor_right_command) 

        # --- B. INA226 데이터 읽기 및 Wi-Fi 송신 (0.5초마다) ---
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_send_time) > 500:
            try:
                v1, ma1 = ina_0x40.read_bus_voltage(), ina_0x40.read_shunt_current()
                v2, ma2 = ina_0x41.read_bus_voltage(), ina_0x41.read_shunt_current()
                
                if None not in (v1, ma1, v2, ma2):
                    if wlan.isconnected():
                        message = f"{v1:.3f},{ma1*1000:.1f},{v2:.3f},{ma2*1000:.1f},{motor_left_command:.1f},{motor_right_command:.1f}"
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
