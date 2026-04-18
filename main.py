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
wifi_retry_ms = 5000
wifi_last_attempt_time = None
wifi_connected_logged = False
wifi_configured = False

def connect_wifi():
    return start_wifi_connect()

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

def start_wifi_connect():
    global wifi_last_attempt_time, wifi_connected_logged, wifi_configured

    try:
        wlan.active(True)
    except Exception as e:
        print("Wi-Fi active error:", e)
        return False

    if not wifi_configured:
        try:
            wlan.config(txpower=10)
            print("TX Power set to 10")
        except Exception as e:
            print("TX Power config skipped:", e)
        wifi_configured = True

    if wlan.isconnected():
        if not wifi_connected_logged:
            print("Wi-Fi connected! IP:", wlan.ifconfig()[0])
            wifi_connected_logged = True
        return True

    current_time = time.ticks_ms()
    if wifi_last_attempt_time is None or time.ticks_diff(current_time, wifi_last_attempt_time) >= wifi_retry_ms:
        wifi_last_attempt_time = current_time
        wifi_connected_logged = False
        print(f"{SSID} Wi-Fi connect attempt...")
        try:
            wlan.connect(SSID, PASSWORD)
        except Exception as e:
            print("Wi-Fi connect attempt failed:", e)

    return False

def service_wifi():
    global wifi_connected_logged

    if wlan.isconnected():
        if not wifi_connected_logged:
            print("Wi-Fi connected! IP:", wlan.ifconfig()[0])
            wifi_connected_logged = True
        return True

    wifi_connected_logged = False
    return start_wifi_connect()

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
base_speed = 95
max_speed = 100
min_speed = 50
slowdown_gain = 0.75
integral_limit = 100
fallback_timeout_ms = 1000
fallback_speed = 35
fallback_turn = 55
reverse_brake_ms = 200
reverse_brake_left_speed = -30
reverse_brake_right_speed = -40
charge_station_stop_enabled = True
charge_stop_ms = 60000
charge_min_stop_ms = 0
charge_max_stop_ms = 120000
race_duration_ms = 20 * 60 * 1000
estimated_solar_scan_ms = 5000
charge_finish_margin_ms = 5000
charge_request_deadband_ms = 60000
battery_full_voltage_v = 4.20
battery_cutoff_voltage_v = 2.80
battery_nominal_voltage_v = 3.70
battery_capacity_ah = 0.110
usable_battery_energy_wh = battery_nominal_voltage_v * battery_capacity_ah
battery_internal_resistance_ohm = 1.00
battery_ocv_soc_curve = (
    (2.80, 0.00),
    (3.00, 0.00),
    (3.30, 0.02),
    (3.45, 0.05),
    (3.58, 0.10),
    (3.63, 0.20),
    (3.70, 0.30),
    (3.75, 0.40),
    (3.79, 0.50),
    (3.83, 0.60),
    (3.91, 0.70),
    (4.00, 0.80),
    (4.10, 0.90),
    (4.20, 1.00),
)
solar_charge_power_w = 0.30
motor_power_intercept_w = 0.425
motor_power_slope_w_per_cmd = 0.00875
stop_marker_ignore_ms = 500
solar_scan_start_angle = 45
solar_scan_center_angle = 90
solar_scan_end_angle = 135
solar_scan_step = 1
solar_scan_low_power_step = 6
solar_scan_speed = 40
solar_scan_settle_ms = 40
solar_scan_power_baseline_w = 0.15
solar_scan_drop_count_limit = 4
battery_voltage_filter_alpha = 0.25
voltage_compensation_ref_v = 3.70
voltage_compensation_min_v = 2.80
pid_integral = 0
pid_previous_error = 0
pid_has_previous_error = False
last_pid_time = time.ticks_ms()
last_seen_steering = 0
lost_line_start_time = None
stop_marker_armed = True
charging_until_time = None
stop_marker_ignore_until_time = None
battery_voltage_filtered_v = None
last_motor_current_a = 0

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def estimate_battery_ocv(terminal_voltage_v, motor_current_a):
    if terminal_voltage_v is None:
        return None

    if motor_current_a is None:
        motor_current_a = 0

    discharge_current_a = abs(motor_current_a)
    ocv_v = terminal_voltage_v + discharge_current_a * battery_internal_resistance_ohm
    return clamp(ocv_v, battery_cutoff_voltage_v, battery_full_voltage_v)

def update_battery_voltage(voltage_v, motor_current_a=0):
    global battery_voltage_filtered_v

    if voltage_v is None:
        return battery_voltage_filtered_v

    voltage_v = estimate_battery_ocv(voltage_v, motor_current_a)
    if voltage_v is None:
        return battery_voltage_filtered_v

    if battery_voltage_filtered_v is None:
        battery_voltage_filtered_v = voltage_v
    else:
        battery_voltage_filtered_v = (
            battery_voltage_filter_alpha * voltage_v
            + (1 - battery_voltage_filter_alpha) * battery_voltage_filtered_v
        )

    return battery_voltage_filtered_v

def read_battery_measurement():
    try:
        return ina_0x41.read_bus_voltage(), ina_0x41.read_shunt_current()
    except Exception as e:
        print("Battery read error:", e)
        return None, None

def station_policy_voltage(raw_voltage_v, motor_current_a):
    raw_ocv_v = estimate_battery_ocv(raw_voltage_v, motor_current_a)
    if battery_voltage_filtered_v is None:
        return raw_ocv_v
    if raw_ocv_v is None:
        return battery_voltage_filtered_v
    return min(battery_voltage_filtered_v, raw_ocv_v)

def motor_power_for_command(command):
    return motor_power_intercept_w + motor_power_slope_w_per_cmd * command

def battery_soc_from_ocv(ocv_v):
    if ocv_v is None:
        return None

    ocv_v = clamp(ocv_v, battery_cutoff_voltage_v, battery_full_voltage_v)
    previous_voltage, previous_soc = battery_ocv_soc_curve[0]

    if ocv_v <= previous_voltage:
        return previous_soc

    for i in range(1, len(battery_ocv_soc_curve)):
        next_voltage, next_soc = battery_ocv_soc_curve[i]
        if ocv_v <= next_voltage:
            voltage_span = next_voltage - previous_voltage
            if voltage_span <= 0:
                return next_soc
            ratio = (ocv_v - previous_voltage) / voltage_span
            return previous_soc + ratio * (next_soc - previous_soc)

        previous_voltage = next_voltage
        previous_soc = next_soc

    return battery_ocv_soc_curve[-1][1]

def battery_energy_remaining_wh(voltage_v):
    if voltage_v is None:
        return None

    soc = battery_soc_from_ocv(voltage_v)
    if soc is None:
        return None

    return usable_battery_energy_wh * soc

def race_remaining_ms(current_time):
    elapsed_ms = time.ticks_diff(current_time, race_start_time)
    remaining_ms = race_duration_ms - elapsed_ms
    return int(clamp(remaining_ms, 0, race_duration_ms))

def charge_duration_for_voltage(voltage_v, current_time):
    remaining_ms = race_remaining_ms(current_time)
    max_charge_ms = remaining_ms - estimated_solar_scan_ms - charge_finish_margin_ms
    if max_charge_ms <= 0:
        return 0

    max_charge_ms = int(clamp(max_charge_ms, 0, charge_max_stop_ms))
    if voltage_v is None:
        return int(clamp(charge_stop_ms, 0, max_charge_ms))

    energy_remaining_wh = battery_energy_remaining_wh(voltage_v)
    if energy_remaining_wh is None:
        return int(clamp(charge_stop_ms, 0, max_charge_ms))

    available_after_scan_h = (
        (remaining_ms - estimated_solar_scan_ms)
        / 3600000
    )
    motor_power_w = motor_power_for_command(base_speed)
    needed_energy_wh = motor_power_w * available_after_scan_h - energy_remaining_wh
    if needed_energy_wh <= 0:
        return 0

    charge_ms = int(
        needed_energy_wh
        / (motor_power_w + solar_charge_power_w)
        * 3600000
    )
    if charge_ms <= charge_request_deadband_ms:
        return 0

    charge_ms = int(clamp(charge_ms, charge_min_stop_ms, max_charge_ms))
    return charge_ms

def drive_base_speed_for_voltage(voltage_v):
    if voltage_v is None:
        return base_speed

    voltage_v = max(voltage_v, voltage_compensation_min_v)
    if voltage_v >= voltage_compensation_ref_v:
        return base_speed

    compensated_speed = base_speed * voltage_compensation_ref_v / voltage_v
    return clamp(compensated_speed, base_speed, max_speed)

def charge_remaining_seconds(current_time):
    if charging_until_time is None:
        return 0

    remaining_ms = time.ticks_diff(charging_until_time, current_time)
    if remaining_ms <= 0:
        return 0

    return remaining_ms / 1000

def format_voltage(voltage_v):
    if voltage_v is None:
        return "unknown"
    return f"{voltage_v:.2f}V"

def scan_best_solar_angle():
    best_angle = None
    best_power = None
    drop_count = 0

    if solar_servo.current_angle < solar_scan_center_angle:
        angle = solar_scan_start_angle
        scan_end_angle = solar_scan_end_angle
        scan_direction_step = 1
        scan_direction = "45->135"
    else:
        angle = solar_scan_end_angle
        scan_end_angle = solar_scan_start_angle
        scan_direction_step = -1
        scan_direction = "135->45"

    print(f"Solar scan start... direction={scan_direction}")
    while (scan_direction_step > 0 and angle <= scan_end_angle) or \
          (scan_direction_step < 0 and angle >= scan_end_angle):
        solar_servo.myServoWriteAngle(angle, solar_scan_speed)
        time.sleep_ms(solar_scan_settle_ms)

        try:
            voltage_v = ina_0x40.read_bus_voltage()
            current_a = ina_0x40.read_shunt_current()
            power_w = voltage_v * current_a
        except Exception as e:
            print("Solar scan read error:", e)
            continue

        if power_w <= solar_scan_power_baseline_w:
            angle += solar_scan_low_power_step * scan_direction_step
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

        angle += solar_scan_step * scan_direction_step

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
start_wifi_connect()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_send_time = time.ticks_ms()
race_start_time = time.ticks_ms()
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
        if not charge_station_stop_enabled:
            stop_marker_detected = False
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
            raw_battery_voltage, raw_motor_current = read_battery_measurement()
            if raw_motor_current is None:
                raw_motor_current = last_motor_current_a
            if raw_battery_voltage is not None:
                update_battery_voltage(raw_battery_voltage, raw_motor_current)
            policy_voltage = station_policy_voltage(raw_battery_voltage, raw_motor_current)
            charge_duration_ms = charge_duration_for_voltage(policy_voltage, current_time)

            if charge_duration_ms <= 0:
                stop_marker_ignore_until_time = time.ticks_add(current_time, stop_marker_ignore_ms)
                print(f"Stop marker skipped. Battery={format_voltage(policy_voltage)}")
            else:
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
                print(f"Stop marker detected. Battery={format_voltage(policy_voltage)}. Optimizing solar angle...")
                scan_best_solar_angle()
                current_time = time.ticks_ms()
                last_pid_time = current_time
                charging_until_time = time.ticks_add(current_time, charge_duration_ms)
                print(f"Charging for {charge_duration_ms // 1000}s...")

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

            target_base_speed = drive_base_speed_for_voltage(battery_voltage_filtered_v)
            target_base_speed = clamp(target_base_speed, min_speed, max_speed)
            slowdown = min(abs(error) * slowdown_gain, target_base_speed - min_speed)
            drive_speed = target_base_speed - slowdown

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
            wifi_ready = service_wifi()
            try:
                v1, ma1 = ina_0x40.read_bus_voltage(), ina_0x40.read_shunt_current()
                v2, ma2 = ina_0x41.read_bus_voltage(), ina_0x41.read_shunt_current()
                
                if None not in (v1, ma1, v2, ma2):
                    last_motor_current_a = ma2
                    update_battery_voltage(v2, ma2)
                    charge_remaining_s = charge_remaining_seconds(current_time)
                    if wifi_ready:
                        message = f"{v1:.3f},{ma1*1000:.1f},{v2:.3f},{ma2*1000:.1f},{motor_left_command:.1f},{motor_right_command:.1f},{charge_remaining_s:.1f}"
                        sock.sendto(message.encode(), (PC_IP, PC_PORT))
                        print(f"전송 -> 0x40: {v1:.2f}V/{ma1*1000:.0f}mA | 0x41: {v2:.2f}V/{ma2*1000:.0f}mA | ChargeLeft: {charge_remaining_s:.1f}s")
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
