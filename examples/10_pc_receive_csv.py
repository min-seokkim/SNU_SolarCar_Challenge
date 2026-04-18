import socket
import datetime
import csv
import time
import os

# --- 1. 통신 설정 ---
UDP_IP = "0.0.0.0" 
UDP_PORT = 5005
FILE_NAME = "sensor_data.csv"
CSV_HEADER = [
    'Timestamp',
    'SolarInput_V(V)',
    'SolarInput_I(mA)',
    'SolarInput_P(W)',
    'SolarInput_Energy_mWh',
    'MotorBattery_V(V)',
    'MotorBattery_I(mA)',
    'MotorOutput_P(W)',
    'MotorOutput_Energy_mWh',
    'NetPowerUsage(W)',
    'NetEnergyUsage_mWh',
    'LeftCmd',
    'RightCmd',
]

# --- 2. CSV 파일 초기화 (머리글 작성) ---
if not os.path.exists(FILE_NAME):
    with open(FILE_NAME, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)

# --- 3. 소켓 설정 ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"포트 {UDP_PORT}에서 수신 및 CSV 저장 시작...")
print("-" * 60)

# 시간을 재기 위한 변수
last_save_time = 0
last_energy_time = None
solar_input_energy_wh = 0.0
motor_output_energy_wh = 0.0

try:
    while True:
        data, addr = sock.recvfrom(1024)
        
        current_time = time.time()
        
        if current_time - last_save_time >= 1:  # 1초마다 저장 (1분에 한 번으로 변경하려면 60으로 수정)
            message = data.decode('utf-8')
            
            try:
                # 수신 데이터 파싱
                parts = message.split(',')
                if len(parts) == 6:
                    v1_s, c1_s, v2_s, c2_s, left_cmd_s, right_cmd_s = parts
                elif len(parts) == 4:
                    v1_s, c1_s, v2_s, c2_s = parts
                    left_cmd_s, right_cmd_s = "", ""
                else:
                    raise ValueError
                
                # 숫자 변환 및 mA 단위 변환
                v1, c1 = float(v1_s), float(c1_s)
                v2, c2 = float(v2_s), float(c2_s)
                left_cmd = float(left_cmd_s) if left_cmd_s else None
                right_cmd = float(right_cmd_s) if right_cmd_s else None
                solar_input_w = v1 * (c1 / 1000)
                motor_output_w = v2 * (c2 / 1000)

                if last_energy_time is not None:
                    dt_hours = (current_time - last_energy_time) / 3600
                    solar_input_energy_wh += solar_input_w * dt_hours
                    motor_output_energy_wh += motor_output_w * dt_hours
                last_energy_time = current_time
                net_power_usage_w = motor_output_w - solar_input_w
                net_energy_usage_wh = motor_output_energy_wh - solar_input_energy_wh
                solar_input_energy_mwh = solar_input_energy_wh * 1000
                motor_output_energy_mwh = motor_output_energy_wh * 1000
                net_energy_usage_mwh = net_energy_usage_wh * 1000
                
                # 현재 시간
                now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                # --- 4. CSV 파일에 데이터 추가 ---
                with open(FILE_NAME, mode='a', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        now_str,
                        f"{v1:.3f}",
                        f"{c1:.1f}",
                        f"{solar_input_w:.4f}",
                        f"{solar_input_energy_mwh:.3f}",
                        f"{v2:.3f}",
                        f"{c2:.1f}",
                        f"{motor_output_w:.4f}",
                        f"{motor_output_energy_mwh:.3f}",
                        f"{net_power_usage_w:.4f}",
                        f"{net_energy_usage_mwh:.3f}",
                        "" if left_cmd is None else f"{left_cmd:.1f}",
                        "" if right_cmd is None else f"{right_cmd:.1f}",
                    ])
                
                # 화면 출력
                if left_cmd is None or right_cmd is None:
                    print(f"[{now_str}] Saved: Solar={v1:.2f}V/{c1:.1f}mA/{solar_input_w:.3f}W, MotorBattery={v2:.2f}V/{c2:.1f}mA/{motor_output_w:.3f}W, Net={net_energy_usage_mwh:.3f}mWh")
                else:
                    print(f"[{now_str}] Saved: Solar={v1:.2f}V/{c1:.1f}mA/{solar_input_w:.3f}W, MotorBattery={v2:.2f}V/{c2:.1f}mA/{motor_output_w:.3f}W, Net={net_energy_usage_mwh:.3f}mWh, L/R={left_cmd:.1f}/{right_cmd:.1f}")
                
                # 저장을 완료했으므로 타이머를 현재 시간으로 리셋
                last_save_time = current_time
                
            except ValueError:
                print(f"잘못된 형식 수신: {message}")

except KeyboardInterrupt:
    print("\n데이터 로깅을 종료합니다.")
finally:
    sock.close()
