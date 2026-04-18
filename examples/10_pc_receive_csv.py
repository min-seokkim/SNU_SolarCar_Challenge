import socket
import datetime
import csv
import time
import os

# --- 1. 통신 설정 ---
UDP_IP = "0.0.0.0" 
UDP_PORT = 5005
FILE_NAME = "sensor_data.csv"

# --- 2. CSV 파일 초기화 (머리글 작성) ---
if not os.path.exists(FILE_NAME):
    with open(FILE_NAME, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'V1(V)', 'I1(mA)', 'V2(V)', 'I2(mA)'])

# --- 3. 소켓 설정 ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"포트 {UDP_PORT}에서 수신 및 CSV 저장 시작...")
print("-" * 60)

# 시간을 재기 위한 변수
last_save_time = 0

try:
    while True:
        data, addr = sock.recvfrom(1024)
        
        current_time = time.time()
        
        if current_time - last_save_time >= 1:  # 1초마다 저장 (1분에 한 번으로 변경하려면 60으로 수정)
            message = data.decode('utf-8')
            
            try:
                # 수신 데이터 파싱
                v1_s, c1_s, v2_s, c2_s = message.split(',')
                
                # 숫자 변환 및 mA 단위 변환
                v1, c1 = float(v1_s), float(c1_s)
                v2, c2 = float(v2_s), float(c2_s)
                
                # 현재 시간
                now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                # --- 4. CSV 파일에 데이터 추가 ---
                with open(FILE_NAME, mode='a', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow([now_str, f"{v1:.3f}", f"{c1:.1f}", f"{v2:.3f}", f"{c2:.1f}"])
                
                # 화면 출력
                print(f"[{now_str}] Saved: A={v1:.2f}V/{c1:.1f}mA, B={v2:.2f}V/{c2:.1f}mA")
                
                # 저장을 완료했으므로 타이머를 현재 시간으로 리셋
                last_save_time = current_time
                
            except ValueError:
                print(f"잘못된 형식 수신: {message}")

except KeyboardInterrupt:
    print("\n데이터 로깅을 종료합니다.")
finally:
    sock.close()
