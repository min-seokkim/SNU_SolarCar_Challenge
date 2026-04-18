from drv8833 import DRV8833
import time

motor = DRV8833()

# 동작 테스트
motor.set_speed(50, 50) # 전진
time.sleep(3)			# 3초 대기
motor.set_speed(0,0)	#정지

