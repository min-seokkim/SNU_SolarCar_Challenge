from mux04 import LineSensor
import time

sensor = LineSensor()

while True:
    channels = sensor.read_channels()
    
    weights = [-55, -30, -20, -10, 10, 20, 30, 55]
    
    error_sum = 0
    active_count = 0
    
    for i in range(8):
        if channels[i]==1:	#라인을 감지했을 때
            error_sum += weights[i]
            active_count += 1
            print("channels : ", channels)
            print("error_sum : ", error_sum)
            print("------------------------------")
    #라인을 완전히 벗어난 경우
    if active_count == 0:
        print("No Line!")
    time.sleep(0.1)

