import pymurapi as mur
import time


class PD():
    _kp = 0.0 # коэффициент пропорциональности
    _kd = 0.0 # дифференциальный коэффициент
    _prev_error = 0.0
    _timestamp = 0
    
    def __init__(self):
        pass
    
    def set_p_gain(self, value):
        self._kp = value
    def set_d_gain(self, value):
        self._kd = value
    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp)*(error - self._prev_error)

auv = mur.mur_init()

prev_time = 0
prev_error = 0

def clamp_to180(angle):
    if angle > 180.0:
        return angle - 360
    if angle < -180.0:
        return angle + 360
    return angle

def clamp(v, max_v, min_v):
    if v > max_v:
        return max_v
    if v < min_v:
        return min_v
    return v

def keep_yaw(value):
    global prev_time
    global prev_error
    current_time = int(round(time.time() * 1000))
    
    error = auv.get_yaw() - value
    error = clamp_to180(error)
    power_0 = 0
    power_1 = 0
    
    power_value = error * 0.8 # 70 пропорциональный коэффициент
    diff_value = 0.5 / (current_time - prev_time) * (error - prev_error) # 5 дифференциальный коэффициент 
    
    power_0 = clamp(power_value + diff_value, 100, -100)
    power_1 = clamp(power_value + diff_value, 100, -100)
    
    auv.set_motor_power(0, -power_0)
    auv.set_motor_power(1, power_1)
    
    prev_time = current_time
    prev_error = error
    
    

while True:
    keep_yaw(2)
    time.sleep(0.03)
    
    
    
