from decimal import DivisionByZero
from cv2 import circle, moments
import pymurapi as mur
import time
import cv2
import math

auv = mur.mur_init()

def clamp(v, min_value, max_value):
    if v > max_value:
        return max_value
    if v < min_value:
        return min_value
    return v

class PD():
    _kp = 0.0 # коэффициент пропорциональности
    _kd = 0.0 # дифференциальный коэффициент
    _prev_error = 0.0
    _timestamp = 1
    
    def __init__(self):
        pass
    
    def set_p_gain(self, value):
        self._kp = value
    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp)*(error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return output

def keep_depth(depth_to_set):
    try:
        error = auv.get_depth() - depth_to_set
        output = keep_depth.regulator.process(error)
        output = clamp(output, -100, 100)
        auv.set_motor_power(2, output)
        auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD()
        keep_depth.regulator.set_p_gain(80)
        keep_depth.regulator.set_d_gain(50)

def keep_yaw(yaw_to_set, speed):
    def clamp_to180(angle):
        if angle > 180.0:
            return angle - 360
        if angle < -180.0:
            return angle + 360
        return angle

    try:
        error = auv.get_yaw() - yaw_to_set
        error = clamp_to180(error)
        output = keep_yaw.regulator.process(error)
        output = clamp(output, -100, 100)
        auv.set_motor_power(0, clamp(speed-output, 100, -100))
        auv.set_motor_power(1, clamp(speed+output, 100, -100))
    except AttributeError:
        keep_yaw.regulator = PD()
        keep_yaw.regulator.set_p_gain(0.8)
        keep_yaw.regulator.set_d_gain(0.5)


def find_yellow_circle(img):
    image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_min = (20, 50, 50)
    hsv_max = (40, 255, 255)
    image_bin = cv2.inRange(image_hsv, hsv_min, hsv_max)
    cnt, _ = cv2.findContours(image_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if cnt:
        for c in cnt:
            area = cv2.contourArea(c)
            if abs(area) > 300:
                ((_, _), (w, h), _) = cv2.minAreaRect(c)
                (_, _), radius = cv2.minEnclosingCircle(c)

                rectangle_area = w * h
                circle_area = radius**2 * math.pi
                aspect_ratio = w / h

                if 0.9 <= aspect_ratio <= 1.1:
                    if rectangle_area > circle_area:
                        moments = cv2.moments(c)
                        try:
                            x = int(moments["m10"] / moments["m00"])
                            y = int(moments["m01"] / moments["m00"])
                            return True, x, y
                        except DivisionByZero:
                            return False, 0, 0
                    else:
                        continue
                else:
                    continue
    return False, 0, 0

def stab_on_yellow_circle(image):
    found, x, y = find_yellow_circle(image)
    if found:
        x_center = x - 320/2
        y_center = y - 240/2
    
        try:
            output_forward = stab_on_yellow_circle.regulator_forward.process(y_center)
            output_side = stab_on_yellow_circle.regulator_side.process(x_center)
            output_side = clamp(output_side, -50, 50)
            output_forward = clamp(output_forward, -50, 50)
            auv.set_motor_power(0, -output_forward)
            auv.set_motor_power(1, -output_forward)

            auv.set_motor_power(4, -output_side)

        except AttributeError:
            stab_on_yellow_circle.regulator_forward = PD()
            stab_on_yellow_circle.regulator_forward.set_p_gain(0.8)
            stab_on_yellow_circle.regulator_forward.set_d_gain(0.5)

            stab_on_yellow_circle.regulator_side = PD()
            stab_on_yellow_circle.regulator_side.set_p_gain(0.8)
            stab_on_yellow_circle.regulator_side.set_d_gain(0.5)




while True: 
    image = auv.get_image_bottom()
    stab_on_yellow_circle(image)
    time.sleep(0.03)
    
    
    
