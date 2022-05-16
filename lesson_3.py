import pymurapi as mur
import cv2

auv = mur.mur_init()

while True:
    depth = auv.get_depth()
    print(depth)
    
    yaw = auv.get_yaw()
    print(yaw)
    
    image = auv.get_image_front()
    cv2.imshow("", image)
    cv2.waitKey(10)
    
    # -100   100
    auv.set_motor_power(0, 50)
