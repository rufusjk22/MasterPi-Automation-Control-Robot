#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import common.pid as PID
import common.misc as Misc
import common.yaml_handle as yaml_handle
import common.mecanum as mecanum
from kinematics.transform import *

#(3.Advanced Lesson/1.AI Vision Games Lesson/Lesson 4 Line Following)

chassis = mecanum.MecanumChassis()
pitch_pid = PID.PID(P=0.0015, I=0.00001, D=0) #P is set to 0.0015, rest is set to 0

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# line following)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


# (set target color)
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

lab_data = None

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# (initial position)
def initMove():
    board.pwm_servo_set_position(0.2, [[3, 500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[4, 2500]])
    time.sleep(0.5)
    #AK.setPitchRangeMoving((0, 7, 11), -60, -90, 0, 1500)
    #AK.setPitchRangeMoving((0, 7, 11), -60, -90, 0, 200)
    chassis.set_velocity(0,90,0)  # (close all motors)

    
line_centerx = -1
#(reset variables)
def reset():
    global line_centerx
    global __target_color
    
    line_centerx = -1
    __target_color = ()
    
# (call the initialization of the app)
def init():
    print("VisualPatrol Init")
    load_config()
    initMove()

__isRunning = False
# (the app starts the game calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")

# (the app stops the game calling)
def stop():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,90,0)  #(close all motors)
    print("VisualPatrol Stop")

# (the app exits the game calling)
def exit():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,90,0)  # (close all motors)
    print("VisualPatrol Exit")
    
# (find the contour with the largest area)
#(the parameter is the listing of contours to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # (iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # (calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  #(Only the contour with the largest area, which is greater than 300, is considered valid to filter out the interference.)
                area_max_contour = c

    return area_max_contour, contour_area_max  # (return the maximum contour)

img_centerx = 320
line_centerx = -1
def move():
    global line_centerx
    while True:
        if __isRunning:
            if line_centerx >= 0:
                #initially, threshold deadzone for deviation was 25px, we got rid of it, so every pixel is treated for error
                num = line_centerx - img_centerx
                #keep target at 0
                pitch_pid.SetPoint = 0
                pitch_pid.update(num) 
                angle = -1*pitch_pid.output #(obtain the output value of the PID)
                #angle is set to negative specificc for our robot (possible due to other mechanical/ wiring issues)
                print('Angle:', angle)
                chassis.set_velocity(30, 90, angle) #send velocity
                #time.sleep(0.05) #short sleep for instant feedback
                
            else :
                chassis.set_velocity(0,90,0)  #no line ->> stop all motors
                time.sleep(0.02)
        else:
            time.sleep(0.01)
 
#(run a sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

roi = [ # [ROI, weight]
        (240, 280,  0, 640, 0.1), 
        (340, 380,  0, 640, 0.3), 
        (430, 460,  0, 640, 0.6)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)
def run(img):
    global line_centerx
    global __target_color

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning or __target_color == ():
        return img

    MIN_CONTOUR_AREA = 500  # Ignore small blobs (noise)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # Larger kernel

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)  # Stronger blur

    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    for r in roi:
        roi_h = roi_h_list[n]
        n += 1
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)

        combined_mask = None

        # Combine masks for black and red
        for color in lab_data:
            if color in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                         tuple(lab_data[color]['min']),
                                         tuple(lab_data[color]['max']))
                eroded = cv2.erode(frame_mask, kernel, iterations=2)
                dilated = cv2.dilate(eroded, kernel, iterations=2)

                if combined_mask is None:
                    combined_mask = dilated
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, dilated)

        if combined_mask is None:
            continue

        cnts = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
        cnt_large, area = getAreaMaxContour(cnts)

        if cnt_large is not None and area > MIN_CONTOUR_AREA:
            rect = cv2.minAreaRect(cnt_large)
            box = np.intp(cv2.boxPoints(rect))
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)

            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0,0,255), -1)
            center_.append([center_x, center_y])

            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if weight_sum != 0:
        line_centerx = int(centroid_x_sum / weight_sum)
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0,255,255), -1)
    else:
        line_centerx = -1

    return img


#(process before closing)
def Stop(signum, frame):
    global __isRunning
    
    __isRunning = False
    print('å…³é—­ä¸­...')
    chassis.set_velocity(0,90,0)  #(close all motors)

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    # (instantiate the inverse kinematics library)
    AK = ArmIK()
    AK.board = board    

    init()
    start()
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    __target_color = ('black','red')
    while __isRunning:
        ret,img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()



