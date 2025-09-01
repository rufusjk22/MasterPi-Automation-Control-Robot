#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/MasterPi/')
import cv2
import time
import math
import signal
import threading
import numpy as np
import common.pid as PID
import common.misc as Misc
import common.yaml_handle as yaml_handle
import common.mecanum as mecanum
from kinematics.transform import *

# This is our final version of the script that we ran for our contest run.
# Since we couldnt fully make the triangle task to work, we commented out our attempt at it throughout the script.

chassis = mecanum.MecanumChassis()
pitch_pid = PID.PID(P=0.0015, I=0, D=0) 
img_centerx = 320
held_color= None
detected_drop_color=None
in_drop_zone=None
lab_data = None
line_centerx = -1
__isRunning = False
# triangle_direction = None
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def open_gripper(): board.pwm_servo_set_position(0.2, [[1, 2000]])

def close_gripper(): board.pwm_servo_set_position(0.2, [[1, 900]])

def beep(n):
    for _ in range(n):
        board.set_buzzer(1900, 0.1, 0.9, 1)
        time.sleep(0.2) 
    initHold()      
    open_gripper() 
    time.sleep(2)
    close_gripper()

def find_cube(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thr = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 3500:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if 0.8 < w / float(h) < 1.2:
            return c
    return None

def detect_colour(frame, contour):
    x, y, w, h = cv2.boundingRect(contour)
    cx, cy = x + w // 2, y + h // 2
    patch = frame[cy - 15 : cy + 15, cx - 15 : cx + 15]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    h_val = np.mean(hsv[:, :, 0])
    if (0 <= h_val <= 10) or (170 <= h_val <= 180):
        return "red"
    if 50 <= h_val <= 85:
        return "green"
    if 95 <= h_val <= 130:
        return "blue"
    return "unknown"

def grab_and_confirm_colour():
    cap = cv2.VideoCapture("http://127.0.0.1:8080?action=stream")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        cnt = find_cube(frame)
        if cnt is None:
            cv2.imshow("Waiting for cube", cv2.resize(frame, (320, 240)))
            print("Waiting for cube")
            if cv2.waitKey(1) == 27:
                break
            continue
        colour = detect_colour(frame, cnt)
        if colour == "red":
            print("Red")
            board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
            beep(1)
            break
        if colour == "green":
            print("Green")
            board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
            beep(2)
            break
        if colour == "blue":
            print("Blue")
            board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
            beep(3)
            break
    cap.release()
    cv2.destroyAllWindows()
    return colour

def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

def initHold():
    board.pwm_servo_set_position(0.2, [[1, 1500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[3, 1500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[4, 1500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[5, 1500]])
    time.sleep(0.5)
    chassis.set_velocity(0,90,0)  

def initMove():
    board.pwm_servo_set_position(0.2, [[1, 1500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[3, 500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[4, 2500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[5, 1340]])
    time.sleep(0.5)
    chassis.set_velocity(0,90,0)  

def reset():
    global line_centerx, __target_color  
    line_centerx = -1
    __target_color = ()
    
def init():
    print("VisualPatrol Init")
    load_config()
    initMove()
    
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")

def exit():
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0,90,0)  
    print("VisualPatrol Exit")

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

def move():
    global line_centerx, held_color, detected_drop_color
    while True:
        if __isRunning:
            if detected_drop_color == held_color:
                chassis.set_velocity(30,90,0)
                time.sleep(0.3)
                chassis.set_velocity(30,180,0) 
                time.sleep(0.5)
                open_gripper()
                chassis.set_velocity(30,0,0)
                time.sleep(0.8)
                chassis.set_velocity(30,90,0)
                time.sleep(1)
                print("Cube Dropped")
            elif line_centerx >= 0:
                #initially, threshold deadzone for deviation was 25px, we got rid of it, so every pixel is treated for error
                num = line_centerx - img_centerx
                #keep target at 0
                pitch_pid.SetPoint = 0
                pitch_pid.update(num) 
                angle = -1*pitch_pid.output #(obtain the output value of the PID)
                #angle is set to negative specific for our robot (possible due to other mechanical/ wiring issues)
                chassis.set_velocity(30, 90, angle) #send velocity
                time.sleep(0.05) #short sleep for instant feedback    
                # This area of code was used to react upon the triangle direction which was decided by the function
                # if triangle_direction:
                #     if triangle_direction == "left":
                #         print('Left turning now')
                #         chassis.set_velocity(30, 90, -0.05)  # slight yaw left
                #         time.sleep(0.5)
                #     elif triangle_direction == "right":
                #         print('Right turning now')
                #         chassis.set_velocity(30, 90, 0.05)  # slight yaw right
                #         time.sleep(0.7)
                #     elif triangle_direction == "straight":
                #         print('Straight')  
            else :
                chassis.set_velocity(0,90,0)  #no line ->> stop all motors
                time.sleep(0.2)
        else:
            time.sleep(0.05)
 
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
    global line_centerx, __target_color, in_drop_zone, detected_drop_color, held_color

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning or __target_color == ():
        return img

    MIN_CONTOUR_AREA = 500  # Ignore small blobs (noise)
    DROP_ZONE_WIDTH_THRESHOLD = 90
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
        # Combine masks for any color
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
            
            x, y, w, h = cv2.boundingRect(cnt_large)
            if w >= DROP_ZONE_WIDTH_THRESHOLD:
                in_drop_zone = True
                drop_roi = frame_resize[y:y+h, x:x+w]  # local region
                avg_color = cv2.mean(drop_roi)[:3]  # BGR
                r = avg_color[2]
                g = avg_color[1]
                b = avg_color[0]
                detected_drop_color = None
                if r > g + 30 and r > b + 30 and r > 80:
                    detected_drop_color = 'red'
                elif g > r + 30 and g > b + 30 and g > 80:
                    detected_drop_color = 'green'
                elif b > r + 30 and b > g + 30 and b > 80:
                    detected_drop_color = 'blue'
                print(detected_drop_color)

    if weight_sum != 0:
        line_centerx = int(centroid_x_sum / weight_sum)
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0,255,255), -1)
    else:
        line_centerx = -1
    #Call the function once every frame
    #triangle_direction = detect_triangle_and_direction(img_copy)
    return img

def Stop(signum, frame):
    global __isRunning
    print('Stopping...')
    __isRunning = False
    chassis.set_velocity(0,90,0)  

#The logic is sound but we were having issues of false detection. It detecting a triangle at corners for example. 
#Playing around with the threshold did make it better but there was still this jitter because the computation load was very high
# def detect_triangle_and_direction(frame):
#     direction = None
#     # Convert to grayscale
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     # Threshold to isolate black (triangle)
#     _, thresh = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)
#     # Find contours
#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     for contour in contours:
#         area = cv2.contourArea(contour)
#         if area < 4500:  # Filter small areas
#             continue
#         # Approximate the contour
#         peri = cv2.arcLength(contour, True)
#         approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
#         if len(approx) == 3:
#             pts = [tuple(pt[0]) for pt in approx]
#             def distance(p1, p2):
#                 return np.linalg.norm(np.array(p1) - np.array(p2))
#             # Compute side lengths
#             d01 = distance(pts[0], pts[1])
#             d12 = distance(pts[1], pts[2])
#             d20 = distance(pts[2], pts[0])
#             # Identify base and tip
#             if d01 >= d12 and d01 >= d20:
#                 base = (pts[0], pts[1])
#                 tip = pts[2]
#             elif d12 >= d01 and d12 >= d20:
#                 base = (pts[1], pts[2])
#                 tip = pts[0]
#             else:
#                 base = (pts[2], pts[0])
#                 tip = pts[1]
#             base_mid_x = (base[0][0] + base[1][0]) / 2
#             base_mid_y = (base[0][1] + base[1][1]) / 2
#             dx = tip[0] - base_mid_x
#             dy = tip[1] - base_mid_y
#             # Classify triangle direction
#             if abs(dx) > abs(dy):
#                 direction = "right" if dx > 0 else "left"
#             else:
#                 direction = "straight" if dy < 0 else "downward"  # downward is unused              
#             # Optional debug drawing
#             cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
#             cv2.circle(frame, tip, 7, (0, 0, 255), -1)
#             cv2.putText(frame, f"{direction}", (tip[0]-30, tip[1]-10),
#             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#             break  # Only process one triangle
#     return direction

if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board    
    held_color = grab_and_confirm_colour()
    time.sleep(3)
    init()
    start()
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    __target_color = ('black','red','green','blue')
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