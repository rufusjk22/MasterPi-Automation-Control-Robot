#!/usr/bin/python3
# -*- coding: utf‑8 -*-

# basic libs
import sys, cv2, time, math, signal, threading, numpy as np
sys.path.append('/home/pi/MasterPi/')          # add robot libraries to path

# robot‑specific libs
import Camera
import common.pid   as PID                    # PID helper
import common.misc  as Misc                   # misc math functions
import common.yaml_handle as yaml_handle      # loads LAB colour data
import common.mecanum as mecanum              # wheel control
from kinematics.transform import *            # arm transformations

chassis   = mecanum.MecanumChassis()          # talk to expansion board
pitch_pid = PID.PID(P=0.0015, I=0.00001, D=0) # steering controller

# colours only for drawing
range_rgb = {'red':(0,0,255),'blue':(255,0,0),
             'green':(0,255,0),'black':(0,0,0),'white':(255,255,255)}

# make sure Python 3 is used
if sys.version_info.major == 2:
    print('Please run this with python3')
    sys.exit(0)

# WonderPi can call this to change line colour
def setTargetColor(c):
    global __target_color
    print("COLOR", c)
    __target_color = c
    return True, ()

# read LAB thresholds from YAML
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

# put servos and wheels in safe pose
def initMove():
    board.pwm_servo_set_position(0.2, [[1,1500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[3, 500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[4,2500]])
    time.sleep(0.5)
    board.pwm_servo_set_position(0.2, [[5,1341]])
    time.sleep(0.5)
    chassis.set_velocity(0, 90, 0)            # stop wheels

# reset globals before each run
line_centerx = -1
__target_color = ()
def reset():
    global line_centerx, __target_color
    line_centerx, __target_color = -1, ()

def init():  load_config(); initMove()        # called once on start

__isRunning = False
def start(): global __isRunning; reset(); __isRunning = True
def stop():  global __isRunning; __isRunning = False; chassis.set_velocity(0,90,0)
exit = stop                                   # same clean‑up

# pick the biggest contour
def getAreaMaxContour(cnts):
    best, area_max = None, 0
    for c in cnts:
        a = abs(cv2.contourArea(c))
        if a > area_max and a >= 5:
            best, area_max = c, a
    return best, area_max

# background thread that sends wheel commands
img_centerx = 320
def move():
    global line_centerx
    while True:
        if __isRunning:
            if line_centerx >= 0:
                error = line_centerx - img_centerx   # pixels to left/right
                pitch_pid.SetPoint = 0
                pitch_pid.update(error)
                yaw = -pitch_pid.output              # turn toward line
                print('Angle:', yaw)
                chassis.set_velocity(30, 90, yaw)    # drive forward + yaw
                time.sleep(0.05)
            else:
                chassis.set_velocity(0, 90, 0)       # stop if line lost
                time.sleep(0.02)
        else:
            time.sleep(0.01)

threading.Thread(target=move, daemon=True).start()

# three horizontal stripes we look at
roi = [(240,280,0,640,0.1),(340,380,0,640,0.3),(430,460,0,640,0.6)]
roi_h_list = [roi[0][0], roi[1][0]-roi[0][0], roi[2][0]-roi[1][0]]
size = (640,480)

# vision processing called every frame
def run(img):
    global line_centerx
    if not __isRunning or __target_color == (): return img

    frame = cv2.GaussianBlur(cv2.resize(img.copy(), size),(3,3),3)
    cx_sum, w_sum, n = 0, 0, 0

    for r in roi:
        roi_h = roi_h_list[n]; n += 1
        slice_lab = cv2.cvtColor(frame[r[0]:r[1], r[2]:r[3]], cv2.COLOR_BGR2LAB)

        mask = None
        for col in lab_data:                      # combine masks for each colour
            if col in __target_color:
                m = cv2.inRange(slice_lab,
                                tuple(lab_data[col]['min']),
                                tuple(lab_data[col]['max']))
                m = cv2.erode(m, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
                m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
        if mask is None: continue

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
        cnt, area = getAreaMaxContour(cnts)
        if cnt is None: continue

        rect = cv2.minAreaRect(cnt); box = np.intp(cv2.boxPoints(rect))
        for i in range(4):                       # map ROI coords to full frame
            box[i,1] = int(Misc.map(box[i,1] + (n-1)*roi_h + roi[0][0],0,size[1],0,img.shape[0]))
            box[i,0] = int(Misc.map(box[i,0], 0,size[0],0,img.shape[1]))
        cv2.drawContours(img,[box],-1,(0,0,255),2)

        cx = (box[0,0] + box[2,0]) / 2           # centre x of blob
        cv2.circle(img,(int(cx),int(box[0,1])),5,(0,0,255),-1)
        cx_sum += cx * r[4]; w_sum += r[4]

    line_centerx = int(cx_sum / w_sum) if w_sum else -1
    if w_sum:
        cv2.circle(img,(line_centerx,int(box[0,1])),10,(0,255,255),-1)
    return img

# clean exit on Ctrl‑C
def Stop(sig,frm):
    global __isRunning
    __isRunning = False
    chassis.set_velocity(0, 90, 0)

# main program
if __name__ == '__main__':
    from kinematics.arm_move_ik           import ArmIK
    from common.ros_robot_controller_sdk import Board
    board = Board();  AK = ArmIK(); AK.board = board

    init(); start()
    signal.signal(signal.SIGINT, Stop)
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    __target_color = ('black','red')        # colours to track

    while __isRunning:
        ok, img = cap.read()
        if ok:
            cv2.imshow('frame', cv2.resize(run(img),(320,240)))
            if cv2.waitKey(1) == 27: break  # Esc quits
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
