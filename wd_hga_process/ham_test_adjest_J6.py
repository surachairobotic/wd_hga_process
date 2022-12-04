import time, keyboard, math, requests, copy, threading
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *

import matplotlib.pyplot as plt
from PIL import Image, ImageFont, ImageDraw
import torch
import pandas as pd
from hsv_detection import *

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"
tip_speed = 0.25
high_offset=0.421
adj_hight=0
enable_grip = True

model_red=torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/red_tiny.pt', source='local')

#model=torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_top_TINY.pt', source='local')
model2=torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_jig_TINY.pt', source='local')

frame_detect = None
iThreadRun = 0
x_axis_offset=0.001289
y_axix_offset=0.001212
robot = UR_SOCKET()
pos=[]
tip=[]
Tool=[0,0,0,0,0,0]
tResetDetector = time.time()
ur_rtde = UR_INFORMATION()
state=0
hsv_debug = False
adject_break=False
def open_camera():
    global frame_detect, iThreadRun, tResetDetector, pos, tip ,Tool,adj_hight
    cnt=0
    print('open_camera')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    print(cnt)
    cnt+=1
    host_ip = '192.168.12.250' # paste your server ip address here
    port = 5678
    print(cnt)
    cnt+=1
    client_socket.connect((host_ip,port)) # a tuple
    print(cnt)
    cnt+=1
    data = b""
    print(cnt)
    cnt+=1
    payload_size = struct.calcsize("Q")
    while len(data) < payload_size:
        packet = client_socket.recv(4*1024) # 4K
        if not packet: break
        data+=packet
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q",packed_msg_size)[0]
    
    while len(data) < msg_size:
        data += client_socket.recv(4*1024)
    frame_data = data[:msg_size]
    data  = data[msg_size:]
    frame_detect = pickle.loads(frame_data)
        
        #####################################################################
    cv2.imwrite('/home/cmit/dev_ws/ham_image/rgb_0.png',frame_detect)
    
    client_socket.close()

def dist(p1, p2):
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)), dx, dy

def adjust_before_detect():
    global frame_detect,adject_break
    frame_hsv=cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
    hsv = cv2.cvtColor(frame_hsv, cv2.COLOR_BGR2HSV)
    cv2.imwrite('/home/cmit/dev_ws/ham_image/HSV_0.png',hsv)
    hsv = cv2.blur(hsv, (5,5))
    cv2.imwrite('/home/cmit/dev_ws/ham_image/HSV_1.png',hsv)
    debug_hsv = copy.deepcopy(hsv)
    red = HSVDetection('red')
    contours, all_contour = red.process(hsv)
    red.drawAll(frame_hsv, all_contour, True)
    cv2.imwrite('/home/cmit/dev_ws/ham_image/HSV_2.png',frame_hsv)
    try:
        x, y, _w, _h = cv2.boundingRect(contours[0])
    except:
        print("contours[0] none")
        return False
    center_1 = find_center(contours[0])

    try:
        x, y, _w, _h = cv2.boundingRect(contours[1])
    except:
       print("contours[1] none")
       return False
    center_2 = find_center(contours[1])
    
    imageTheta = math.atan2(center_1[1]-center_2[1], center_1[0]-center_2[0])
    min_cx = min(center_1[0], center_2[0])
    dist_x = max(center_1[0], center_2[0]) - min_cx
    min_cy = min(center_1[1], center_2[1])
    dist_y = max(center_1[0], center_2[0]) - min_cy
    center_point = (int(min_cx+abs((dist_x/2)*math.cos(imageTheta))), int(min_cy+abs((dist_y/2.0)*math.sin(imageTheta))+(dist_x/2.0/270.0*245.0)))
    dz = dist(center_1, center_2)
    #print("center_point={}, imageTheta={}, dz={}".format(center_point, imageTheta, dz))
    if imageTheta < 0.0:
        imageTheta += (2*math.pi)

    cx = center_point[0]
    cy = center_point[1]
    frame_hsv = cv2.line(frame_detect, (cx,cy-20), (cx,cy+20), (255,255,0), 2)
    frame_hsv = cv2.line(frame_detect, (cx-20,cy), (cx+20,cy), (255,255,0), 2)
    bAutomate = 5
    offset = 10.0
    bFirst = True

    kx=0.0009481000202507924
    ky=0.0009637579043316474
    
    kx2=0.0006921136932012477
    ky2=0.0006951284260666848

    while True:
        if frame_hsv is None:
            time.sleep(0.5)
            continue
        height, width, _ = frame_hsv.shape
        ez2 = dz[0]-372.16259887312697
        ez = dz[0]-251.1075473176868
        target_theta = 3.112384304723928
        target_cx = 449
        target_cy = 231
                
        e, ex, ey = dist((target_cx, target_cy), center_point)
        e_theta = imageTheta-target_theta
        print("[{}], ez={}, robot.z={}, dz={}".format(bAutomate, ez, robot.ur_rtde.tip_pos[2], dz[0]))
        #print("{},{}".format(robot.ur_rtde.tip_pos[2], dz[0]))
        
        # ===========================================================

        if bAutomate == 5 or bAutomate == 10: # press l
            current_cmd = [0,0,0,0,0,0]
            tolerance_theta = 0.01
            if bAutomate >= 10:
                tolerance_theta = 0.005
            if e_theta > tolerance_theta:
                current_cmd[5] = -offset
            elif e_theta < -tolerance_theta:
                current_cmd[5] = offset
            #current_cmd[2] = getZforMove(dz[0], 270)

            local_ez = copy.deepcopy(ez)
            if local_ez > 1.0:
                current_cmd[2] = -offset
            elif local_ez < -1.0:
                current_cmd[2] = offset

            print('target_cmd={}, current_cmd={}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if (not compare(np.sign(target_cmd), np.sign(current_cmd))) or bFirst:
                bFirst = False
                v = 0.015
                if bAutomate >= 10:
                    v = 0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)

    return True


def detect_red():
    state=0
    while(state ==0):
        open_camera()
        frame_detect =cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        #frame_detect = cv2.resize(frame_detect,(848,480))
        out2=model_red(frame_detect)
        print(out2.xyxy)
        for i in len(out2.xyxy):
            bbox= out2.xyxy[i]
            x1=int((bbox.data[i][0]).item())
            y1=int((bbox.data[i][1]).item())
            x2=int((bbox.data[i][2]).item())
            y2=int((bbox.data[i][3]).item())
            start_point = (x1, y1)
            end_point = (x2, y2)
            frame = cv2.rectangle(frame_detect, start_point, end_point, color, thickness)
            cv2.imwrite('/home/cmit/dev_ws/ham_image/red_detect.png',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
def detect_pick():
    global frame_detect, iThreadRun, tResetDetector,pos,tip,Tool,enable_grip
    high_offset=0.421
    state=0
    while(state ==0):
        open_camera()
        frame_detect =cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        #frame_detect = cv2.resize(frame_detect,(848,480))
        out2=model(frame_detect)
        if len(out2.xyxy[0]) != 0:
            bbox= out2.xyxy[0]
            x1=int((bbox.data[0][0]).item())
            y1=int((bbox.data[0][1]).item())
            x2=int((bbox.data[0][2]).item())
            y2=int((bbox.data[0][3]).item())
            center_x=x1+(x2-x1)/2
            center_y=y1+(y2-y1)/2
            start_point = (x1, y1)
            end_point = (x2, y2)
            color = (255, 255, 0)
            thickness = 1
            ideal_start_point =(325,205) ##########x1 y1 x2 y2 :301 124 565 350 center of pic

            ideal_end_point =(588,433)############ x1 y1 x2 y2 :325 205 588 433

            color_ideal=(0,0,255)
            frame = cv2.rectangle(frame_detect, ideal_start_point,  ideal_end_point, color_ideal, 1)
            frame = cv2.rectangle(frame_detect, start_point, end_point, color, thickness)
            print("x1 y1 x2 y2 :{} {} {} {}".format(x1,y1,x2,y2))
            print("center_x: {}".format(center_x))
            print("center_y: {}".format(center_y))
            cv2.imwrite('/home/cmit/dev_ws/ham_image/detect_0.png',frame)
            #cv2.imshow("RECEIVING VIDEO", frame)
        ###########################
            area_detect=(x2-x1)*(y2-y1)
            area_ideal=(ideal_end_point[0]-ideal_start_point[0])*(ideal_end_point[1]-ideal_start_point[1])
            if (abs(area_ideal-area_detect)/area_ideal)*100 >5 :
                if area_detect>area_ideal:
                    print("too close")
                    #Tool[2]-=0.001
                if area_detect<area_ideal:
                    print("too far")
                    #Tool[2]+=0.001
            if (abs(area_ideal-area_detect)/area_ideal)*100 <5 :
                print("z Axis :OK")
         ####################################### Z axis 
            if abs(x2-ideal_end_point[0])>3:
                if x2>ideal_end_point[0]:
                    print('move right')
                    new_pos=(x2-ideal_end_point[0])*x_axis_offset
                    Tool[0]-=new_pos
                    robot.moveTool(Tool,block=False)
                    print("move Tool : "+str(Tool))
                    Tool=[0,0,0,0,0,0]
                if x2<ideal_end_point[0]:
                    print('move left')      
                    new_pos=(ideal_end_point[0]-x2)*x_axis_offset
                    Tool[0]+=new_pos
                    robot.moveTool(Tool,block=False)
                    print("move Tool : "+str(Tool))
                    Tool=[0,0,0,0,0,0]
            if abs(y1-ideal_start_point[1])>3:
                if y1>ideal_start_point[1]:
                    print('move up')
                    new_pos=(y1-ideal_start_point[1])*y_axix_offset
                    Tool[1]-=new_pos   
                    robot.moveTool(Tool,block=False)
                    print("move Tool : "+str(Tool))
                    Tool=[0,0,0,0,0,0]
                if y1<ideal_start_point[1]:
                    print('move down')
                    new_pos=(ideal_start_point[1]-y1)*y_axix_offset
                    Tool[1]+=new_pos
                    robot.moveTool(Tool,block=False)
                    print("move Tool : "+str(Tool))
                    Tool=[0,0,0,0,0,0]
            if abs(y1-ideal_start_point[1]) <= 3 and abs(x2-ideal_end_point[0]) <= 3 :
                state=1
                print("break while loop")
                cv2.destroyAllWindows()
            else:
                state=0
        #robot.moveTool(Tool,block=False)
       #print("move Tool : "+str(Tool))
        #Tool=[0,0,0,0,0,0]

            print("state :" + str(state))
            if abs(y1-ideal_start_point[1]) <=3 and abs(x2-ideal_end_point[0]) <=3 :
                pp = [  [0.6157207196310158, -0.0034951583738285054, 0.3259457979936484,-2.1669420116783513, -2.2746159319997306, 0.0002964836215492035],
                [0.6157633906357547, -0.0033207232788505835, -0.09568281924359112, 2.1665312432692843, 2.2749129756716675, -9.94432715514844e-06],
                [0.61573910454117, -0.003344623563521177, 0.2524990991124455, 2.1666182382888155, 2.274809208082394, 3.522994048246124e-05],
                [-0.010139404930124567, -0.375639022864649, 0.2959769126174789,-3.124005796356226, 0.011138367088803001, -0.0013212295123491617]] 
                print(' grip open')

                if enable_grip:
                    robot.grip_open()   
                new_pp=copy.deepcopy(ur_rtde.tip_pos)
                new_pp[2] = new_pp[2]-(high_offset/2)
                print("move robot to grip")
                robot.moveLine(new_pp, tip_speed)
                while adjust_before_detect!=False :
                    open_camera()
                    adjust_before_detect()
                print("test adject HSV")
                break
                robot.moveLine(new_pp, tip_speed)
                print(' grip close') 
                if enable_grip:
                    robot.grip_close()
                print("move robot up")
                new_pp=copy.deepcopy(ur_rtde.tip_pos)
                new_pp[2] = new_pp[2]+high_offset
                robot.moveLine(new_pp, tip_speed) 
                print("move robot arm to jig")
                robot.moveLine(pp[3], tip_speed)
                pp = [  [-0.01017494401209725, -0.3756525442585086, 0.13840573695699745,-3.1241048704483654, 0.011357524148814309, -0.0011440264047513579],#5
                [-0.011850422601318423, -0.37585862537959563, 0.317095305821409,-3.1308455555118337,-0.017135301633691005,-0.0007322231555360971],#6
                [-0.011868193208126201, -0.3758594707806762, 0.32461148496658865,-3.130902314470439,-0.017127665212704805, -0.0006788930909500954],#7
                [0.6157319922335678, -0.0033028450931117614, 0.06900699254837098,-2.1666266010453423, -2.274861107011292, -4.6540396680441666e-05],#8
                [0.6157760979216934, -0.003319206398711913, -0.09569432134831601,2.1666037432212284, 2.2748633605956563, 6.43388420211832e-05],#9
                [0.615705930495875, -0.0034379305093566524, 0.32595563912016234,-2.166769598094382, -2.274694779560951, 0.0002946796220985595]#10
                ]
                print('put tray to jig')
                robot.moveLine(pp[0], tip_speed)
                print('put tray')
                if enable_grip:
                    robot.grip_open()    
                print('lift arm up') 
                robot.moveLine(pp[1], tip_speed)
                print("********************************finish_Pick********************************")


if __name__ == '__main__':
    robot.init()
    pp = [[0.6814899895774164, -0.0405411724510514, 0.3259812667947507,2.1991536910693466,2.2431889339135105,9.717629458548752e-05]] # [x,y,z,r,p,y]
    robot.moveLine(pp[0], debug=False)
    #detect_pick()
    detect_red()
    
    del robot
