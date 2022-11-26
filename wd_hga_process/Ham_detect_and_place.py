import time, keyboard, math, requests, copy, threading
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *

import matplotlib.pyplot as plt
from PIL import Image, ImageFont, ImageDraw
import torch
import pandas as pd


SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"
tip_speed = 0.5
high_offset=0.415
adj_hight=0
enable_grip = True

model=torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_jig_TINY.pt', source='local')

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

def threadDetection():
    global frame_detect, iThreadRun, tResetDetector,pos,tip,Tool
    pos = copy.deepcopy(robot.ur_rtde.joint_pos)
    tip = copy.deepcopy(robot.ur_rtde.tip_pos)
    print('threadDetection')
    frame_detect = cv2.resize(frame_detect,(848,480))
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
        ideal_start_point =(317,199) ##########x1 y1 x2 y2 :301 124 565 350 center of pic

        ideal_end_point =(589,450)############ x1 y1 x2 y2 :317 207 589 453   x1 y1 x2 y2 :317 204 592 455 x1 y1 x2 y2 :316 199 589 450




        color_ideal=(0,0,255)
        frame = cv2.rectangle(frame_detect, ideal_start_point,  ideal_end_point, color_ideal, 1)
        frame = cv2.rectangle(frame_detect, start_point, end_point, color, thickness)
        print("x1 y1 x2 y2 :{} {} {} {}".format(x1,y1,x2,y2))
        print("center_x: {}".format(center_x))
        print("center_y: {}".format(center_y))
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
            if x2<ideal_end_point[0]:
                print('move left')      
                new_pos=(ideal_end_point[0]-x2)*x_axis_offset
                Tool[0]+=new_pos
        if abs(y1-ideal_start_point[1])>3:
            if y1>ideal_start_point[1]:
                print('move up')
                new_pos=(y1-ideal_start_point[1])*y_axix_offset
                Tool[1]-=new_pos   
            if y1<ideal_start_point[1]:
                print('move down')
                new_pos=(ideal_start_point[1]-y1)*y_axix_offset
                Tool[1]+=new_pos
                print(pos)
        if abs(y1-ideal_start_point[1]) <3 and abs(x2-ideal_end_point[0]) <3 :
            print("----------------------------------------- MOVE DONE-----------------------------------------")
       ###################################################################################################################move robot

            
              
     
      ####################################### X-Y AXIS
    else:
        print("Can't Detect")
    #####################################################################
  
    print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    iThreadRun = 2
    print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    tResetDetector = time.time()


def ham_detect_and_adjust(ur):
    global frame_detect, iThreadRun, tResetDetector, pos, tip ,Tool,adj_hight

    pp = [[0.6814899895774164, -0.0405411724510514, 0.3259812667947507,2.1991536910693466,2.2431889339135105,9.717629458548752e-05]] # [x,y,z,r,p,y]
    print("move pp0")
    robot.moveLine(pp[0], debug=False)
    
    
    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);

    cnt=0
    print('ham_detect_and_adjust')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    print(cnt)
    cnt+=1
    host_ip = '192.168.12.193' # paste your server ip address here
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

    print('while loop')
    while True:
        t = time.time()
        if time.time()-tResetDetector > 5.0:
            tResetDetector = time.time()
            iThreadRun = 0

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
        frame = pickle.loads(frame_data)
        
        #frame2, err = colorDetection(frame)
        #print(err)
        #####################################################################
        print('iThreadRun : {}'.format(iThreadRun))
        if iThreadRun == 0:
            print('in iThreadRun')
            iThreadRun = 1
            frame_detect = copy.deepcopy(frame)
            threadStatus = threading.Thread(target=threadDetection)
            #threadStatus = threading.Thread(target=threadColorDetection)
            threadStatus.start()
        elif iThreadRun == 2:
            threadStatus.join()
            print("finish1")
            print("actual pos : {}".format(robot.ur_rtde.tip_pos))
            print("Action move : {}".format(tip))
           
            robot.moveTool(Tool)
            print("TOOL : {}".format(Tool))
            Tool=[0,0,0,0,0,0]
            cv2.imshow("Detection", frame_detect)
            print("finish2")
            iThreadRun = 0            
            
        #frame = cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        
        cv2.imshow("RECEIVING VIDEO", frame)
        
        #print('FPS : ' + str(1.0/(time.time()-t)))
        key = cv2.waitKey(1) & 0xFF
        if key == ord('k'):
            ham_pp = [  [-0.01017494401209725, -0.3756525442585086, 0.13840573695699745,-3.1241048704483654, 0.011357524148814309, -0.0011440264047513579],#5
            [-0.011850422601318423, -0.37585862537959563, 0.317095305821409,-3.1308455555118337,-0.017135301633691005,-0.0007322231555360971],#6
            [-0.011868193208126201, -0.3758594707806762, 0.32461148496658865,-3.130902314470439,-0.017127665212704805, -0.0006788930909500954],#7
            [0.6157319922335678, -0.0033028450931117614, 0.06900699254837098,-2.1666266010453423, -2.274861107011292, -4.6540396680441666e-05],#8
            [0.6157760979216934, -0.003319206398711913, -0.09569432134831601,2.1666037432212284, 2.2748633605956563, 6.43388420211832e-05],#9
            [0.615705930495875, -0.0034379305093566524, 0.32595563912016234,-2.166769598094382, -2.274694779560951, 0.0002946796220985595]#10
         ]
            agv=[[-0.010139404930124567, -0.375639022864649, 0.2959769126174789,-3.124005796356226, 0.011138367088803001, -0.0013212295123491617]]
            ham_pp2=[ [-0.010139404930124567, -0.375639022864649, 0.2959769126174789,-3.124005796356226, 0.011138367088803001, -0.0013212295123491617]]
            pos_tip=copy.deepcopy(robot.ur_rtde.tip_pos)
            pos_pos=copy.deepcopy(robot.ur_rtde.joint_pos)
            print("pick tray on agv")
            print('step put arm to jig 1') # put arm to jig
            robot.moveLine(ham_pp2[0], tip_speed)
            
            print('step put arm to jig 2') # put arm to jig
            robot.moveLine(ham_pp[0], tip_speed)
    
            print('step grab tray') #grab tray
            if enable_grip:
                robot.grip_close()
            print('step lift arm up')
            robot.moveLine(ham_pp[1], tip_speed)  
            print("move arm to save pos")
            robot.moveJ(pos_pos, tip_speed)  
            print("put tray to jig")
            pos_tip[2] = pos_tip[2]-high_offset
            robot.moveLine(pos_tip, tip_speed)  
            if enable_grip:
                robot.grip_open()    
            print('lift arm up') 
            robot.moveJ(pos_pos, tip_speed)
            print('move arm to agv') 
            robot.moveLine(agv[0], tip_speed)
            print("-------------------------------finish----------------------------------")
            break

    cv2.destroyAllWindows()
    client_socket.close()

  
if __name__ == '__main__':
    #ur = UR_SOCKET() # for move control
    #ur.init()

    robot.init()
    #ur_rtde = UR_INFORMATION() # get current robot state
    ham_detect_and_adjust(10)
    del robot


    
   
