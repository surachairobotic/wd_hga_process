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
tip_speed = 0.1

enable_grip = False

model=torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_top_TINY.pt', source='local')

frame_detect = None
iThreadRun = 0
x_axis_offset=0.001289
y_axix_offset=0.001212
robot = UR_SOCKET()
pos=[]
tip=[]
Tool=[0,0,0,0,0,0]
tResetDetector = time.time()

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
        ideal_start_point =(301,124) ##########x1 y1 x2 y2 :301 124 565 350

        ideal_end_point =(565,350)############ 

        color_ideal=(0,0,255)
        frame = cv2.rectangle(frame_detect, ideal_start_point,  ideal_end_point, color_ideal, 1)
        frame = cv2.rectangle(frame_detect, start_point, end_point, color, thickness)
        print("x1 y1 x2 y2 :{} {} {} {}".format(x1,y1,x2,y2))
        print("center_x: {}".format(center_x))
        print("center_y: {}".format(center_y))
    ###########################
        area_detect=(x2-x1)*(y2-y1)
        area_ideal=(ideal_end_point[0]-ideal_start_point[0])*(ideal_end_point[1]-ideal_start_point[1])
        if (abs(area_ideal-area_detect)/area_ideal)*100 >10 :
            if area_detect>area_ideal:
                print("too close")
            if area_detect<area_ideal:
                print("too far")
        if (abs(area_ideal-area_detect)/area_ideal)*100 <10 :
            print("z Axis :OK")
     ####################################### Z axis 
        if abs(center_x-424)>10:
            if center_x>424:
                print('move right')
                
                
                
                new_pos=(center_x-424)*x_axis_offset
                #print("new_pos {}".format(new_pos))
                tip = copy.deepcopy(robot.ur_rtde.tip_pos)
                #print("Before : {}".format(tip))
                tip[1] += new_pos
                #print("After : {}".format(tip))
                Tool[0]-=new_pos
                #robot.moveLine(pos, v=0.1, block=False)
                
            if center_x<424:
                print('move left')
                
                new_pos=(424-center_x)*x_axis_offset
                #print("new_pos {}".format(new_pos))
                tip = copy.deepcopy(robot.ur_rtde.tip_pos)
                #print("Before : {}".format(tip))
                tip[1] += new_pos
                Tool[0]+=new_pos
                #print("After : {}".format(tip))
                #robot.moveLine(pos, v=0.1, block=False)
        if abs(center_y-240)>10:
            if center_y>240:
                print('move up')
                
                new_pos=(center_y-240)*y_axix_offset
                #pos = copy.deepcopy(robot.ur_rtde.joint_pos)
                #pos[1] += new_pos
                Tool[1]-=new_pos
                print(pos)
                #ur.moveX(0.01, tip_speed)
                #time.sleep(30)
               
            if center_y<240:
                print('move down')
                
                new_pos=(240-center_y)*y_axix_offset
                #pos = copy.deepcopy(robot.ur_rtde.joint_pos)
                #pos[1] -= new_pos
                Tool[1]+=new_pos
                print(pos)
                #ur.moveX(-0.01, tip_speed)
                #time.sleep(30)
                
            '''
            if abs(y2-ideal_end_point[1])>10:
                print('move down')
                ur.moveX(-0.01, tip_speed, debug=True)
                time.sleep(3)
            '''
      ####################################### X-Y AXIS
    else:
        print("Can't Detect")
    #####################################################################
  
    print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    iThreadRun = 2
    print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    tResetDetector = time.time()

def threadColorDetection():
    global frame_detect, iThreadRun,pos
    
    print('threadDetection')
    #frame_detect = cv2.resize(frame_detect,(848,480))

    frame_detect = colorDetection(frame_detect)

    print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    iThreadRun = 2
    print('threadDetection - iThreadRun : {}'.format(iThreadRun))

def colorDetection(img):
    height, width, channels = img.shape

    # convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (10,10)) 

    # lower bound and upper bound for Green color
    #lower_bound = np.array([50,50,50])
    #upper_bound = np.array([150,255,255])

    # lower bound and upper bound for Red color
    lower_bound = np.array([90,100,75])
    upper_bound = np.array([100,255,255])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8))
    mask = cv2.dilate(mask, np.ones((5, 5), dtype=np.uint8))
    
    # Now you can finally find contours.
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    final_contours = 0
    final_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > final_area:
            final_contours = contour
            final_area = area

    x, y, w, h = cv2.boundingRect(final_contours)
    #print(x, " ", y, " ", w, " ", h)
    cx = int(w/2.0+x)
    cy = int(h/2.0+y)

    # Center coordinates
    center_coordinates = (cx, cy)
     
    # Radius of circle
    radius = 2
      
    # Blue color in BGR
    color = (0, 255, 255)
      
    # Line thickness of 2 px
    thickness = 2
      
    # Using cv2.circle() method
    # Draw a circle with blue line borders of thickness of 2 px
    img = cv2.circle(img, center_coordinates, radius, color, thickness)

    #for i in range(len(final_contours)):
    #cv2.drawContours(img, final_contours, i, np.array([50, 250, 50]), 4)
    cv2.drawContours(image=img, contours=final_contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    #res = cv2.bitwise_and(img, img, mask=mask)
    err = int(width/2.0) - cx

    return img, err


def ham_detect_and_adjust(ur):
    global frame_detect, iThreadRun, tResetDetector, pos, tip ,Tool

    pp = [  [0.6157207196310158, -0.0034951583738285054, 0.3259457979936484,-2.1669420116783513, -2.2746159319997306, 0.0002964836215492035],
        [0.6157633906357547, -0.0033207232788505835, -0.09568281924359112, 2.1665312432692843, 2.2749129756716675, -9.94432715514844e-06],
        [0.61573910454117, -0.003344623563521177, 0.2524990991124455, 2.1666182382888155, 2.274809208082394, 3.522994048246124e-05],
        [-0.010139404930124567, -0.375639022864649, 0.2959769126174789,-3.124005796356226, 0.011138367088803001, -0.0013212295123491617]
     ] # [x,y,z,r,p,y]
    robot.moveLine(pp[0], debug=False)
    
    #exit()
    
    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);

    cnt=0
    print('ham_detect_and_adjust')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    print(cnt)
    cnt+=1
    host_ip = '192.168.12.200' # paste your server ip address here
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
            #robot.moveLine(tip, v=0.01, block=True)
            robot.moveTool(Tool)
            print("TOOL : {}".format(Tool))
            Tool=[0,0,0,0,0,0]
            cv2.imshow("Detection", frame_detect)
            print("finish2")
            iThreadRun = 0            
            
        #frame = cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        cv2.imwrite('ham_scale.png',frame)
        cv2.imshow("RECEIVING VIDEO", frame)
        
        #print('FPS : ' + str(1.0/(time.time()-t)))
        key = cv2.waitKey(1) & 0xFF
        '''
        if key == ord('q'):
            break
        elif key == ord('w'):
            print('w')
            ur.moveX(0.01, tip_speed, debug=True)
        elif key == ord('s'):
            print('s')
            ur.moveX(-0.01, tip_speed, debug=True)
        elif key == ord('a'):
            print('a')
            ur.moveY(0.01, tip_speed, debug=True)
        elif key == ord('d'):
            print('d')
            ur.moveY(-0.01, tip_speed, debug=True)
        elif key == ord('+'):
            print('+')
            ur.moveZ(0.01, tip_speed, debug=True)
        elif key == ord('-'):
            print('-')
            ur.moveZ(-0.01, tip_speed, debug=True)
        '''

    cv2.destroyAllWindows()
    client_socket.close()

def Pick_From_Station(ur): #1-4 waypoint
    ur.stop()
    
    jj = [  [2.918431520462036, -1.2875743967345734, 1.0661829153644007, -1.3476866346648713, -1.5630458037005823, 2.867288827896118],#1
            [2.917178153991699, -0.9941102427295228, 1.8037517706500452, -2.378920694390768, -1.5648248831378382, 2.8737618923187256],#2
            [2.9182775020599365, -1.310342864399292, 1.2846034208880823, -1.5435486652753134, -1.562657658253805, 2.868584632873535],#3
            [1.1867767572402954, -1.880596777001852, 1.7243483702289026, -1.3963246953538437, -1.568775478993551, 2.7630863189697266]   #4
         ] # radian
    pp = [  [0.6157207196310158, -0.0034951583738285054, 0.3259457979936484,-2.1669420116783513, -2.2746159319997306, 0.0002964836215492035],
            [0.6157633906357547, -0.0033207232788505835, -0.09568281924359112, 2.1665312432692843, 2.2749129756716675, -9.94432715514844e-06],
            [0.61573910454117, -0.003344623563521177, 0.2524990991124455, 2.1666182382888155, 2.274809208082394, 3.522994048246124e-05],
            [-0.010139404930124567, -0.375639022864649, 0.2959769126174789,-3.124005796356226, 0.011138367088803001, -0.0013212295123491617]
         ] # [x,y,z,r,p,y]
    
    print('step 1 grip open')
    if enable_grip:
        ur.grip_open()
        

    ham_detect_and_adjust(ur)

    print('step 2 ur.moveLine(pp[0]) waypoint 1') ## not use
    ur.moveLine(pp[0], tip_speed)
    
    
    return -1
    
    print('step 3 ur.moveLine(pp[1]) waypoint 2') ## not use
    ur.moveLine(pp[1], tip_speed)
    
    print('step 4 grip close')
    if enable_grip:
        ur.grip_close()

    print('step 5 ur.moveLine(pp[2]) waypoint 3')
    ur.moveLine(pp[2], tip_speed)
    
    print('step 6 ur.moveLine(pp[3]) waypoint 4')
    ur.moveLine(pp[3], tip_speed)
    
def PickPlaceOnCar(ur):
    ur.stop()
    
    jj = [  [1.1854966878890991, -1.86033358196401, 2.1157615820514124, -1.8081351719298304, -1.5688279310809534, 2.7654571533203125],#5
            [1.1807478666305542, -1.8768779240050257, 1.668762509022848, -1.3510250610164185, -1.56681996980776, 2.7385597229003906],#6
            [1.1807717084884644, -1.8731304607787074, 1.6466658751117151, -1.3327413511327286, -1.5668709913836878, 2.7384324073791504],#7
            [2.9175713062286377, -1.2228343945792695, 1.6495335737811487, -1.9958745441832484, -1.5631216208087366, 2.8716158866882324],#8
            [2.917159080505371, -0.9940787118724366, 1.8037403265582483, -2.3789531193175257, -1.5647771994220179, 2.873798131942749],#9
            [2.918501138687134, -1.2875669759562989, 1.0661338011371058, -1.3475923401168366, -1.5630133787738245, 2.867243528366089]#1
         
         ]
    
    '''
    res = ur.getFK(jj[0])
    print(res)
    return -1
    '''

    pp = [  [-0.01017494401209725, -0.3756525442585086, 0.13840573695699745,-3.1241048704483654, 0.011357524148814309, -0.0011440264047513579],#5
            [-0.011850422601318423, -0.37585862537959563, 0.317095305821409,-3.1308455555118337,-0.017135301633691005,-0.0007322231555360971],#6
            [-0.011868193208126201, -0.3758594707806762, 0.32461148496658865,-3.130902314470439,-0.017127665212704805, -0.0006788930909500954],#7
            [0.6157319922335678, -0.0033028450931117614, 0.06900699254837098,-2.1666266010453423, -2.274861107011292, -4.6540396680441666e-05],#8
            [0.6157760979216934, -0.003319206398711913, -0.09569432134831601,2.1666037432212284, 2.2748633605956563, 6.43388420211832e-05],#9
            [0.615705930495875, -0.0034379305093566524, 0.32595563912016234,-2.166769598094382, -2.274694779560951, 0.0002946796220985595]#10
         ]
    
    print('step 7 ur.moveLine(pp[0]) waypoint 5')
    ur.moveLine(pp[0], tip_speed)
    
    print('step 8 grip open')
    if enable_grip:
        ur.grip_open()    
        
    return -1
    
    print('step 9 ur.moveLine(pp[1]) waypoint 6')
    ur.moveLine(pp[1], tip_speed)
    
    print('step 11 ur.moveLine(pp[0]) waypoint 5')
    ur.moveLine(pp[0], tip_speed)
    
    print('step 12 grip close')
    if enable_grip:
        ur.grip_close()    
    
    print('step 13 ur.moveLine(pp[2]) waypoint 7')
    ur.moveLine(pp[2], tip_speed)
    
    print('step 14 ur.moveLine(pp[3]) waypoint 8')
    ur.moveLine(pp[3], tip_speed)

    print('step 15 ur.moveLine(pp[4]) waypoint 9')
    ur.moveLine(pp[4], tip_speed)
    
    print('step 16 grip open')
    if enable_grip:
        ur.grip_open()
    
    print('step 17 ur.moveLine(pp[5]) waypoint 10')
    ur.moveLine(pp[5], tip_speed)
  
if __name__ == '__main__':
    #ur = UR_SOCKET() # for move control
    #ur.init()

    robot.init()
    #ur_rtde = UR_INFORMATION() # get current robot state
    ham_detect_and_adjust(10)

    print('Pick_From_Station')
    #Pick_From_Station(ur)
    print('PickPlaceOnCar')
    #PickPlaceOnCar(ur)
    print('end')

    del ur
   
