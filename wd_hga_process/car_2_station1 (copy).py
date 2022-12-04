import time, keyboard, math, requests, copy, threading
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *

import matplotlib.pyplot as plt
from PIL import Image, ImageFont, ImageDraw
import torch
import pandas as pd

import requests, threading, time, json
from hsv_detection import *

robot = UR_SOCKET()
tip_speed = 0.1

pos=[]
tip=[]
Tool=[0,0,0,0,0,0]

IP_WEBSERVER = '192.168.12.252:8000'
IP_CAMERA = '192.168.12.195'

frame_detect = None
errTheta = 0.0
iThreadRun = 0
oldTarget = []
bAutomate = 0

debug_hsv = []
center_point = (0,0)
imageTheta = 0.0
target_theta = 3.0947258255584007
target_cmd = [0,0,0,0,0,0]
dz = [0.0]

hsv_debug = False


def sendJson(num, ip_port):
    # Get Request
    host = 'http://'+ip_port+'/btn_call/'
    #host = '0.0.0.0:8000/btn_call/'

    r = requests.post(host, json={"call_id": num})
    print('Status code : ', r.status_code)
    print(r.json())

def getStatus(ip_port):
    # Format Headers
    headers = {}
    headers['Content-Type'] = 'application/json'
    headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='

    _host = 'http://' + ip_port + '/robotstate'
    get_status = requests.get(_host, headers=headers)
    parsed = json.loads(get_status.content)            
    res = str(parsed['message'])
    return res

def detect(host_ip, port = 1234):
    global frame_detect, iThreadRun, h, s, v, robot, bAutomate, oldTarget, debug_hsv, center_point, imageTheta, target_theta, target_cmd, dz

    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);
    iThreadRun=0

    cnt=0
    #print('detect_and_adjust')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    client_socket.connect((host_ip,port)) # a tuple
    data = b""
    payload_size = struct.calcsize("Q")

    #print('while loop')
    while True:
        t = time.time()

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

        if iThreadRun == 0:
            iThreadRun = 1
            frame_detect = copy.deepcopy(frame)
            threadStatus = threading.Thread(target=threadColorDetection)
            threadStatus.start()
        elif iThreadRun == 2:
            vis = np.concatenate((frame_detect, debug_hsv), axis=1)
            cv2.imshow("Detection", vis)
            iThreadRun = 0            
        
        offset = 0.1
        key = cv2.waitKey(10) & 0xFF

    cv2.destroyAllWindows()
    client_socket.close()


def adjust(host_ip, port = 1234):
    global frame_detect, iThreadRun, h, s, v, robot, bAutomate, oldTarget, debug_hsv, center_point, imageTheta, target_theta, target_cmd, dz
    bAutomate = 5
    offset = 0.1
    while True:
        if frame_detect is None:
            time.sleep(0.5)
            continue
        height, width, _ = frame_detect.shape
        ez = dz[0]-585.3768017268877
        if abs(ez) > 20:
            target_cy = 82
        else:
            target_cy = 163
        e, ex, ey = dist((489, target_cy), center_point)
        e_theta = imageTheta-target_theta
        print("[{}], ex={}, ey={}, e_theta={}, ez={}".format(bAutomate, ex, ey, e_theta, ez))
        
        if bAutomate == 5: # press l
            current_cmd = [0,0,0,0,0,0]
            if ez > 2.5:
                current_cmd[2] = -offset
            elif ez < -2.5:
                current_cmd[2] = offset
            if ex > 5:
                current_cmd[0] = offset
            elif ex < -5:
                current_cmd[0] = -offset
            if ey > 5:
                current_cmd[1] = offset
            elif ey < -5:
                current_cmd[1] = -offset
            if e_theta > 0.01:
                current_cmd[5] = offset
            elif e_theta < -0.01:
                current_cmd[5] = -offset
            print('{}, {}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if not compare(np.sign(target_cmd), np.sign(current_cmd)):
                v = 0.025
                if current_cmd[5] != 0:
                    v=0.006
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                bAutomate = 6
        if bAutomate == 6:
            current_cmd = [0,0,0,0,0,0]
            if ez > 1.0:
                current_cmd[2] = -offset
            elif ez < -1.0:
                current_cmd[2] = offset
            if ex > 1:
                current_cmd[0] = offset
            elif ex < -1:
                current_cmd[0] = -offset
            if ey > 1:
                current_cmd[1] = offset
            elif ey < -1:
                current_cmd[1] = -offset
            if e_theta > 0.005:
                current_cmd[5] = offset
            elif e_theta < -0.005:
                current_cmd[5] = -offset
            print('{}, {}'.format(target_cmd, current_cmd))
            if target_cmd != current_cmd:
                v = 0.001
                if current_cmd[5] != 0:
                    v=0.0005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                bAutomate = 7
        if bAutomate == 7:
            robot.moveTool([0,0,0.06,0,0,0], 0.01, block=True, exceptStop=True)
            break

def detect_and_adjust(host_ip, port = 1234):
    global frame_detect, iThreadRun, h, s, v, robot, bAutomate, oldTarget, debug_hsv, center_point, imageTheta, target_theta, target_cmd, dz

    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);
    iThreadRun=0

    cnt=0
    #print('detect_and_adjust')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    client_socket.connect((host_ip,port)) # a tuple
    data = b""
    payload_size = struct.calcsize("Q")

    #print('while loop')
    while True:
        t = time.time()

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
        
        if iThreadRun == 0:
            iThreadRun = 1
            frame_detect = copy.deepcopy(frame)
            threadStatus = threading.Thread(target=threadColorDetection)
            threadStatus.start()
        elif iThreadRun == 2:
            vis = np.concatenate((frame_detect, debug_hsv), axis=1)
            cv2.imshow("Detection", vis)
            iThreadRun = 0            
        
        offset = 0.1
        key = cv2.waitKey(10) & 0xFF
        
        bAutomate = 5
        
        if key == ord('j'):
            bAutomate = 0
            print('bAutomate = ', bAutomate)
        elif key == ord('k'):
            bAutomate += 1
            print('bAutomate = ', bAutomate)
        elif key == ord('l'):
            bAutomate = 5
            print('bAutomate = ', bAutomate)
        
        height, width, _ = frame_detect.shape
        ez = dz[0]-585.3768017268877
        if abs(ez) > 20:
            target_cy = 82
        else:
            target_cy = 163
        e, ex, ey = dist((489, target_cy), center_point)
        e_theta = imageTheta-target_theta
        print("[{}], ex={}, ey={}, e_theta={}, ez={}".format(bAutomate, ex, ey, e_theta, ez))
        
        if bAutomate == 5: # press l
            current_cmd = [0,0,0,0,0,0]
            if ez > 2.5:
                current_cmd[2] = -offset
            elif ez < -2.5:
                current_cmd[2] = offset
            if ex > 5:
                current_cmd[0] = offset
            elif ex < -5:
                current_cmd[0] = -offset
            if ey > 5:
                current_cmd[1] = offset
            elif ey < -5:
                current_cmd[1] = -offset
            if e_theta > 0.01:
                current_cmd[5] = offset
            elif e_theta < -0.01:
                current_cmd[5] = -offset
            print('{}, {}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if not compare(np.sign(target_cmd), np.sign(current_cmd)):
                v = 0.025
                if current_cmd[5] != 0:
                    v=0.006
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                bAutomate = 6
        if bAutomate == 6:
            current_cmd = [0,0,0,0,0,0]
            if ez > 1.0:
                current_cmd[2] = -offset
            elif ez < -1.0:
                current_cmd[2] = offset
            if ex > 1:
                current_cmd[0] = offset
            elif ex < -1:
                current_cmd[0] = -offset
            if ey > 1:
                current_cmd[1] = offset
            elif ey < -1:
                current_cmd[1] = -offset
            if e_theta > 0.005:
                current_cmd[5] = offset
            elif e_theta < -0.005:
                current_cmd[5] = -offset
            print('{}, {}'.format(target_cmd, current_cmd))
            if target_cmd != current_cmd:
                v = 0.001
                if current_cmd[5] != 0:
                    v=0.0005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                bAutomate = 7
        if bAutomate == 7:
            robot.moveTool([0,0,0.06,0,0,0], 0.01, block=True, exceptStop=True)
            threadStatus.join()
            break
        
        if key == ord('s'):
            print('stop')
            robot.stop()
        elif key == ord('u'):
            print('zero speed')
            robot.moveTool([0,0,0.05,0,0,0], 0.01, block=True, exceptStop=True)
            #robot.moveTool([0,0,0,0,0,0], block=False)
        elif key == ord('q'):
            threadStatus.join()
            break

    cv2.destroyAllWindows()
    client_socket.close()

def threadColorDetection():
    global frame_detect, iThreadRun

    if colorDetection2() == -1:
        return -1

    iThreadRun = 2

def colorDetection2():
    global frame_detect, iThreadRun, h, s, v, robot, errTheta, debug_hsv, center_point, imageTheta, dz
    height, width, channels = frame_detect.shape

    points = np.array([[0, 190], [width, 190], [width, height], [0, height]])
    cv2.fillPoly(frame_detect, pts=[points], color=(0, 0, 0))

    # convert to hsv colorspace
    hsv = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (5,5))
    debug_hsv = copy.deepcopy(hsv)

    red = HSVDetection('red')
    contours, all_contour = red.process(hsv)
    red.drawAll(frame_detect, all_contour, hsv_debug)
    
    if contours == None or len(contours) != 2:
        iThreadRun = 2
        return False
        
    try:
        x, y, _w, _h = cv2.boundingRect(contours[0])
    except:
        iThreadRun = 2
        return False
    center_1 = find_center(contours[0])

    try:
        x, y, _w, _h = cv2.boundingRect(contours[1])
    except:
        iThreadRun = 2
        return False
    center_2 = find_center(contours[1])
    
    cx = min(center_1[0], center_2[0])
    cx += ((max(center_1[0], center_2[0]) - cx) / 2)
    cx = int(cx)
    cy = min(center_1[1], center_2[1])
    cy += ((max(center_1[1], center_2[1]) - cy) / 2)
    cy = int(cy)
    center_point = (cx, cy)
    imageTheta = math.atan2(center_1[1]-center_2[1], center_1[0]-center_2[0])
    dz = dist(center_1, center_2)
    #print("center_point={}, imageTheta={}, dz={}".format(center_point, imageTheta, dz))
    if imageTheta < 0.0:
        imageTheta += (2*math.pi)

    iThreadRun = 2
    return True

def dist(p1, p2):
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)), dx, dy

def find_center(contour):
    x, y, _w, _h = cv2.boundingRect(contour)
    cx = int(_w/2.0+x)
    cy = int(_h/2.0+y)

    # Center coordinates
    center_coordinates = (cx, cy)
    return center_coordinates

def compare(a, b):
    n = len(a)
    if n != len(b):
        return False
    for i in range(n):
        if int(a[i]) != int(b[i]):
            return False
    return True

if __name__ == '__main__':
    print('car2station')
    robot.init()
    
    jj = [  [1.1903325319290161, -1.8719140491881312, 1.62439471880068, -1.321559564476349, -1.5631573835956019, 2.7640068531036377]
         ] # radian
    pp = [  [-0.007828889502697804, -0.37811824691844165, 0.33376358921559546, -3.141496661051462, 0.008164449775267655, -0.00011932026040040477]
         ] # [x,y,z,r,p,y]

    step = [1,2]

    threadDetect = threading.Thread(target=detect, args=(IP_CAMERA,))
    threadDetect.start()

    print('step 1 grip open')
    robot.grip_open()
    robot.moveLine(pp[0], tip_speed)
    sendJson(step[0], IP_WEBSERVER)    
    time.sleep(5.0)

    t = time.time()
    while (time.time()-t) < 30.0:
        msgs = getStatus(IP_WEBSERVER)
        print(msgs)
        if msgs.find('Waiting for new missions') != -1:
            break
        time.sleep(0.1)

    new_pose = copy.deepcopy(jj[0])
    new_pose[0] = new_pose[0]+(math.pi/2.0)
    robot.moveJ(new_pose, 0.3)
    
    robot.moveTool([0,0.1,0,0,0,0], block=False)
    time.sleep(1.0)

    threadDetect = threading.Thread(target=detect, args=(IP_CAMERA,))
    threadDetect.start()
    adjust()
    time.sleep(5.0)

    robot.grip_close()
    robot.moveTool([0,0,-0.4,0,0,0], block=False)
    time.sleep(7.5)
    robot.moveLine(pp[0], tip_speed)
    ################################################################################
    
    ham_pp = [  [-0.01017494401209725, -0.3756525442585086, 0.13840573695699745,-3.1241048704483654, 0.011357524148814309, -0.0011440264047513579],#5
            [-0.011850422601318423, -0.37585862537959563, 0.317095305821409,-3.1308455555118337,-0.017135301633691005,-0.0007322231555360971],#6
            [-0.011868193208126201, -0.3758594707806762, 0.32461148496658865,-3.130902314470439,-0.017127665212704805, -0.0006788930909500954],#7
            [0.6157319922335678, -0.0033028450931117614, 0.06900699254837098,-2.1666266010453423, -2.274861107011292, -4.6540396680441666e-05],#8
            [0.6157760979216934, -0.003319206398711913, -0.09569432134831601,2.1666037432212284, 2.2748633605956563, 6.43388420211832e-05],#9
            [0.615705930495875, -0.0034379305093566524, 0.32595563912016234,-2.166769598094382, -2.274694779560951, 0.0002946796220985595]#10
         ]
    agv=[-0.010139404930124567, -0.375639022864649, 0.2959769126174789,-3.124005796356226, 0.011138367088803001, -0.0013212295123491617] 
    print('step move arm to agv')
    robot.moveLine(agv[0], tip_speed)
    time.sleep(1.0)
    print('step put tray to jig')
    robot.moveLine(ham_pp[0], tip_speed)
    print('step put tray') 
    if enable_grip:
        robot.grip_open() 
    print('step lift arm up')
    robot.moveLine(ham_pp[1], tip_speed)
    print('End pick')
    
    
    ################################################################################
    
    sendJson(step[1], IP_WEBSERVER)
    time.sleep(5.0)
    t = time.time()
    while (time.time()-t) < 30.0:
        msgs = getStatus(IP_WEBSERVER)
        print(msgs)
        if msgs.find('Waiting for new missions') != -1:
            break
        time.sleep(0.1)
        
    ################################################################################
    print('step put arm to jig') # put arm to jig
    robot.moveLine(ham_pp[0], tip_speed)
    
    print('step grab tray') #grab tray
    if enable_grip:
        robot.grip_close()
    print('step lift arm up')
    robot.moveLine(ham_pp[1], tip_speed)        
    ################################################################################################################################################################
    new_pose = copy.deepcopy(jj[0])
    new_pose[0] = new_pose[0]+(math.pi/2.0)
    robot.moveJ(new_pose, 0.3)
    
    robot.moveTool([0,0.1,0,0,0,0], block=False)
    time.sleep(1.0)

    adjust()
    robot.grip_open()
    robot.moveTool([0,0,-0.4,0,0,0], block=False)
    time.sleep(5.0)
    robot.moveLine(pp[0], tip_speed)
    
    print('end')
    del robot
   
