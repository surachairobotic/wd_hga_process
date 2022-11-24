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

def getZforMove(current, target):
    current_z = redDistance2robotZ(current)
    target_z = redDistance2robotZ(target)
    return current_z-target_z

def redDistance2robotZ(_in):
    coeff = [-1.06696161e-11, 1.75842516e-08, -8.44855503e-06, 5.80445262e-01]
    res = (coeff[0]*pow(_in,4)) + (coeff[1]*pow(_in,3)) + (coeff[2]*pow(_in,2)) + coeff[3] 
    return res

def adjust():
    global frame_detect, iThreadRun, h, s, v, robot, bAutomate, oldTarget, debug_hsv, center_point, imageTheta, target_theta, target_cmd, dz

    bAutomate = 5
    offset = 10.0
    bFirst = True

    kx=0.0009481000202507924
    ky=0.0009637579043316474
    
    kx2=0.0006921136932012477
    ky2=0.0006951284260666848

    while True:
        if frame_detect is None:
            time.sleep(0.5)
            continue
        height, width, _ = frame_detect.shape
        ez2 = dz[0]-372.16259887312697
        ez = dz[0]-270
        target_theta = 3.112031375024515
        target_cx = width/2
        target_cy = height/2
                
        e, ex, ey = dist((target_cx, target_cy), center_point)
        e_theta = imageTheta-target_theta
        print("[{}], ez={}, ez2={}, robot.z={}, dz={}".format(bAutomate, ez, ez2, robot.ur_rtde.tip_pos[2], dz[0]))
        #print("{},{}".format(robot.ur_rtde.tip_pos[2], dz[0]))
        if bAutomate == 1: # press k
            current_cmd = [0,0,0,0,0,0]
            if ey > 5:
                current_cmd[1] = (abs(ey)*ky) #offset
            elif ey < -5:
                current_cmd[1] = -(abs(ey)*ky)
            if ex > 5:
                current_cmd[0] = (abs(ex)*kx) #offset
            elif ex < -5:
                current_cmd[0] = -(abs(ex)*kx)

            print('current_cmd = ', current_cmd)
            robot.moveTool(current_cmd, v=0.005, block=False)
            bAutomate=0
        elif bAutomate == 2: # press k
            current_cmd = [0,0,0,0,0,0]
            current_cmd[2] = getZforMove(dz[0], 372.16259887312697)
            robot.moveTool(current_cmd, block=False)
            bAutomate=0
        elif bAutomate == 3: # press k
            vel = [0,0,0,0,0,0]
            if abs(e_theta) < 0.01:
                robot.moveTool(vel, block=False)
        
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
            if bAutomate >= 10:
                local_ez = copy.deepcopy(ez2)
            if local_ez > 1.0:
                current_cmd[2] = -offset
            elif local_ez < -1.0:
                current_cmd[2] = offset

            print('target_cmd={}, current_cmd={}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if (not compare(np.sign(target_cmd), np.sign(current_cmd))) or bFirst:
                bFirst = False
                v = 0.015
                if current_cmd[5] != 0:
                    v=0.005
                if bAutomate >= 10:
                    v = 0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                if bAutomate == 5:
                    bAutomate = 6
                else:
                    bFirst = True
                    bAutomate = 7
        elif bAutomate == 6 or bAutomate == 11:
            current_cmd = [0,0,0,0,0,0]
            tolerance = 5
            local_kx = kx
            local_ky = ky
            if bAutomate >= 10:
                tolerance = 1
                local_kx = kx2
                local_ky = ky2
            if ey > tolerance:
                current_cmd[1] = (abs(ey)*local_ky) #offset
            elif ey < -tolerance:
                current_cmd[1] = -(abs(ey)*local_ky)
            if ex > tolerance:
                current_cmd[0] = (abs(ex)*local_kx) #offset
            elif ex < -tolerance:
                current_cmd[0] = -(abs(ex)*local_kx)
            current_cmd[2] = getZforMove(dz[0], 372.16259887312697)
            if current_cmd[2] < 0.005 and current_cmd[2] > -0.005:
                current_cmd[2] = 0.0

            print('current_cmd=', current_cmd)
            if sum(current_cmd) != 0.0:
                robot.moveTool(current_cmd, v=0.25, block=True)
                target_cmd = copy.deepcopy(current_cmd)
            if bAutomate == 6:
                bFirst = True
                bAutomate = 10
            else:
                bAutomate = 0
        elif bAutomate == 7 or bAutomate == 12:
            current_cmd = [0,0,0,0,0,0]
            if ex > 1:
                current_cmd[0] = offset
            elif ex < -1:
                current_cmd[0] = -offset
            if ey > 1:
                current_cmd[1] = offset
            elif ey < -1:
                current_cmd[1] = -offset

            #if target_cmd != current_cmd or bFirst:
            if (not compare(np.sign(target_cmd), np.sign(current_cmd))) or bFirst:
                bFirst = False
                v=0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                if bAutomate == 7:
                    bAutomate = 20
                else:
                    bAutomate = 0
        elif bAutomate == 8 or bAutomate == 13:
            current_cmd = [0,0,0,0,0,0]
            if ez2 > 7.0:
                current_cmd[2] = -offset
            elif ez2 < -1.0:
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
                current_cmd[5] = -offset
            elif e_theta < -0.005:
                current_cmd[5] = offset
            #print('{}, {}'.format(target_cmd, current_cmd))
            if target_cmd != current_cmd or bFirst:
                bFirst = False
                v = 0.01
                if current_cmd[5] != 0:
                    v=0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                if bAutomate == 8:
                    bAutomate = 20
                else:
                    bAutomate = 0
        elif bAutomate == 20:
            print('moveTool([0.02686939900984832, 0.08657685150415684,0.15,0,0,0]')
            robot.moveTool([0.02686939900984832, 0.08657685150415684,0.15,0,0,0], 0.25, block=True, exceptStop=True)
            while True:
                err = 0.0
                for x in robot.ur_rtde.joint_velo:
                    err += abs(x)
                if err < 0.001:
                    break
            print('moveTool([0,0,0.05,0,0,0]')
            robot.moveTool([0,0,0.05,0,0,0], 0.1, block=True, exceptStop=True)
            break


def threadColorDetection():
    global frame_detect, iThreadRun

    if colorDetection2() == -1:
        return -1

    iThreadRun = 2

def colorDetection2():
    global frame_detect, iThreadRun, h, s, v, robot, errTheta, debug_hsv, center_point, imageTheta, dz
    height, width, channels = frame_detect.shape

    '''
    points = np.array([[0, 190], [width, 190], [width, height], [0, height]])
    cv2.fillPoly(frame_detect, pts=[points], color=(0, 0, 0))
    points = np.array([[width, 0], [width, height], [width-int(width/10), height], [width-int(width/10), 0]])
    cv2.fillPoly(frame_detect, pts=[points], color=(0, 0, 0))
    '''

    # convert to hsv colorspace
    hsv = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (5,5))
    debug_hsv = copy.deepcopy(hsv)

    #orange = HSVDetection('orange')
    #contours = orange.process(hsv)
    #orange.drawAll(frame_detect, contours, hsv_debug)

    red = HSVDetection('red')
    contours, all_contour = red.process(hsv)
    #red.drawAll(frame_detect, contour, hsv_debug)
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
    frame_detect = cv2.line(frame_detect, (cx,cy-20), (cx,cy+20), (255,255,0), 2)
    frame_detect = cv2.line(frame_detect, (cx-20,cy), (cx+20,cy), (255,255,0), 2)

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

def myBlock(t, duration):
    while time.time()-t < duration:
        time.sleep(0.01)

if __name__ == '__main__':
    print('car2station')
    robot.init()
    
    '''
    over car
    Joint : [1.1900781393051147, -1.8786550960936488, 1.6523898283587855, -1.3427302998355408, -1.5630763212787073, 2.764693260192871]
    TCP: pos [-0.007583460058656687, -0.37736000459046504, 0.3247866033575867] m, rot [-3.14142698997794, 0.009354814800434154, -0.00021372635081474143] rad

    place on car
    Joint : [1.188525915145874, -1.8645268879332484, 2.1453784147845667, -1.8499347172179164, -1.5629962126361292, 2.7674479484558105]
    TCP: pos [-0.007576178412409958, -0.3773818028145571, 0.12986619843536518] m, rot [-3.1414985027277504, 0.009336444075501774, -0.00018903153494215214] rad
    '''

    jj = [  [1.1900781393051147, -1.8786550960936488, 1.6523898283587855, -1.3427302998355408, -1.5630763212787073, 2.764693260192871],
            [1.188525915145874, -1.8645268879332484, 2.1453784147845667, -1.8499347172179164, -1.5629962126361292, 2.7674479484558105]
         ] # radian
    pp = [  [-0.007583460058656687, -0.37736000459046504, 0.3247866033575867, -3.14142698997794, 0.009354814800434154, -0.00021372635081474143],
            [-0.007576178412409958, -0.3773818028145571, 0.12986619843536518, -3.1414985027277504, 0.009336444075501774, -0.00018903153494215214]
         ] # [x,y,z,r,p,y]


    tip_speed = 0.25
    step = [1,2]

    print('step 1 grip open')
    #robot.grip_open()
    print('step 2 moveLine to home pose')
    robot.moveLine(pp[0], tip_speed)
    print('step 3 move to station A')
    target_station = step[0]
    sendJson(target_station, IP_WEBSERVER)    

    print('step 6 detection thread start')
    threadDetect = threading.Thread(target=detect, args=(IP_CAMERA,))
    threadDetect.start()
    time.sleep(5.0)

    t = time.time()
    while (time.time()-t) < 30.0:
        msgs = getStatus(IP_WEBSERVER)
        print(msgs)
        if msgs.find('Waiting for new missions') != -1:
            break
        time.sleep(0.1)

    print('step 4 moveJ to camera ready position')
    new_pose = copy.deepcopy(jj[0])
    new_pose[0] = new_pose[0]+(math.pi/2.0)
    robot.moveJ(j=new_pose, v=math.pi/2.0)
    
    print('step 5 moveTool y+0.1')
    #robot.moveTool([0,0.2,0,0,0,0], v=0.025, block=True)
    offset_front = [0,0.2,0.05,0,0,0]
    #if target_station == 2:
    #    offset_front = [0,0.2,0.05,0,0,0]
    robot.moveTool([0,0.2,0.05,0,0,0], v=tip_speed, block=True)
    time.sleep(1.0)

    '''
    print('step 6 detection thread start')
    threadDetect = threading.Thread(target=detect, args=(IP_CAMERA,))
    threadDetect.start()
    time.sleep(5.0)
    '''

    print('step 7 adjust start')
    adjust()
    #time.sleep(5.0)

    print('step 8 grip close')
    robot.grip_close()
    print('step 9 moveTool z-0.4')
    robot.moveTool([0,0,-0.2,0,0,0], v=0.1, block=True)
    #time.sleep(7.5)
    print('step 10 moveLine to home pose')
    robot.moveLine(pp[0], v=tip_speed, block=True)
    robot.moveLine(pp[1], v=tip_speed, block=True)
    robot.grip_open()
    robot.moveLine(pp[0], v=tip_speed, block=True)
    
    print('step 11 move to station B')
    target_station = step[1]
    sendJson(target_station, IP_WEBSERVER)
    time.sleep(5.0)
    t = time.time()
    while (time.time()-t) < 30.0:
        msgs = getStatus(IP_WEBSERVER)
        print(msgs)
        if msgs.find('Waiting for new missions') != -1:
            break
        time.sleep(0.1)
    
    robot.moveLine(pp[0], tip_speed, block=True)
    robot.moveLine(pp[1], v=tip_speed, block=True)
    robot.grip_close()
    robot.moveLine(pp[0], v=tip_speed, block=True)
    
    
    print('step 12 moveJ to camera ready position')
    new_pose = copy.deepcopy(jj[0])
    new_pose[0] = new_pose[0]+(math.pi/2.0)
    robot.moveJ(j=new_pose, v=1.0)
    
    print('step 13 moveTool y+0.1')
    offset_front = [0,0.2,0.05,0,0,0]
    #if target_station == 2:
    #    offset_front = [0,0.1,0,0,0,0]
    robot.moveTool(offset_front, v=tip_speed, block=True)
    #time.sleep(1.0)

    print('step 14 adjust start')
    adjust()
    print('step 15 grip open')
    robot.grip_open()
    print('step 16 moveTool z-0.4')
    robot.moveTool([0,0,-0.2,0,0,0], v=tip_speed, block=True)
    #time.sleep(5.0)
    print('step 17 moveLine to home pose')
    robot.moveLine(pp[0], tip_speed)
    
    print('end')
    del robot
   
