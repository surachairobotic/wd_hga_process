import time, keyboard, math, requests, copy, threading
import socket, cv2, pickle, struct
import numpy as np

import matplotlib.pyplot as plt
from PIL import Image, ImageFont, ImageDraw
from ur_socket import *

frame_detect = None
errTheta = 0.0
iThreadRun = 0
robot = UR_SOCKET() # for move control
oldTarget = []
bAutomate = 0

debug_hsv = []
center_point = (0,0)
imageTheta = 0.0
target_theta = 3.0947258255584007
target_cmd = [0,0,0,0,0,0]
dz = [0.0]

class Pink():
    def __init__(self):
        self.h=[160, 175]
        self.s=[90, 255]
        self.v=[0, 255]

class Orange():
    def __init__(self):
        self.h=[14, 20]
        self.s=[100, 255]
        self.v=[100, 255]

class Green():
    def __init__(self):
        self.h=[14, 20]
        self.s=[170, 255]
        self.v=[140, 255]

def threadColorDetection():
    global frame_detect, iThreadRun
    
    #print('threadDetection')
    #frame_detect = cv2.resize(frame_detect,(848,480))

    if colorDetection() == -1:
        return -1

    #print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    iThreadRun = 2
    #print('threadDetection - iThreadRun : {}'.format(iThreadRun))

def colorDetection():
    global frame_detect, iThreadRun, h, s, v, robot, errTheta, debug_hsv, center_point, imageTheta, dz
    height, width, channels = frame_detect.shape

    '''
    points = np.array([[0, 0], [0, height], [int(width/10), height], [int(width/10), 0]])
    cv2.fillPoly(frame_detect, pts=[points], color=(0, 0, 0))
    points = np.array([[width, 0], [width, height], [width-int(width/10), height], [width-int(width/10), 0]])
    cv2.fillPoly(frame_detect, pts=[points], color=(0, 0, 0))
    '''

    # convert to hsv colorspace
    hsv = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (5,5))
    debug_hsv = copy.deepcopy(hsv)

    # lower bound and upper bound for Pink color
    pink = Pink()
    lower_bound = np.array([pink.h[0],pink.s[0],pink.v[0]])
    upper_bound = np.array([pink.h[1],pink.s[1],pink.v[1]])

    # find the colors within the boundaries
    maskPink = cv2.inRange(hsv, lower_bound, upper_bound)
    maskPink = cv2.erode(maskPink, np.ones((5, 5), dtype=np.uint8))
    maskPink = cv2.dilate(maskPink, np.ones((5, 5), dtype=np.uint8))

    # lower bound and upper bound for Orange color
    orange = Orange()
    lower_bound = np.array([orange.h[0],orange.s[0],orange.v[0]])
    upper_bound = np.array([orange.h[1],orange.s[1],orange.v[1]])

    maskOrange = cv2.inRange(hsv, lower_bound, upper_bound)
    maskOrange = cv2.erode(maskOrange, np.ones((5, 5), dtype=np.uint8))
    maskOrange = cv2.dilate(maskOrange, np.ones((5, 5), dtype=np.uint8))

    '''
    # lower bound and upper bound for Orange color
    green = Green()
    lower_bound = np.array([green.h[0],green.s[0],green.v[0]])
    upper_bound = np.array([green.h[1],green.s[1],green.v[1]])

    maskGreen = cv2.inRange(hsv, lower_bound, upper_bound)
    maskGreen = cv2.erode(maskGreen, np.ones((5, 5), dtype=np.uint8))
    maskGreen = cv2.dilate(maskGreen, np.ones((5, 5), dtype=np.uint8))
    '''
    
    # Now you can finally find contours.
    contours, hierarchy = cv2.findContours(maskPink.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pink_contour = getNearestContour(contours, (int(width/2), int(height/2)))

    if pink_contour is None:
        iThreadRun = 2
        return -1
    frame_detect, center_pink = draw(frame_detect, pink_contour, (0,255,0))

    contours, hierarchy = cv2.findContours(maskOrange.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    orange_contour = getNearestContour(contours, center_pink)

    '''
    contours, hierarchy = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    green_contour = getLargestContour(contours)
    '''

    if orange_contour is None:
        iThreadRun = 2
        return -1

    '''
    draw(debug_hsv, pink_contour, (0,255,0))
    draw(debug_hsv, orange_contour, (0,0,255))
    draw(debug_hsv, green_contour, (255,0,255))
    '''

    #draw2(frame_detect, contours, (0,0,255), center_pink)
    frame_detect, center_orange = draw(frame_detect, orange_contour, (0,0,255))
    #frame_detect, center_green = draw(frame_detect, green_contour, (255,0,255))
    
    cx = min(center_orange[0], center_pink[0])
    cx += ((max(center_orange[0], center_pink[0]) - cx) / 2)
    cx = int(cx)
    cy = min(center_orange[1], center_pink[1])
    cy += ((max(center_orange[1], center_pink[1]) - cy) / 2)
    cy = int(cy)
    center_point = (cx, cy)
    imageTheta = math.atan2(center_orange[1]-center_pink[1], center_orange[0]-center_pink[0])
    dz = dist(center_orange, center_pink)
    #print("center_point={}, imageTheta={}, dz={}".format(center_point, imageTheta, dz))
    if imageTheta < 0.0:
        imageTheta += (2*math.pi)
    '''
    j6 = robot.ur_rtde.joint_pos[5]
    target_theta = 3.0947258255584007
    4.66552215235
    errTheta = imageTheta-target_theta
    print("theta={}, dist={}, orange[{}], pink[{}], j6={}".format(imageTheta, dist(center_orange, center_pink), center_orange, center_pink, j6))
    '''

    '''
    if imageTheta-3.1 > 0.1:
        js = copy.deepcopy(robot.ur_rtde.joint_pos)
        js[5] += 0.05
        robot.moveJ(js)
    '''

    # theta=3.0947258255584007, dist=597.6562557189542, orange[(196, 171)], pink[(793, 143)]
    return 1

def dist(p1, p2):
    dx = p1[0]-p2[0]
    dy = p1[1]-p2[1]
    return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)), dx, dy

def getLargestContour(contours):
    final_contours = None
    final_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > final_area:
            final_contours = contour
            final_area = area
    try:
        x, y, _w, _h = cv2.boundingRect(final_contours)
    except:
        return None
    return final_contours
def getNearestContour(contours, refPoint):
    final_contours = None
    final_err = 9999
    for contour in contours:
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        err,_,_ = dist((cX,cY), refPoint)
        if err < final_err:
            final_contours = contour
            final_err = err
    try:
        x, y, _w, _h = cv2.boundingRect(final_contours)
    except:
        return None
    return final_contours


def draw(frame, contour, color):
    #print(x, " ", y, " ", w, " ", h)
    x, y, _w, _h = cv2.boundingRect(contour)
    cx = int(_w/2.0+x)
    cy = int(_h/2.0+y)

    # Center coordinates
    center_coordinates = (cx, cy)
            
    # Using cv2.circle() method
    # Draw a circle with blue line borders of thickness of 2 px
    frame = cv2.circle(frame, center_coordinates, radius=2, color=color, thickness=2)

    #for i in range(len(contours)):
    #    cv2.drawContours(image=frame_detect, contours=contours, contourIdx=i, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    #cv2.drawContours(img, final_contours, i, np.array([50, 250, 50]), 4)
    cv2.drawContours(image=frame, contours=contour, contourIdx=-1, color=color, thickness=2, lineType=cv2.LINE_AA)
    #print([_w, _h])
    #pt1 = (int(x),int(y))
    #pt2 = (int(x+_w),int(y+_h))
    #f1 = copy.deepcopy(frame_detect)
    #cv2.rectangle(img=frame_detect, start_point=pt1, end_point=pt2, color=(255, 255, 0), thickness=2)

    #res = cv2.bitwise_and(img, img, mask=mask)
    return frame, (cx,cy)
def draw2(frame, contours, color, ref):
    for i in range(len(contours)):
        cv2.drawContours(image=frame_detect, contours=contours, contourIdx=i, color=color, thickness=2, lineType=cv2.LINE_AA)
    return frame

def detect_and_adjust(host_ip = '192.168.12.200', port = 5678): #192.168.12.200
    global frame_detect, iThreadRun, h, s, v, robot, bAutomate, oldTarget, debug_hsv, center_point, imageTheta, target_theta, target_cmd, dz

    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);

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
        
        #frame2, err = colorDetection(frame)
        #print(err)
        #####################################################################
        #print('iThreadRun : {}'.format(iThreadRun))
        #print("h={}, s={}, v={}".format(h,s,v))
        if iThreadRun == 0:
            #print('in iThreadRun')
            iThreadRun = 1
            frame_detect = copy.deepcopy(frame)
            threadStatus = threading.Thread(target=threadColorDetection)
            threadStatus.start()
        elif iThreadRun == 2:
            #print("finish1")
            vis = np.concatenate((frame_detect, debug_hsv), axis=1)
            cv2.imshow("Detection", vis)
            #cv2.imshow("Detection", debug_hsv)
            #print("finish2")
            iThreadRun = 0            
        
        #frame = cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        #cv2.imwrite('ham_scale.png',frame)
        #cv2.imshow("RECEIVING VIDEO", frame)
        #print('FPS : ' + str(1.0/(time.time()-t)))
        offset = 0.1
        key = cv2.waitKey(10) & 0xFF

        
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
        if bAutomate == 1: # press k
            vel = [0,0,0,0,0,0]
            if abs(ex) < 5:
                robot.moveTool(vel, block=False)
        elif bAutomate == 2: # press k
            vel = [0,0,0,0,0,0]
            if abs(ey) < 5:
                robot.moveTool(vel, block=False)
        elif bAutomate == 3: # press k
            vel = [0,0,0,0,0,0]
            if abs(e_theta) < 0.01:
                robot.moveTool(vel, block=False)
        
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
            if target_cmd != current_cmd:
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
            robot.moveTool([0,0,0.05,0,0,0], 0.01, block=True, exceptStop=True)
            bAutomate = 0
        
        if key == ord('s'):
            print('stop')
            robot.stop()
        elif key == ord('u'):
            print('zero speed')
            robot.moveTool([0,0,0.05,0,0,0], 0.01, block=True, exceptStop=True)
            #robot.moveTool([0,0,0,0,0,0], block=False)
        elif key == ord('q'):
            break
        elif key == ord('+'):
            robot.moveTool([0,0,0,0,0,-offset], v=0.005, block=False)
        elif key == ord('-'):
            robot.moveTool([0,0,0,0,0,offset], v=0.005, block=False)

        elif key == ord('4'):
            print('x+')
            robot.moveTool([-offset,0,0,0,0,0], block=False)
        elif key == ord('1'):
            print('x-')
            robot.moveTool([offset,0,0,0,0,0], block=False)

        elif key == ord('5'):
            robot.moveTool([0,-offset,0,0,0,0], block=False)
        elif key == ord('2'):
            robot.moveTool([0,offset,0,0,0,0], block=False)

        elif key == ord('6'):
            robot.moveTool([0,0,offset,0,0,0], block=False)
        elif key == ord('3'):
            robot.moveTool([0,0,-offset,0,0,0], block=False)

    cv2.destroyAllWindows()
    client_socket.close()

def Err(a, b):
    _err = 0.0
    for i in range(len(a)):
        _err += math.pow(a[i]-b[i], 2)
    return math.sqrt(_err)

if __name__ == '__main__':
    robot.init()

    detect_and_adjust()

    print('end')   
