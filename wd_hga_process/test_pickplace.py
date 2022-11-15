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
target_cmd = [0,0,0,0,0,0]
dz = [0.0]

hsv_debug = False

class HSVDetection():
    def __init__(self, color):
        if color == 'pink':
            self.h=[110, 180]
            self.s=[20, 255]
            self.v=[0, 255]
            self.color = [123,82,179]
        elif color == 'orange':
            self.h=[17, 40]
            self.s=[20, 255]
            self.v=[20, 255]
            self.color = [57,179,231]
        elif color == 'red':
            self.h=[0, 10]
            self.s=[100, 255]
            self.v=[100, 255]
            self.color = [57,179,231]
        self.lower_bound = np.array([self.h[0],self.s[0],self.v[0]])
        self.upper_bound = np.array([self.h[1],self.s[1],self.v[1]])
    def process(self, hsv):
        mask = cv2.inRange(copy.deepcopy(hsv), self.lower_bound, self.upper_bound)
        mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8))
        mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #print(type(contours))
        lContour = []
        maxArea = 0
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            x, y, _w, _h = cv2.boundingRect(contours[i])
            s = min(_w,_h)/max(_w,_h)
            p = float(area) / float(_w*_h)
            if p > 0.55 and s > 0.5:
                if len(lContour) < 2:
                    lContour.append(contours[i])
                elif area > cv2.contourArea(lContour[0]):
                    lContour[0] = contours[i]
                elif area > cv2.contourArea(lContour[1]):
                    lContour[1] = contours[i]
        try:
            x0,y0,_,_ = cv2.boundingRect(lContour[0])
            x1,y1,_,_ = cv2.boundingRect(lContour[1])
            #print('{}, {} : before'.format((x0,y0), (x1,y1)))
        except:
            #print('except : x0,_,_,_ = cv2.boundingRect(lContour[0])')
            return None, contours
        if x1 < x0:
            tmp = copy.deepcopy(lContour[0])
            lContour[0] = copy.deepcopy(lContour[1])
            lContour[1] = copy.deepcopy(tmp)

        #x0,y0,_,_ = cv2.boundingRect(lContour[0])
        #x1,y1,_,_ = cv2.boundingRect(lContour[1])
        #print('{}, {}'.format((x0,y0), (x1,y1)))

        return lContour, contours
    def drawAll(self, _img, contours, debug=False):
        if contours == None:
            return False
        _color = self.color
        if debug:
            _color = [0,0,0]
        for i in range(len(contours)):
            '''
            area = cv2.contourArea(contours[i])
            x, y, _w, _h = cv2.boundingRect(contours[i])
            s = min(_w,_h)/max(_w,_h)
            p = float(area) / float(_w*_h)
            #print(s)
            if p > 0.55 and s > 0.5:
            '''
                #cv2.drawContours(image=_img, contours=contours, contourIdx=i, color=_color, thickness=cv2.FILLED)
            cv2.drawContours(image=_img, contours=contours, contourIdx=i, color=_color, thickness=2)


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

    if colorDetection2() == -1:
        return -1

    #print('threadDetection - iThreadRun : {}'.format(iThreadRun))
    iThreadRun = 2
    #print('threadDetection - iThreadRun : {}'.format(iThreadRun))

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

def find_center(contour):
    x, y, _w, _h = cv2.boundingRect(contour)
    cx = int(_w/2.0+x)
    cy = int(_h/2.0+y)

    # Center coordinates
    center_coordinates = (cx, cy)
    return center_coordinates

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

def detect_and_adjust(host_ip = '192.168.12.195', port = 1234):
    global frame_detect, iThreadRun, h, s, v, robot, bAutomate, oldTarget, debug_hsv, center_point, imageTheta, target_cmd, dz

    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);

    cnt=0
    #print('detect_and_adjust')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    client_socket.connect((host_ip,port)) # a tuple
    data = b""
    payload_size = struct.calcsize("Q")

    tShow = time.time()
    #print('while loop')

    kx=0.0005997452962869177
    ky=0.0010742776159686715

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
        offset = 10.0
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
        #ez = dz[0]-372.16259887312697
        ez = dz[0]-242.09
        target_theta = 3.112031375024515
        target_cx = width/2
        target_cy = height/2

        if bAutomate >= 10:
            target_cx = 464
            target_cy = 363
        
        bFirst = True

        '''
        robot.tip=[0.523255723993786, -0.10090175027911867, 0.33169211497291273], xy=(433, 132), dz=(242.10121850168372, -242, 7)
        robot.tip=[0.7445569128833323, -0.1007989001148236, 0.3317135924288006], xy=(438, 338), dz=(242.0743687382041, -242, 6)
        diff : robot.tip=[-0.2213011888895463, -0.00010285016429506522]
        diff : xy=[[-5, -206]]
        kx=-0.24289684499620168/-405=0.0005997452962869177
        ky=-0.2213011888895463/-206=0.0010742776159686715
        
        '''
        
        e, ex, ey = dist((target_cx, target_cy), center_point)
        e_theta = imageTheta-target_theta
        print("[{}], ex={}, ey={}, e_theta={}, ez={}".format(bAutomate, ex, ey, e_theta, ez))
        #print("robot.tip={}, xy={}, dz={}".format(robot.ur_rtde.tip_pos[:3], center_point, dz))
        if bAutomate == 1: # press k
            current_cmd = [0,0,0,0,0,0]
            if ey > 5:
                current_cmd[0] = -(ey*ky) #offset
            elif ey < -5:
                current_cmd[0] = (ey*ky)
            '''
            if ey > 5:
                current_cmd[1] = ey*ky #offset
            elif ey < -5:
                current_cmd[1] = -(ey*ky)
            '''
            print('current_cmd = ', current_cmd)
            robot.moveTool(current_cmd, v=0.005, block=False)
            bAutomate=0
        elif bAutomate == 2: # press k
            vel = [0,0,0,0,0,0]
            if abs(ey) < 5:
                robot.moveTool(vel, block=False)
        elif bAutomate == 3: # press k
            vel = [0,0,0,0,0,0]
            if abs(e_theta) < 0.01:
                robot.moveTool(vel, block=False)
        
        if bAutomate == 5 or bAutomate == 10: # press l
            current_cmd = [0,0,0,0,0,0]
            if e_theta > 0.01:
                current_cmd[5] = -offset
            elif e_theta < -0.01:
                current_cmd[5] = offset
            if ez > 1.0:
                current_cmd[2] = -offset
            elif ez < -1.0:
                current_cmd[2] = offset
            print('{}, {}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if not compare(np.sign(target_cmd), np.sign(current_cmd)):
                v = 0.025
                if current_cmd[5] != 0:
                    v=0.006
                if bAutomate >= 10:
                    v = 0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                if bAutomate == 5:
                    bAutomate = 0
                else:
                    bAutomate = 11
        elif bAutomate == 6 or bAutomate == 11:
            current_cmd = [0,0,0,0,0,0]
            if ex > 5:
                current_cmd[0] = offset
            elif ex < -5:
                current_cmd[0] = -offset
            if ey > 5:
                current_cmd[1] = offset
            elif ey < -5:
                current_cmd[1] = -offset
            print('{}, {}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if not compare(np.sign(target_cmd), np.sign(current_cmd)):
                v = 0.025
                if bAutomate >= 10 or ez > 0.0:
                    v = 0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                if bAutomate == 6:
                    bAutomate = 7
                else:
                    bAutomate = 12
        elif bAutomate == 7 or bAutomate == 12:
            current_cmd = [0,0,0,0,0,0]
            if ez > 1.0:
                current_cmd[2] = -offset
            elif ez < -1.0:
                current_cmd[2] = offset
            print('{}, {}'.format(target_cmd, current_cmd))
            #if target_cmd != current_cmd:
            if not compare(np.sign(target_cmd), np.sign(current_cmd)):
                v = 0.025
                if bAutomate >= 10:
                    v = 0.005
                #robot.stop()
                robot.moveTool(current_cmd, v, block=False)
                target_cmd = copy.deepcopy(current_cmd)
            if target_cmd == [0,0,0,0,0,0]:
                if bAutomate == 7:
                    bAutomate = 8
                else:
                    bAutomate = 13
        elif bAutomate == 8 or bAutomate == 13:
            current_cmd = [0,0,0,0,0,0]
            if ez > 7.0:
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
                current_cmd[5] = -offset
            elif e_theta < -0.005:
                current_cmd[5] = offset
            #print('{}, {}'.format(target_cmd, current_cmd))
            if target_cmd != current_cmd or bFirst:
                bFirst = False
                v = 0.002
                if current_cmd[5] != 0:
                    v=0.001
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
            robot.moveTool([0.02686939900984832, 0.08657685150415684,0.15,0,0,0], 0.05, block=True, exceptStop=True)
            while True:
                err = 0.0
                for x in robot.ur_rtde.joint_velo:
                    err += abs(x)
                if err < 0.001:
                    break
            print('moveTool([0,0,0.05,0,0,0]')
            robot.moveTool([0,0,0.05,0,0,0], 0.025, block=True, exceptStop=True)
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

        elif key == ord('t'):
            robot.moveTool([0.1,0,0,0,0,0], block=True)
        elif key == ord('r'):
            robot.moveTool([-0.1,0,0,0,0,0], block=True)

        '''
        if time.time()-tShow > 0.5:
            print(robot.ur_rtde.tip_pos)
            tShow = time.time()
        '''

    cv2.destroyAllWindows()
    client_socket.close()

def compare(a, b):
    n = len(a)
    if n != len(b):
        return False
    for i in range(n):
        if int(a[i]) != int(b[i]):
            return False
    return True

def Err(a, b):
    _err = 0.0
    for i in range(len(a)):
        _err += math.pow(a[i]-b[i], 2)
    return math.sqrt(_err)

if __name__ == '__main__':
    robot.init()

    detect_and_adjust()

    print('end')   
