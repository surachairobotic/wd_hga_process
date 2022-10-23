import time, keyboard, math, requests, copy, threading
import socket, cv2, pickle, struct
import numpy as np

import matplotlib.pyplot as plt
from PIL import Image, ImageFont, ImageDraw
from ur_socket import *

frame_detect = None
hsv = None
gray = None
iThreadRun = 0
robot = UR_SOCKET() # for move control

class Pink():
    def __init__(self):
        self.h=[160, 175]
        self.s=[90, 255]
        self.v=[0, 255]

class Orange():
    def __init__(self):
        self.h=[14, 20]
        self.s=[200, 255]
        self.v=[160, 255]


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
    global frame_detect, iThreadRun, h, s, v, robot, hsv, gray
    height, width, channels = frame_detect.shape

    # convert to hsv colorspace
    hsv = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (5,5))
    
    gray = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2GRAY)
    #gray = cv2.blur(gray, (10,10))

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
    
    # Now you can finally find contours.
    contours, hierarchy = cv2.findContours(maskPink.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pink_contour = getLargestContour(contours)

    contours, hierarchy = cv2.findContours(maskOrange.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    orange_contour = getLargestContour(contours)

    if pink_contour is None and orange_contour is None:
        iThreadRun = 2
        return -1

    frame_detect, center_pink = draw(frame_detect, pink_contour, (0,255,0))
    frame_detect, center_orange = draw(frame_detect, orange_contour, (0,0,255))
    
    #print("{} : {} : {}".format(center_orange, center_pink, math.atan2(center_orange[1]-center_pink[1], center_orange[0]-center_pink[0])))
    imageTheta = math.atan2(center_orange[1]-center_pink[1], center_orange[0]-center_pink[0])
    j6 = robot.ur_rtde.joint_pos[5]
    print("{} : {}".format(imageTheta, j6))
    '''
    if imageTheta-3.1 > 0.1:
        js = copy.deepcopy(robot.ur_rtde.joint_pos)
        js[5] += 0.05
        robot.moveJ(js)
    '''

    return 1

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

def detect_and_adjust(host_ip = '192.168.137.49', port = 1234):
    global frame_detect, iThreadRun, h, s, v, robot

    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);
    cv2.namedWindow("HSV", cv2.WINDOW_AUTOSIZE);
    #cv2.namedWindow("Gray", cv2.WINDOW_AUTOSIZE);

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
            cv2.imshow("Detection", frame_detect)
            cv2.imshow("HSV", hsv)
            #cv2.imshow("Gray", gray)
            #print("finish2")
            iThreadRun = 0            
        
        #frame = cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        #cv2.imwrite('ham_scale.png',frame)
        #cv2.imshow("RECEIVING VIDEO", frame)
        #print('FPS : ' + str(1.0/(time.time()-t)))
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):
            h[0] = min(h[0]+1, 255)
        elif key == ord('z'):
            h[0] = max(h[0]-1, 0)

        elif key == ord('s'):
            h[1] = min(h[1]+1, 255)
        elif key == ord('x'):
            h[1] = max(h[1]-1, 0)

        elif key == ord('d'):
            s[0] = min(s[0]+1, 255)
        elif key == ord('c'):
            s[0] = max(s[0]-1, 0)

        elif key == ord('f'):
            s[1] = min(s[1]+1, 255)
        elif key == ord('v'):
            s[1] = max(s[1]-1, 0)
            
        elif key == ord('g'):
            v[0] = min(v[0]+1, 255)
        elif key == ord('b'):
            v[0] = max(v[0]-1, 0)

        elif key == ord('h'):
            v[1] = min(v[1]+1, 255)
        elif key == ord('n'):
            v[1] = max(v[1]-1, 0)

    cv2.destroyAllWindows()
    client_socket.close()
  
if __name__ == '__main__':
    robot.init()

    detect_and_adjust()

    print('end')   
