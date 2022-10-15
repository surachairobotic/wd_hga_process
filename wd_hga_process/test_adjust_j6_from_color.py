import time, keyboard, math, requests, copy, threading
import socket, cv2, pickle, struct
import numpy as np

import matplotlib.pyplot as plt
from PIL import Image, ImageFont, ImageDraw

frame_detect = None
iThreadRun = 0

h=[14, 20]
s=[70, 255]
v=[80, 255]

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
    global frame_detect, iThreadRun, h, s, v
    height, width, channels = frame_detect.shape

    # convert to hsv colorspace
    hsv = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (10,10)) 

    # lower bound and upper bound for Green color
    #lower_bound = np.array([50,50,50])
    #upper_bound = np.array([150,255,255])

    # lower bound and upper bound for Red color
    lower_bound = np.array([h[0],s[0],v[0]])
    upper_bound = np.array([h[1],s[1],v[1]])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8))
    mask = cv2.dilate(mask, np.ones((5, 5), dtype=np.uint8))
    
    # Now you can finally find contours.
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

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
        iThreadRun = 2
        return -1
    #print(x, " ", y, " ", w, " ", h)
    cx = int(_w/2.0+x)
    cy = int(_h/2.0+y)

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
    frame_detect = cv2.circle(frame_detect, center_coordinates, radius, color, thickness)

    #for i in range(len(contours)):
    #    cv2.drawContours(image=frame_detect, contours=contours, contourIdx=i, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    #cv2.drawContours(img, final_contours, i, np.array([50, 250, 50]), 4)
    cv2.drawContours(image=frame_detect, contours=final_contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    print([_w, _h])
    #pt1 = (int(x),int(y))
    #pt2 = (int(x+_w),int(y+_h))
    #f1 = copy.deepcopy(frame_detect)
    #cv2.rectangle(img=frame_detect, start_point=pt1, end_point=pt2, color=(255, 255, 0), thickness=2)

    #res = cv2.bitwise_and(img, img, mask=mask)
    err = int(width/2.0) - cx

    return err


def detect_and_adjust():
    global frame_detect, iThreadRun, h, s, v

    cv2.namedWindow("Detection", cv2.WINDOW_AUTOSIZE);

    cnt=0
    #print('detect_and_adjust')
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    host_ip = '192.168.137.49' # paste your server ip address here
    port = 1111
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
    detect_and_adjust()

    print('end')   
