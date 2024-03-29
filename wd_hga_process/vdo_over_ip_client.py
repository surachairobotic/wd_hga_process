# lets make the client code
import socket, cv2, pickle, struct, time
import numpy as np
import torch
from PIL import Image, ImageFont, ImageDraw
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
                 
    return img, err

def main():
    # create socket
    ############################################
    model = torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_top_TINY.pt', source='local')
    print("model load DONE")
    ###########################################
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip = '192.168.137.49' # paste your server ip address here
    port = 1111
    #print('Enter port : ')
    #port = int(input())
    client_socket.connect((host_ip, port)) # a tuple
    data = b""
    payload_size = struct.calcsize("Q")
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
        
        #####################################################
        out2=model(frame)
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
            thickness = 2
            frame = cv2.rectangle(frame, start_point, end_point, color, thickness)
        #####################################################
        
        #frame2, err = colorDetection(frame)
        #print(err)
        cv2.imshow("RECEIVING VIDEO", frame)
        #print('FPS : ' + str(1.0/(time.time()-t)))
        key = cv2.waitKey(1) & 0xFF
        if key  == ord('q'):
            break
    client_socket.close()
	
if __name__ == '__main__':
    main()
