import time, keyboard
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *

# Gripper Variables
ACT = "ACT"
GTO = "GTO"
ATR = "ATR"
ARD = "ARD"
FOR = "FOR"
SPE = "SPE"
OBJ = "OBJ"
STA = "STA"
FLT = "FLT"
POS = "POS"

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"

def main(args=None):

    ur = UR_SOCKET()
    ur.init()
    ur.stop()    
    
    jj = [  [1.6100164651870728, -1.64317049602651, 2.4955900351153772, -3.99554242710256, -1.6224005858050745, 3.14430570602417],
            [4.368937015533447, -1.2535660129836579, 2.224023167287008, -4.138756414453024, -1.2165988127337855, 3.145020008087158],
            [4.37146520614624, -0.012463168506958056, 1.7640226523028772, -4.91780223468923, -1.2204092184649866, 3.1394753456115723]
         ]
    pp = [  [ 0.14075575758431658, -0.5505126556505817,  0.3912572106874151,  1.5674877209203752, -0.00001, -0.00001],
            [-0.01376703336070136,  0.7046249426517024,  0.3499899351397665,  0.0305088624493274, -2.19006, -2.24545],
            [-0.013694179808458936, 0.7046214638825812, -0.11002220052983638, 0.030255677140335162, -2.190209656127298, -2.2453879746576333]
            
         ]
    
    '''
    end = pp[2]
    end[0] += 0.1
    end[1] += 0.23
    ur.moveLine(end)
    
    exit()
    '''
    
    '''
    0 = 8
    0.1865 = -290
    '''
    
    fw = 0
    if fw == 0:
        ur.moveLine(pp[2], 0.05)
        #time.sleep(5)
        #exit()
        # create socket
        client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        host_ip = '192.168.8.136' # paste your server ip address here
        print('Enter port : ')
        port = int(input())
        client_socket.connect((host_ip,port)) # a tuple
        data = b""
        payload_size = struct.calcsize("Q")
        state = '*'
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
            frame2, err = colorDetection(frame)
            cv2.imshow("RECEIVING VIDEO", frame2)
            #err += 47
            print(err)

            if abs(err) < 1:
                ur.stop()
                state = '*'
            #    break
            else:
                '''
                y = (-0.000625838926174*err) - 0.04
                end = pp[2]
                end[0] = pp[2][0] + y
                print(err, " : ", y, " : ", end[0])
                ur.moveLine(end, 0.05)
                input()
                end[1] += 0.21
                ur.moveLine(end, 0.05)
                break
                '''

                print('state : ', state)
                end = pp[2]
                if err > 0:
                    if state != '+':
                        end[0] = pp[2][0] - 1.0
                        print('move -1.0')
                        ur.moveLine(end, 0.02)
                        state = '+'
                else:
                    if state != '-':
                        print('move +1.0')
                        end[0] = pp[2][0] + 1.0
                        ur.moveLine(end, 0.02)
                        state = '-'

            #print('FPS : ' + str(1.0/(time.time()-t)))
            key = cv2.waitKey(1) & 0xFF
            if key  == ord('q'):
                break
        client_socket.close()
        '''
        end = pp[2]
        end[0] += 0.1865
        ur.moveLine(end)
        '''

    elif fw == 1:
        ur.moveJ(jj[0])
        input()
        ur.moveJ(jj[1])
        input()
        ur.moveLine(pp[2])
        input()
        end = pp[2]
        end[0] += 0.1
        end[1] += 0.23
        ur.moveLine(end)
        input() # G-
        ur.moveLine(pp[1])        
        input()
        ur.moveJ(jj[0])
        # G+
    elif fw == 2: # back
        ur.moveJ(jj[2])
        input()
        ur.moveLine(pp[1])
        input()
        ur.moveJ(jj[0])
    elif fw == 3: # forward
        ur.moveLine(pp[1])
        input()
        ur.moveJ(jj[0])    
    elif fw == 10: # forward
        ur.moveJ(jj[0])
        input()
        ur.moveJ(jj[1])
        input()
        ur.moveLine(pp[2])
    #print(ur.read())
    ur.stop()
    exit()


    HOST = "192.168.12.100"
    PORT = 30002
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    d = [90,-90,140,-237,-90,180]
    j = deg2rad(d)
    print(d)
    print(j)

    '''
    def _set_gripper_activate(self):
        self._socket_set_var(GTO, 1, self.socket_name)

    msg = "socket_set_var(\"{}\",{},\"{}\")".format(var, value, socket_name)  # noqa
    '''

    #cmd = "set_digital_out(0,False)" + "\n"


    var = GTO
    value = 1
    socket_name = SOCKET_NAME
    socket_host = SOCKET_HOST
    socket_port = SOCKET_PORT
    msg = "\tsocket_close(\"{}\")".format(socket_name)

    msg = msg + "\n\t" + "socket_open(\"{}\",{},\"{}\")".format(socket_host,
                                                 socket_port,
                                                 socket_name)

    msg = msg + "\n\t" + "socket_set_var(\"{}\",{},\"{}\")".format(var, value, socket_name)  # noqa
    

    cmd = [ "rq_activate()\n",
    "rq_reset()\n",
    "movej([1.57,-1.57,2.45,-4.13,-1.57,3.14],a=1.0,v=0.5,t=0,r=0)" + "\n",
    "movej([1.0,-1.57,2.45,-4.13,-1.57,3.14],a=1.0,v=0.5,t=0,r=0)" + "\n",
    ]
    indx = 0
    #cmd = "get_actual_joint_positions()" + "\n"    

    myprog = """def myProg():{}\nend""".format(msg)

    
    loop = True
    while loop:
        if keyboard.is_pressed('q'):
            break
        elif keyboard.is_pressed('w'):
            s.send(cmd[indx].encode('utf-8'))
            data = s.recv(1024)
            
            indx = (indx+1) % 2
        time.sleep(0.05)
    
    if not loop:
        s.send(cmd[0].encode('utf-8'))
        data = s.recv(1024)
        print('Recv : ', str(data))
        time.sleep(0.1)
        s.send(cmd[1].encode('utf-8'))
        data = s.recv(1024)
        print('Recv : ', str(data))
    
    s.close()

    '''    
    print('Recv : ', str(data))
    print('-----')
    for x in data:
        print(str(x) + ', ' + str(chr(x)))
    '''

def deg2rad(_in):
    _out = [(j/180.0*3.14) for j in _in]
    return _out

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
    final_area = 6000
    bNotDetect = True
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > final_area:
            final_contours = contour
            final_area = area
            bNotDetect = False

    print(final_area)
    if bNotDetect:
        return img, 0
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

    return img, err+47

if __name__ == '__main__':
    main()

