import time, keyboard, math, requests, copy
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *
from ur_rtde import *

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

def PickPlaceOnCar(ur, ur_rtde):
    ur.stop()
    
    jj = [  [1.2282694578170776, -1.7880450687804164, 2.1002982298480433, -1.881458421746725, -1.5631631056415003, 2.814305543899536],
            [1.2276793718338013, -1.68815340618276, 2.2516377607928675, -2.1327573261656703, -1.5639894644366663, 2.815645694732666],
         ]
    pp = [  [-0.001968705070039036, -0.4036118059006364, 0.12632014942622866,  0.00001, 0.00001, 0.00001],
            [-0.0019853880799577347, -0.4036254646836985, 0.04449670896032401,  0.00001, 0.00001, 0.00001],
         ]
    
    #print('jj[0]')
    ur.moveJ(jj[0])
    time.sleep(1.0)
    '''
    while True:
        print(calError(ur_rtde.j, jj[0]))
        time.sleep(0.1)
    '''
    while True:
        if calError(ur_rtde.joint_pos, jj[0]) < 0.001:
            break
        print('running1')
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)
    #print('jj[1]')
    ur.moveJ(jj[1])
    time.sleep(1.0)
    while True:
        if calError(ur_rtde.joint_pos, jj[1]) < 0.001:
            break
        print('running2')
        time.sleep(0.05)
    ur.stop()

    pos = [ 'e0f8f53c-02bd-11ed-90f4-0001299a3e90', # G+
            'd3ab6322-02bd-11ed-90f4-0001299a3e90',
          ]

    # Get Request
    robot_ip='192.168.12.20'
    host = 'http://' + robot_ip + '/api/v2.0.0/'

    # Format Headers
    headers = {}
    headers['Content-Type'] = 'application/json'
    headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
    #print(headers)

    mission_id = {"mission_id": pos[1]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)

    ur.moveJ(jj[0])
    time.sleep(1.0)
    while True:
        if calError(ur_rtde.joint_pos, jj[0]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)
    ur.moveJ(jj[1])
    time.sleep(1.0)
    while True:
        if calError(ur_rtde.joint_pos, jj[1]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()

    mission_id = {"mission_id": pos[0]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)

    ur.moveJ(jj[0])
    time.sleep(1.0)
    while True:
        #ur.moveJ(jj[0])
        e = calError(ur_rtde.joint_pos, jj[0])
        if e < 0.001:
            break
        time.sleep(0.5)
    print('ur.stop()')
    ur.stop()

def PickPlaceOnStation(ur, ur_rtde):
    tip_speed = 0.125
    ur.stop()
    
    jj = [  [2.853015184402466, -1.521887083803751, 1.3489940802203577, -1.3963006299785157, -1.5629175345050257, 2.868340492248535],
            [2.917309045791626, -1.149418131714203, 1.728659454976217, -2.1483165226378382, -1.5637615362750452, 2.8725500106811523],
            [2.917154550552368, -0.9958768051913758, 1.803354565297262, -2.3766147098936976, -1.5648234526263636, 2.8738040924072266],
         ] # radian
    pp = [  [0.5238256358393917, -0.016707621935164344, 0.32373465470222984,  2.2406576759783845, 2.202056688146876, 1.918706557866698e-05],
            [0.6156507074236995, -0.003378447663110685, 0.005003157466865166, -2.166706856318857, -2.2747083377912247, 0.0001657433156371269],
            [0.6157324570241369, -0.0033541572824511605, -0.09466795884229998, 2.166647904084402, 2.2748494254731235, -0.00013407364599941598]
         ] # [x,y,z,r,p,y]
    
    pos = [ 'e0f8f53c-02bd-11ed-90f4-0001299a3e90', # grip open
            'd3ab6322-02bd-11ed-90f4-0001299a3e90', # grip close
          ]
    # Get Request
    robot_ip='192.168.12.20'
    host = 'http://' + robot_ip + '/api/v2.0.0/'
    # Format Headers
    headers = {}
    headers['Content-Type'] = 'application/json'
    headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
    #print(headers)

    print('140 grip open')
    mission_id = {"mission_id": pos[0]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)

    print('145 ur.moveJ(jj[0])')
    ur.moveJ(jj[0])
    time.sleep(1.0)
    while True:
        if calError(ur_rtde.joint_pos, jj[0]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)

    print('155 ur.moveLine(pp[1])')
    ur.moveLine(pp[1], tip_speed)
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[1])
        if e < 0.001:
            break
        print(e)
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)

    print('165 ur.moveLine(new_pp)')
    print('-----')
    print(pp[1])
    new_pp = copy.deepcopy(pp[1])
    new_pp[2] = new_pp[2]-0.1
    print(pp[1])
    print('-----')
    ur.moveLine(new_pp, tip_speed)
    time.sleep(1.0)
    while True:
        print('new_pp loop')
        e = calError(ur_rtde.joint_pos, jj[2])
        if e < 0.45: #0.00175
            break
        print(e)
        time.sleep(0.5)
    ur.stop()
    time.sleep(0.1)
    
    print('181 grip close')
    mission_id = {"mission_id": pos[1]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(6.0)

    print('186 ur.moveLine(pp[1])')
    ur.moveLine(pp[1], tip_speed)
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[1])
        if e < 0.001:
            break
        print(e)
        '''
        print('e')
        print('ur_rtde.j')
        print(ur_rtde.j)
        print('jj[1]')
        print(jj[1])
        '''
        time.sleep(0.25)
    ur.stop()
    time.sleep(0.1)

    print('205 pp[1][2]-0.1')
    print('-----')
    print(pp[1])
    new_pp = copy.deepcopy(pp[1])
    new_pp[2] = new_pp[2]-0.1
    print(pp[1])
    print('-----')
    ur.moveLine(new_pp, tip_speed)
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[2])
        if e < 0.00175:
            break
        print(e)
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)

    print('221 grip open')
    mission_id = {"mission_id": pos[0]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(6.0)

    print('pp[1]')
    ur.moveLine(pp[1], tip_speed)
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[1])
        if e < 0.005:
            break
        print(e)
        time.sleep(0.05)
    ur.stop()

    return 1
    
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

def calError(actual, target):
    e = 0
    for i in range(len(actual)):
        e += math.sqrt(pow(actual[i]-target[i], 2))
    return e

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
    ur = UR_SOCKET() # for move control
    ur.init()

    ur_rtde = UR_INFORMATION() # get current robot state

    print('PickPlaceOnStation')
    PickPlaceOnStation(ur, ur_rtde)
    print('PickPlaceOnCar')
    PickPlaceOnCar(ur, ur_rtde)
    print('end')

