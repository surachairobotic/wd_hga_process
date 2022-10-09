import time, keyboard, math, requests, copy
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"
tip_speed = 0.1

enable_grip = False

def ham_detect_and_adjust(ur):
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip = '192.168.137.49' # paste your server ip address here
    port = 1111
    client_socket.connect((host_ip,port)) # a tuple
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
        #frame2, err = colorDetection(frame)
        #print(err)
                
        #frame = cv2.imread('/home/cmit/dev_ws/ham_image/rgb_0.png')
        cv2.imshow("RECEIVING VIDEO", frame)
        #print('FPS : ' + str(1.0/(time.time()-t)))
        key = cv2.waitKey(1) & 0xFF
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

    print('step 2 ur.moveLine(pp[0]) waypoint 1')
    ur.moveLine(pp[0], tip_speed)
    
    return -1
    
    print('step 3 ur.moveLine(pp[1]) waypoint 2')
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
    ur = UR_SOCKET() # for move control
    ur.init()

    #ur_rtde = UR_INFORMATION() # get current robot state

    print('Pick_From_Station')
    Pick_From_Station(ur)
    print('PickPlaceOnCar')
    #PickPlaceOnCar(ur)
    print('end')

    del ur
   
