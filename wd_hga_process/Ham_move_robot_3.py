import time, keyboard, math, requests, copy
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *
from ur_rtde import *

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"
tip_speed = 0.075

def calError(actual, target):
    e = 0
    for i in range(len(actual)):
        e += math.sqrt(pow(actual[i]-target[i], 2))
    return e
def Pick_From_Station(ur, ur_rtde): #1-4 waypoint
    tip_speed = 0.125
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

    print('step 1 grip open')
    mission_id = {"mission_id": pos[0]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)

    print('step 2 ur.moveLine(pp[0]) waypoint 1')
    ur.moveLine(pp[0])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[0])
        print(e)
        if e < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)

    print('step 3 ur.moveLine(pp[1]) waypoint 2')
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
    
    print('step 4 grip close')
    mission_id = {"mission_id": pos[1]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(6.0)

    print('step 5 ur.moveLine(pp[2]) waypoint 3')
    ur.moveLine(pp[2], tip_speed)
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[2])
        if e < 0.001:
            break
        print(e)

        time.sleep(0.25)
    ur.stop()
    time.sleep(0.1)
    
    print('step 6 ur.moveLine(pp[3]) waypoint 4')
    ur.moveLine(pp[3], tip_speed)
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[3])
        if e < 0.001:
            break
        print(e)

        time.sleep(0.25)
    ur.stop()
    time.sleep(0.1)
    
def PickPlaceOnCar(ur, ur_rtde):
    ur.stop()
    
    jj = [  [1.1854966878890991, -1.86033358196401, 2.1157615820514124, -1.8081351719298304, -1.5688279310809534, 2.7654571533203125],#5
            [1.1807478666305542, -1.8768779240050257, 1.668762509022848, -1.3510250610164185, -1.56681996980776, 2.7385597229003906],#6
            [1.1807717084884644, -1.8731304607787074, 1.6466658751117151, -1.3327413511327286, -1.5668709913836878, 2.7384324073791504],#7
            [2.9175713062286377, -1.2228343945792695, 1.6495335737811487, -1.9958745441832484, -1.5631216208087366, 2.8716158866882324],#8
            [2.917159080505371, -0.9940787118724366, 1.8037403265582483, -2.3789531193175257, -1.5647771994220179, 2.873798131942749],#9
            [2.918501138687134, -1.2875669759562989, 1.0661338011371058, -1.3475923401168366, -1.5630133787738245, 2.867243528366089]#1
         
         ]
    pp = [  [-0.01017494401209725, -0.3756525442585086, 0.13840573695699745,-3.1241048704483654, 0.011357524148814309, -0.0011440264047513579],#5
            [-0.011850422601318423, -0.37585862537959563, 0.317095305821409,-3.1308455555118337,-0.017135301633691005,-0.0007322231555360971],#6
            [-0.011868193208126201, -0.3758594707806762, 0.32461148496658865,-3.130902314470439,-0.017127665212704805, -0.0006788930909500954],#7
            [0.6157319922335678, -0.0033028450931117614, 0.06900699254837098,-2.1666266010453423, -2.274861107011292, -4.6540396680441666e-05],#8
            [0.6157760979216934, -0.003319206398711913, -0.09569432134831601,2.1666037432212284, 2.2748633605956563, 6.43388420211832e-05],#9
            [0.615705930495875, -0.0034379305093566524, 0.32595563912016234,-2.166769598094382, -2.274694779560951, 0.0002946796220985595]#10
         ]
    pos = [ 'e0f8f53c-02bd-11ed-90f4-0001299a3e90', # grip open
            'd3ab6322-02bd-11ed-90f4-0001299a3e90', # grip close
          ]
    
    print('step 7 ur.moveLine(pp[0]) waypoint 5')
    ur.moveLine(pp[0], tip_speed)
    time.sleep(1.0)

    while True:
        e = calError(ur_rtde.joint_pos, jj[0])
        print(e)
        if calError(ur_rtde.joint_pos, jj[0]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)
    

    # Get Request
    robot_ip='192.168.12.20'
    host = 'http://' + robot_ip + '/api/v2.0.0/'

    # Format Headers
    headers = {}
    headers['Content-Type'] = 'application/json'
    headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
    #print(headers)

    print('step 8 grip open')
    mission_id = {"mission_id": pos[0]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)
    
    
    print('step 9 ur.moveLine(pp[1]) waypoint 6')
    ur.moveLine(pp[1])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[1])
        print(e)
        if calError(ur_rtde.joint_pos, jj[1]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    
    print("step 10 wait 10 sec")
    time.sleep(10)
    
    print('step 11 ur.moveLine(pp[0]) waypoint 5')
    ur.moveLine(pp[0])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[0])
        print(e)
        if calError(ur_rtde.joint_pos, jj[0]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)
    
    print('step 12 grip close')
    mission_id = {"mission_id": pos[1]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)
    
    
    print('step 13 ur.moveLine(pp[2]) waypoint 7')

    ur.moveLine(pp[2])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[2])
        print(e)
        if calError(ur_rtde.joint_pos, jj[2]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)
    
    print('step 14 ur.moveLine(pp[3]) waypoint 8')

    ur.moveLine(pp[3])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[3])
        print(e)
        if calError(ur_rtde.joint_pos, jj[3]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1)    

    print('step 15 ur.moveLine(pp[4]) waypoint 9')

    ur.moveLine(pp[4])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[4])
        print(e)
        if calError(ur_rtde.joint_pos, jj[4]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1) 
    
    print('step 16 grip open')
    mission_id = {"mission_id": pos[0]}
    post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
    time.sleep(5.0)
    
    print('step 17 ur.moveLine(pp[5]) waypoint 10')
    ur.moveLine(pp[5])
    time.sleep(1.0)
    while True:
        e = calError(ur_rtde.joint_pos, jj[5])
        print(e)
        if calError(ur_rtde.joint_pos, jj[5]) < 0.001:
            break
        time.sleep(0.05)
    ur.stop()
    time.sleep(0.1) 
    
    

  
if __name__ == '__main__':
    ur = UR_SOCKET() # for move control
    ur.init()

    ur_rtde = UR_INFORMATION() # get current robot state

    print('Pick_From_Station')
    Pick_From_Station(ur, ur_rtde)
    print('PickPlaceOnCar')
    PickPlaceOnCar(ur, ur_rtde)
    print('end')

   
