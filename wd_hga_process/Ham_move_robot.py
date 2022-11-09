import time, keyboard, math, requests, copy
import socket, cv2, pickle, struct
import numpy as np
from ur_socket import *
from ur_rtde import *

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"
tip_speed = 0.075
ur = UR_SOCKET() # for move control
ur.init()

ur_rtde = UR_INFORMATION() # get current robot state

def calError(actual, target):
    e = 0
    for i in range(len(actual)):
        e += math.sqrt(pow(actual[i]-target[i], 2))
    return e

'''
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

mission_id = {"mission_id": pos[0]}
post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)
time.sleep(5.0)
print ("DONE!!!")
'''

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

#print('jj[0]')
ur.moveJ(jj[0])
time.sleep(1.0)
print("**************************JJ MOVE ********************************************")
while True:
    e = calError(ur_rtde.joint_pos, jj[0])
    print("break 1:{}".format(e))
    if e < 0.01:
        break
    print('running1')
    time.sleep(0.05)
ur.stop()
time.sleep(0.1)
#print('jj[1]')
ur.moveJ(jj[1])
time.sleep(1.0)
while True:
    e2 = calError(ur_rtde.joint_pos, jj[1])
    print("break 2:{}".format(e2))
    if e2 < 0.01:
        break
    print('running2')
    time.sleep(0.05)
ur.stop()


print("***************************PP MOVE*****************************************")
print(pp[1])
new_pp = copy.deepcopy(pp[1])
new_pp[2] = new_pp[2]-0.1
print(pp[1])
print('-----')
ur.moveLine(new_pp, tip_speed)
while True:
        e = calError(ur_rtde.joint_pos, jj[2])
        if e < 0.00175:
            break
        print(e)
        time.sleep(0.5)
ur.stop()
time.sleep(0.1)

ur.stop()
print(pp[0])
print('-----')
ur.moveLine(pp[1], tip_speed)
time.sleep(1.0)
while True:
    e = calError(ur_rtde.joint_pos, jj[1])
    if e < 0.001:
        break
    print(e)
    time.sleep(0.25)
ur.stop()
time.sleep(0.1)


