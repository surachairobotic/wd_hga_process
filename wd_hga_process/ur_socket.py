import socket, time, requests, math, copy
from ur_rtde import *
from ur5_kinematics import *

class GRIPPER():
    def __init__(self, _ip='192.168.12.20'):
        # Gripper init
        self.ip = _ip
        self.host = 'http://' + self.ip + '/api/v2.0.0/'
        # Format Headers
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='

        self.pos = ['e0f8f53c-02bd-11ed-90f4-0001299a3e90', # grip open
                    'd3ab6322-02bd-11ed-90f4-0001299a3e90', # grip close
                   ]

    def open(self):
        self.mission_id = {"mission_id": self.pos[0]}
        post_mission = requests.post(self.host + 'mission_queue', json=self.mission_id, headers=self.headers)
    def close(self):
        self.mission_id = {"mission_id": self.pos[1]}
        post_mission = requests.post(self.host + 'mission_queue', json=self.mission_id, headers=self.headers)

class UR_SOCKET():
    def __init__(self, _ur_ip='192.168.12.100', _ur_port=30002, disableButton=True):
        self.ur_rtde = UR_INFORMATION() # get current robot state
        while len(self.ur_rtde.joint_pos) == 0:
            pass
        while len(self.ur_rtde.tip_pos) == 0:
            pass
        self.ur_ip = _ur_ip
        self.ur_port = _ur_port
        self.connected = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.grip = GRIPPER()
        self.kinematics = Kinematic()
        
    def init(self):
        x = self.sock.connect((self.ur_ip, self.ur_port))
        print('connect : ', str(x))
        self.connected = True

    def __del__(self):
        print('UR_SOCKET del')
        if self.connected:
            self.sock.close()
    
    def send(self, msg):
        msg += "\n"
        self.sock.send(msg.encode('utf-8'))
    def read(self):
        data = []
        while True:
            d = self.sock.recv(1024)
            print(d)
            if (len(d) < 1):
                break
            data.append(d)
        return data
    
    def stop(self):
        msg = 'stopj(2)'
        self.send(msg)
    
    def moveLine(self, p, v=1, debug=False, err=0.0005, block=True): #v=0.1
        pose = "{},{},{},{},{},{}".format(p[0], p[1], p[2], p[3], p[4], p[5])
        msg = 'movep(p[{}],a=0.1,v={},r=0)'.format(pose, v)#a=0.1
        print(msg)
        self.send(msg)
        if debug:
            print('moveLine : debug={}'.format(debug))
        if block:
            self.blockL(p, err=err, debug=debug)

    def moveJ(self, j, v=0.1, debug=False, block=True):
        pose = "{},{},{},{},{},{}".format(j[0], j[1], j[2], j[3], j[4], j[5])
        msg = 'movej([{}],a=0.1,v={},t=0,r=0)'.format(pose, v)
        #print(msg)
        self.send(msg)
        if block:
            self.blockJ(j, debug=debug)

    def moveTool(self, p, v=0.05, block=True, exceptStop=False): #v=0.01
        pose = "{},{},{},{},{},{}".format(p[0], p[1], p[2], p[3], p[4], p[5])
        msg = 'movel(pose_trans(get_actual_tcp_pose(),p[{}]),a=0.01, v={})'.format(pose, v)#a=0.01
        #msg = 'speedl([{},{},{},0,0,0],a=0.1,t=1.0)'.format(x, y, z)
        self.send(msg)
        time.sleep(0.2)
        if block:
            self.blockP(debug=False, exceptStop=exceptStop)
        '''
        print('==============')
        print(self.ur_rtde.target_tool_pos, " : ", self.ur_rtde.target_joint)
        print('==============')
        '''

    def getFK(self, j):
        pose = "{},{},{},{},{},{}".format(j[0], j[1], j[2], j[3], j[4], j[5])
        msg = 'get_forward_kin([{}])'.format(j)
        self.send(msg)
        return self.read()
    
    def grip_open(self):
        self.grip.open()
        time.sleep(6.0)
    def grip_close(self):
        self.grip.close()
        time.sleep(6.0)

    def calError(self, actual, target):
        e = 0
        for i in range(len(actual)):
            e += math.sqrt(pow(actual[i]-target[i], 2))
        return e

    def blockP(self, err=0.0001, debug=False, exceptStop=False):
        if debug:
            print('blockP : debug={}'.format(debug))
        t = time.time()
        while time.time()-t < 15:
            pose = self.ur_rtde.tip_pos
            target = self.ur_rtde.target_tool_pos
            e = self.calError(pose, target)
            if debug:
                print("{}:{}:{}".format(pose, target, e))
            if e < err:
                break
            time.sleep(0.05)
        if not exceptStop:
            self.stop()
        time.sleep(0.1)
    def blockL(self, p, err=0.0005, debug=False):
        print('blockL : debug={}'.format(debug))
        t = time.time()
        while time.time()-t < 15:
            pose = self.ur_rtde.tip_pos
            e = self.calError(pose, p)
            if debug:
                print("{}:{}:{}".format(pose, p, e))
            if e < err:
                break
            time.sleep(0.05)
        self.stop()
        time.sleep(0.1)
    def blockJ(self, j, _e=0.001, debug=False):
        t = time.time()
        while time.time()-t < 15:
            joint_state = self.ur_rtde.joint_pos
            e = self.calError(joint_state, j)
            if debug:
                print("{}:{}:{}".format(joint_state, j, e))
            if e < _e:
                break
            time.sleep(0.05)
        self.stop()
        time.sleep(0.1)

    def moveX(self, offset, speed, debug=False):
        pos = copy.deepcopy(self.ur_rtde.tip_pos)
        pos[0] += offset
        #joint = kinematics.IK(state.actual_TCP_pose, j)
        self.moveLine(pos, speed, debug=debug)
    def moveY(self, offset, speed, debug=False):
        pos = copy.deepcopy(self.ur_rtde.tip_pos)
        pos[1] += offset
        #joint = kinematics.IK(state.actual_TCP_pose, j)
        self.moveLine(pos, speed, debug=debug)
    def moveZ(self, offset, speed, debug=False):
        pos = copy.deepcopy(self.ur_rtde.tip_pos)
        pos[2] += offset
        #joint = kinematics.IK(state.actual_TCP_pose, j)
        self.moveLine(pos, speed, debug=debug)
        
    '''
    def jointSpeed(self):
        msg = 'get_actual_joint_speeds()'
        self.send(msg)
        res = self.read()
        print(res)
    '''
