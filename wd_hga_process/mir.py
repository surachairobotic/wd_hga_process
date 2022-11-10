from rclpy.node import Node
from wd_hga_process.minimal_pub import *
from std_msgs.msg import *

import requests, json, threading, time

class MiR():
    def __init__(self, node, robot_ip='192.168.12.20', feet_ip='192.168.12.252:8000', disableButton=False):
        self.node = node
        self.robot_ip = robot_ip
        self.prefix = '/mir'
        #self.pub_robotstate = MinimalPub(node=self.node, name=self.prefix+'/robot_state', msgsType=String, topic=self.prefix+'/robot_state')

        self.pub_robotstate = self.node.create_publisher(String, self.prefix+'/robot_state', 10)
        self.pub_callInfo = self.node.create_publisher(String, self.prefix+'/call_info', 10)
        self.sub_btn_call = self.node.create_subscription(String, self.prefix+'/btn_call', self.cb_btn, 1)
        self.bThread = True
        #self.timer = self.node.create_timer(0.02, self.timer_callback)
        #self.i = 0
        
        self.feet_ip = feet_ip
        self.robotstateThreadInit()
        
        self.disableButton = disableButton
        if not self.disableButton:
            self.btncallThreadInit()

    def __del__(self):
        print('MiR del')
        self.bThread = False
        self.threadStatus.join()
        if not self.disableButton:
            self.threadBtnCall.join()
    
    def robotstateThreadInit(self):
        # Get Request
        self.host = 'http://' + self.robot_ip + '/api/v2.0.0/status'

        # Format Headers
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        print(self.headers)
        
        self.threadStatus = threading.Thread(target=self.robotstateThreadRun)
        self.threadStatus.start()
        
    def robotstateThreadRun(self):
        while self.bThread:
            #t = time.time()
            get_status = requests.get(self.host, headers=self.headers)
            parsed = json.loads(get_status.content)
            #print(type(parsed))
            #print(type(json.dumps(parsed, indent=4, sort_keys=True)))
            #print(parsed['mission_text'])
            res = String()
            res.data = parsed['mission_text']
            self.pub_robotstate.publish(res)
            if not self.disableButton:
                self.postRobotstate(res.data)
            #print(time.time()-t)
            time.sleep(0.5)

    def postRobotstate(self, _status):
        #print(_status)
        stateJson = {"state" : _status}
        _host = 'http://' + self.feet_ip + '/set_robotstate'
        requests.post(_host, json=stateJson)
        
    def btncallThreadInit(self):
        self.threadBtnCall = threading.Thread(target=self.btncallThreadRun)
        self.threadBtnCall.start()

    def btncallThreadRun(self):
        while self.bThread:
            _host = 'http://' + self.feet_ip + '/btn_call'
            get_status = requests.get(_host, headers=self.headers)
            #print(get_status)
            parsed = json.loads(get_status.content)            
            res = String()
            res.data = str(parsed['call_id'])
            #print('res.data : ' + res.data)
            if int(res.data) != -1:
                self.pub_callInfo.publish(res)
                self.goto_pos(int(res.data))
            time.sleep(0.5)

    def cb_btn(self, msg):
        print('msg: ', msg.data)
        pnt = int(msg.data)
        if pnt > 0 and pnt <= 6:
            self.goto_pos(pnt)

    def goto_pos(self, _num):
        pos = [ '1875eb4e-3fc3-11ed-84c3-0001299a3e90', # ku_goPos_1
                '40b37c04-3fc3-11ed-84c3-0001299a3e90', # ku_goPos_2
                'ea86df57-c876-11ec-8696-0001299a3e90', # 
                '80ac3e98-0e63-11ed-843d-0001299a3e90',
                'bdffb631-15bb-11ed-ae82-0001299a3e90',
                'd74f2c17-15bb-11ed-ae82-0001299a3e90',
 ]
        mission_id = {"mission_id": pos[_num-1]}

        # Get Request
        host = 'http://' + self.robot_ip + '/api/v2.0.0/'

        # Format Headers
        headers = {}
        headers['Content-Type'] = 'application/json'
        headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        print(headers)

        post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)


