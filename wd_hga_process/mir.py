from rclpy.node import Node
from wd_hga_process.minimal_pub import *
from std_msgs.msg import *

import requests, json, threading, time

class MiR():
    def __init__(self, node, robot_ip='192.168.12.20'):
        self.node = node
        self.robot_ip = robot_ip
        self.prefix = '/mir'
        #self.pub_robotstate = MinimalPub(node=self.node, name=self.prefix+'/robot_state', msgsType=String, topic=self.prefix+'/robot_state')

        self.pub_robotstate = self.node.create_publisher(String, self.prefix+'/robot_state', 10)
        self.pub_btncall = self.node.create_publisher(String, self.prefix+'/btn_call', 10)
        self.bThread = True
        #self.timer = self.node.create_timer(0.02, self.timer_callback)
        #self.i = 0
        
        self.feet_ip = '0.0.0.0:8000'
        self.robotstateThreadInit()
        self.btncallThreadInit()

    def __del__(self):
        print('MiR del')
        self.bThread = False
        self.threadStatus.join()
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
                self.pub_btncall.publish(res)
                self.goto_pos(int(res.data))
            time.sleep(0.5)


    def goto_pos(self, _num):
        pos = [ 'be4da03e-c876-11ec-8696-0001299a3e90',
                'd9dc1dba-c876-11ec-8696-0001299a3e90',
                'ea86df57-c876-11ec-8696-0001299a3e90' ]
        mission_id = {"mission_id": pos[_num-1]}

        # Get Request
        host = 'http://' + self.robot_ip + '/api/v2.0.0/'

        # Format Headers
        headers = {}
        headers['Content-Type'] = 'application/json'
        headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        print(headers)

        post_mission = requests.post(host + 'mission_queue', json=mission_id, headers=headers)


