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
        self.bThread = True
        #self.timer = self.node.create_timer(0.02, self.timer_callback)
        #self.i = 0
        
        self.feet_ip = '0.0.0.0:8000/set_robotstate'
        self.robotstateThreadInit()

    def __del__(self):
        print('MiR del')
        self.bThread = False
        self.threadStatus.join()
    
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
            t = time.time()
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
            time.sleep(0.25)

    def postRobotstate(self, _status):
        stateJson = {"state" : _status}
        requests.post(self.feet_ip, data=stateJson)
        



