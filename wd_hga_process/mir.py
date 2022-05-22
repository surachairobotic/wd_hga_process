from rclpy.node import Node
from wd_hga_process.minimal_pub import *
from std_msgs.msg import *

import requests, json, threading, time

class MiR():
    def __init__(self, node, robot_ip='192.168.12.20'):
        self.node = node
        self.robot_ip = robot_ip
        self.prefix = '/mir'
        self.pub_robotstate = MinimalPub(node=self.node, name=self.prefix+'/robot_state', msgsType=String, topic=self.prefix+'/robot_state')

        self.pub_robotstate = self.node.create_publisher(String, self.prefix+'/robot_state', 10)
        
        #self.timer = self.node.create_timer(0.02, self.timer_callback)
        #self.i = 0
        self.robotstateThreadInit()

    def robotstateThreadInit(self):
        # Get Request
        self.host = 'http://' + self.robot_ip + '/api/v2.0.0/status'

        # Format Headers
        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        print(self.headers)
        
        self.tStatus = threading.Thread(target=self.robotstateThreadRun)
        self.tStatus.start()
        
    def robotstateThreadRun(self):
        while True:
            t = time.time()
            get_status = requests.get(self.host, headers=self.headers)
            parsed = json.loads(get_status.content)
            print(json.dumps(parsed, indent=4, sort_keys=True))
            print(time.time()-t)
            time.sleep(0.25)

