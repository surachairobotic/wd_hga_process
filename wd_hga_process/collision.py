from rclpy.node import Node
from wd_hga_process.minimal_pub import *
from sensor_msgs.msg import PointCloud2
import time

class Collision():
    def __init__(self, node):
        self.node = node
        self.prefix = '/mir'

        self.sub_btn_call = self.node.create_subscription(PointCloud2, '/camera/depth/color/points', self.cb_pc2, 1)
        self.t = time.time()

    def __del__(self):
        print('collision del')

    def cb_pc2(self, msg):
        dt = time.time()-self.t
        fps = 1.0/dt
        print('FPS : ', fps)
        self.t = time.time()
        
