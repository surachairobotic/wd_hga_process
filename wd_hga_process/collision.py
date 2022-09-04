from rclpy.node import Node
from wd_hga_process.minimal_pub import *
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
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
        self.t = time.time()
        fps = 1.0/dt
        print('FPS : ', fps)
        if 1:
            pc = pc2.read_points_list(msg, skip_nans=True)
            #print(pc)
            #print(type(pc))
            #print(pc.shape)
            #print(len(pc))
            #print(next(pc))
            mmin = pc[0][2]
            #mmax = [pc[0][0], pc[0][1], pc[0][2], pc[0][3]]
            for i in range(len(pc)):
                pc[i][0] += 1
        else:
            print(msg.header)
            print([msg.height, msg.width])
            print(msg.fields)
            print([msg.point_step, msg.point_step])
            print(len(msg.data))
            '''
            480 x 848 = 407,040
            
            '''
        
