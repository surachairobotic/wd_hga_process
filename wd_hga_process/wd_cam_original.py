import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import keyboard

import torch
import pandas as pd
import copy, time

import pyrealsense2 as rs

class IMAGE():
    def __init__(self, node):
        self.node = node

        self.model = torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_jig_TINY.pt', source='local')

        self.sub_rgb = self.node.create_subscription(Image, '/camera/color/image_raw', self.cb_img, 10)
        self.sub_depth = self.node.create_subscription(Image, '/camera/depth/image_rect_raw', self.cb_depth, 10)
        
        self.timer = self.node.create_timer(0.25, self.timer_callback)
        self.i = 0
        self.rgb_i = 0

        self.sub_rgb
        self.sub_depth
        
        self.br = CvBridge()
        
        self.rgb_frame = ''
        self.depth_frame = ''
        self.depth_data = ''
        self.path = '/home/cmit/dev_ws/ham_image/'


    def cb_img(self, msg):
        #print('RGB[{}] : '.format(self.rgb_i) + msg.header.frame_id)
        self.rgb_i=(self.rgb_i+1)%1000
        
        # Convert ROS Image message to OpenCV image
        #ori_frame = self.br.imgmsg_to_cv2(msg)
        #print('RGB1')
        current_frame = self.br.imgmsg_to_cv2(msg)
        #print('RGB2')
        #print(type(current_frame))

        self.rgb_frame = current_frame

        #cv2.imwrite(self.path + 'rgb_detect.png', self.rgb_frame)
        #out = cv2.imread(self.path + 'rgb_detect.png')

        #print('RGB3')

        '''
        cv2.imwrite(self.path + 'rgb_detect.png', self.rgb_frame)
        out = cv2.imread(self.path + 'rgb_detect.png')
        out2 = self.model(out)
        print('RGB4')
        #print(type(out2))
        if len(out2.xyxy[0]) != 0:
            print('RGB5')
            bbox= out2.xyxy[0]
            x1=int((bbox.data[0][0]).item())
            y1=int((bbox.data[0][1]).item())
            x2=int((bbox.data[0][2]).item())
            y2=int((bbox.data[0][3]).item())
            print([x1,y1,x2,y2])
            start_point = (x1, y1)
            end_point = (x2, y2)
            color = (255, 255, 0)
            thickness = 2
            print('RGB6')
            #ori_frame = cv2.rectangle(ori_frame, start_point, end_point, color, thickness)
        '''
        
        #print('RGB')
        # Display image
        #cv2.imshow("rgb", current_frame)
        #cv2.waitKey(1)
        
    def cb_depth(self, msg):
        #print('DEPTH : ' + msg.header.frame_id)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        self.depth_data = current_frame

        #print(type(current_frame))
        #print(current_frame.shape)

        #print(type(current_frame[0][0]))

        depth_array = np.array(current_frame)
        center_idx = np.array(depth_array.shape) / 2
        center_idx = center_idx.astype(np.int64)
        #center_idx[0] = int(center_idx[0])
        #center_idx[1] = int(center_idx[1])
        #print(type(center_idx))
        #print(center_idx)
        #print(type(depth_array.shape))
        #print ('center depth:', depth_array[center_idx[0], center_idx[1]])
        #print(current_frame[center_idx[0], center_idx[1]])

        self.depth_frame = current_frame

        # Display image
        #cv2.imshow("depth", current_frame)
        #cv2.waitKey(1)
        
    def timer_callback(self):
        t=time.time()
        print('timer_cb : {}'.format(self.i))
        
        #cv2.imwrite(self.path + 'rgb_detect.png', self.rgb_frame)
        #out = cv2.imread(self.path + 'rgb_detect.png')

        #model = torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_top.pt', source='local')
        #out2 = model(out)
        #out2.show()

        '''
        rgb = copy.deepcopy(self.rgb_frame)
        depth = copy.deepcopy(self.depth_frame)
        
        scale = 1.0
        w = int(depth.shape[1] * scale)
        h = int(depth.shape[0] * scale)
        dim = (w, h)
        #print(dim)
        
        resized = rgb
        #resized = cv2.resize(rgb, dim)

        print(type(depth))
        print(depth.shape)
        
        lPix = []
        for i in range(h):
            for j in range(w):
                pix = depth.item(i,j)
                if pix != 0:
                    lPix.append(pix)
                    if pix > 2000:
                        resized[i,j] = np.array([0,0,0])
        
        print(type(rgb[0,0]))
        print(rgb[0,0])
        rgb[0,0] = np.array([0,0,0])
        print(type(rgb[0,0]))
        print(rgb[0,0])
        
        print([min(lPix), max(lPix)])
        
        cv2.imshow("rgb", resized)
        cv2.imshow("depth", depth)
        cv2.waitKey(1)
        '''

        rgb = copy.deepcopy(self.rgb_frame)
        out2 = self.model(rgb)

        if len(out2.xyxy[0]) != 0:
            bbox= out2.xyxy[0]
            x1=int((bbox.data[0][0]).item())
            y1=int((bbox.data[0][1]).item())
            x2=int((bbox.data[0][2]).item())
            y2=int((bbox.data[0][3]).item())
            #print([x1,y1,x2,y2])
            start_point = (x1, y1)
            end_point = (x2, y2)
            color = (255, 255, 0)
            thickness = 2
            rgb = cv2.rectangle(rgb, start_point, end_point, color, thickness)
            #print(rgb.shape)

        cv2.imshow("Yolo", rgb)
        cv2.waitKey(1)

        #cv2.imwrite(self.path + 'rgb_{}.png'.format(self.i), self.rgb_frame)
        #cv2.imwrite(self.path + 'grey_{}.png'.format(self.i), self.depth_frame)
        
        ##np.savetxt(self.path + 'rgb_{}.csv'.format(self.i), self.rgb_frame, delimiter=",")
        #np.savetxt(self.path + 'grey_{}.csv'.format(self.i), self.depth_data, delimiter=",")

        #cv2.imshow("detect", out)
        #cv2.waitKey(1)

        self.i=(self.i+1)%1000
        dt=time.time()-t
        print('Processing time : {}'.format(dt))

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wd_cam')
    
    img = IMAGE(node)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
