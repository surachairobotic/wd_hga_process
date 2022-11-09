import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import keyboard

import torch
import pandas as pd

import shutil
import time

class DETECTOR():
    def __init__(self, node):
        self.node = node

        self.model = torch.hub.load('/home/cmit/yolov5/', 'custom', path='/home/cmit/yolov5/best_top_TINY.pt', source='local')

        self.timer = self.node.create_timer(2.0, self.timer_callback)
        self.i = 0

        self.path = '/home/cmit/dev_ws/ham_image/'

    def timer_callback(self):
        t=time.time()
        print('start : timer_cb : {}'.format(self.i))
        
        #cv2.imwrite(self.path + 'rgb_detect.png', self.rgb_frame)
        src = self.path + 'rgb_detect.png'
        dest = self.path + 'rgb_detect2.png'
        shutil.copy2(src, dest)
        print(dest)
        print(type(dest))
        try:
            out = cv2.imread(dest)
            scale = 1.0
            w = int(out.shape[1] * scale)
            h = int(out.shape[0] * scale)
            dim = (w, h)
            print(dim)
            
            resized = cv2.resize(out, dim)
        except Exception as e:
            print(e)
            return False

        print(type(out))
        if out is None:
            return False
        else:
            try:
                out2 = self.model(resized)
                #out2.show()

                if len(out2.xyxy[0]) != 0:
                    bbox= out2.xyxy[0]
                    x1=int((bbox.data[0][0]).item())
                    y1=int((bbox.data[0][1]).item())
                    x2=int((bbox.data[0][2]).item())
                    y2=int((bbox.data[0][3]).item())
                    print([x1,y1,x2,y2])
                    #############################
                    center_x=x1+(x2-x1)/2
                    center_y=y1+(y2-y1)/2
                    ideal_start_point =(344,99)
                    ideal_end_point =(747,434)
                    color_ideal=(255,0,0)
                    resized=cv2.rectangle(resized, ideal_start_point,  ideal_end_point,color_ideal, 5)
                    #############################
                    area_detect=(x2-x1)*(y2-y1)
                    area_ideal=(ideal_end_point[0]-ideal_start_point[0])*(ideal_end_point[1]-ideal_start_point[1])
                    if (abs(area_ideal-area_detect)/area_ideal)*100 >10 : 
                         if area_detect>area_ideal:
                            print("too close")
                         if area_detect<area_ideal:
                            print("too far")   
                    if (abs(area_ideal-area_detect)/area_ideal)*100 <10:
                        print("OK")
                        if abs(x1-ideal_start_point[0])>20:
                            print('move left')
                        elif abs(x2-ideal_end_point[0])>20:
                            print('move right')
                        if abs(y1-ideal_start_point[1])>20:
                            print('move up')
                        elif abs(y2-ideal_start_point[1])>20:
                            print('move down')
                    #############################
                    start_point = (x1, y1)
                    end_point = (x2, y2)
                    color = (255, 255, 0)
                    thickness = 2
                    resized = cv2.rectangle(resized, start_point, end_point, color, thickness)
                    print(resized.shape)
            except Exception as e:
                print(e)

        try:
            cv2.imshow("detect", resized)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

        #out2 = self.model(self.rgb_frame)
        #cv2.imshow("Yolo", out2)
        #cv2.waitKey(1)

        #cv2.imwrite(self.path + 'rgb_{}.png'.format(self.i), self.rgb_frame)
        #cv2.imwrite(self.path + 'grey_{}.png'.format(self.i), self.depth_frame)
        
        ##np.savetxt(self.path + 'rgb_{}.csv'.format(self.i), self.rgb_frame, delimiter=",")
        #np.savetxt(self.path + 'grey_{}.csv'.format(self.i), self.depth_data, delimiter=",")
        #print(out)

        self.i+=1
        dt=time.time()-t
        print('Processing time : {}'.format(dt))

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wd_detector')
    
    img = DETECTOR(node)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
