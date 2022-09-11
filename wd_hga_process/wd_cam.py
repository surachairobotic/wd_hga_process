import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import keyboard

class IMAGE():
    def __init__(self, node):
        self.node = node

        self.sub_rgb = self.node.create_subscription(Image, '/camera/color/image_raw', self.cb_img, 1)
        self.sub_depth = self.node.create_subscription(Image, '/camera/depth/image_rect_raw', self.cb_depth, 1)
        
        self.timer = self.node.create_timer(2.0, self.timer_callback)
        self.i = 0

        self.sub_rgb
        self.sub_depth
        
        self.br = CvBridge()
        
        self.rgb_frame = ''
        self.depth_frame = ''
        self.depth_data = ''
        self.path = '/home/cmit/dev_ws/ham_image/'

    def cb_img(self, msg):
        #print('RGB : ' + msg.header.frame_id)
        
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        #print(type(current_frame))

        self.rgb_frame = current_frame
        
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
        print('timer_cb : {}'.format(self.i))
        
        cv2.imwrite(self.path + 'rgb_{}.png'.format(self.i), self.rgb_frame)
        cv2.imwrite(self.path + 'grey_{}.png'.format(self.i), self.depth_frame)
        
        #np.savetxt(self.path + 'rgb_{}.csv'.format(self.i), self.rgb_frame, delimiter=",")
        np.savetxt(self.path + 'grey_{}.csv'.format(self.i), self.depth_data, delimiter=",")
        self.i+=1

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
