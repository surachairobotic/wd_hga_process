import rclpy
from sensor_msgs.msg import Image

class IMAGE():
    def __init__(self, node):
        self.node = node

        #self.pub_robotstate = self.node.create_publisher(String, self.prefix+'/robot_state', 10)
        self.pub_img = self.node.create_publisher(Image, '/img/output', 10)
        self.sub_btn_call = self.node.create_subscription(Image, '/camera/depth/image_rect_raw', self.cb_img, 1)

    def cb_img(self, msg):
        #print(msg.header.frame_id)
        msg.header.frame_id = 'camera_color_optical_frame'
        self.pub_img.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('change_image_frame')
    
    img = IMAGE(node)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
