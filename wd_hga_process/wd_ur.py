import rclpy
from wd_hga_process.ur import *

def main(args=None):
    print('UR Node')
    rclpy.init(args=args)
    node = rclpy.create_node('wd_ur')
    
    robot = UR(node)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('before del ur')
    del robot
    print('after del ur')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
