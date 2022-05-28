import rclpy
from wd_hga_process.mir import *

def main(args=None):
    print('Hi from wd_hga_process. 1')
    rclpy.init(args=args)
    print('Hi from wd_hga_process. 2')
    node = rclpy.create_node('wd_mir')
    print('Hi from wd_hga_process. 3')
    
    robot = MiR(node)
    print('Hi from wd_hga_process. 4')

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('before del robot')
    del robot
    print('after del robot')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
