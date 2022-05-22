import rclpy
from wd_hga_process.mir import *

def main(args=None):
    print('Hi from wd_hga_process.')
    rclpy.init(args=args)
    node = rclpy.create_node('wd_mir')
    
    robot = MiR(node)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
