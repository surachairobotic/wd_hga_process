import rclpy
from wd_hga_process.collision import *

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wd_collision')
    
    col = Collision(node)
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
