import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class DynamicFrameBroadcaster(Node):

   def __init__(self):
      super().__init__('dynamic_frame_tf2_broadcaster')
      self.br = TransformBroadcaster(self)
      self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

      #self.sub_btn_call = self.node.create_subscription(PointCloud2, '/camera/depth/color/points', self.cb_pc2, 1)

   def broadcast_timer_callback(self):
      seconds, _ = self.get_clock().now().seconds_nanoseconds()
      x = seconds * math.pi

      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = 'odom'
      t.child_frame_id = 'camera_link'
      t.transform.translation.x = 1 * math.sin(x)
      t.transform.translation.y = 1 * math.cos(x)
      t.transform.translation.z = 0.0
      t.transform.rotation.x = 0.0
      t.transform.rotation.y = 0.0
      t.transform.rotation.z = 0.0
      t.transform.rotation.w = 1.0

      self.br.sendTransform(t)


def main():
   rclpy.init()
   node = DynamicFrameBroadcaster()
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()
