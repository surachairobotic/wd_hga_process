from rclpy.node import Node
from std_msgs.msg import *

class MinimalPub():
    def __init__(self, node, name='MinimalPub', msgsType=String, topic='miniPub'):
        self.node = node
        self.name = name
        self.msgsType = msgsType
        self.topic = topic
        self.publisher_ = self.node.create_publisher(self.msgsType, self.topic, 10)

    def start(self, cb_func):
        self.timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def publish(self, msg):
        self.publisher_.publish(msg)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World 5: %d' % self.i
        self.publisher_.publish(msg)
        self.node.get_logger().info('%s: "%s"' % (self.name, msg.data))
        self.i += 1
