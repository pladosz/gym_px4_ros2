import rclpy
from rclpy.node import Node

from px4_msgs.msg import Timesync

class TimesyncSubscriber(Node):
    def __init__(self):
        super().__init__('timesync_subscriber')
        self.subscription = self.create_subscription(
            Timesync,
            '/fmu/timesync/out',
            self.listener_callback,
            10)
        self.current_time = 0

    def listener_callback(self, msg):
        self.current_time = msg.timestamp
        #self.get_logger().info('I heard: "%s"' % msg.timestamp)