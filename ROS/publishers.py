import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand


class OffboardControlPublisher(Node):

    def __init__(self):
        super().__init__('offboard_control_publisher')
        self.publisher_ = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.offboard_callback)
        self.i = 0

    def offboard_callback(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing offboard message')

    def publish(self,timestamp):
        msg = OffboardControlMode()
        msg.timestamp = timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_.publish(msg)

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pos_callback)
        self.i = 0

    def pos_callback(self):
        msg = TrajectorySetpoint()
        msg.timestamp = 0#timestamp
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.yaw = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing position message')
    
    def publish(self,timestamp,x,y,z,yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = timestamp
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw
        self.publisher_.publish(msg)

class VehicleCommandPublisher(Node):
    def __init__(self):
        super().__init__('offboard_mode_publisher')
        self.publisher_ = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', 10)
    
    def publish(self,timestamp,command,param1 = 0.0,param2 = 0.0):
        msg = VehicleCommand() 
        msg.timestamp = timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publisher_.publish(msg)


