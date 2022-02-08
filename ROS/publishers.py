from cmath import nan
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
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleOdometry

class OffboardControlPublisher(Node):

    def __init__(self):
        super().__init__('offboard_control_publisher')
        self.publisher_ = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.offboard_callback)
        self.i = 0
        self.msg = OffboardControlMode()
        self.position = False
        self.velocity = False 
        self.accelration = False 
        self.attitude = False
        self.body_rate = False

    def offboard_callback(self):
        #currently does not work
        msg = OffboardControlMode()
        msg.position = self.position
        msg.velocity = self.velocity
        msg.acceleration = self.accelration 
        msg.attitude = self.attitude
        msg.body_rate = self.body_rate
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing offboard message')

    def publish(self,timestamp, position = False, velocity = False, accelration = False, attitude = False, body_rate = False):
        msg = OffboardControlMode()
        msg.timestamp = timestamp
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = accelration
        msg.attitude = attitude
        msg.body_rate = body_rate
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

class GymNode(Node):
    #eventually time subscriber node will have to pass here by reference
    def __init__(self):
        super().__init__('gym_node')
        self.publisher_vel_ = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.publisher_com_ = self.create_publisher(OffboardControlMode, '/fmu/offboard_control_mode/in', 10)
        timer_period = 0.1  # seconds
        #just one timer to call both publishers
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # decide what to publish either 'position' or 'velocity'
        self.mode = 'None'
        #velocity commads
        self.i = 0
        self.x = nan
        self.y = nan
        self.z = nan
        self.yaw =nan
        self.vx = 0.0
        self.vy = 0.0
        self.vz =0.0
        self.yawspeed = 0.0
        # offboard command mode
        self.position = False
        self.velocity = True
        self.accelration = False 
        self.attitude = False
        self.body_rate = False
        # create timestamp subscriber
        self.subscription = self.create_subscription(
            Timesync,
            '/fmu/timesync/out',
            self.timestamp_listener_callback,
            10)
        self.drone_state_subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/vehicle_odometry/out',
            self.UAV_state_listener_callback,
            10)
        self.current_time = 0
        #initialize states
        self.uav_position = [0,0,0,0]
    
    def timer_callback(self):
        if self.mode == 'None':
            pass
        elif self.mode == 'velocity':
            self.trajectory_callback()
            self.position = False
            self.velocity = True
            self.veh_com_callback()
        elif self.mode == 'position':
            self.trajectory_callback()
            self.position = True
            self.velocity = False
            self.veh_com_callback()

    def trajectory_callback(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.current_time
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.yaw =self.yaw
        msg.vx = self.vx
        msg.vy = self.vy
        msg.vz = self.vz
        msg.yawspeed = self.yawspeed
        self.publisher_vel_.publish(msg)

    def veh_com_callback(self):
        msg = OffboardControlMode()
        msg.timestamp = self.current_time
        msg.position = self.position
        msg.velocity = self.velocity
        msg.acceleration = self.accelration 
        msg.attitude = self.attitude
        msg.body_rate = self.body_rate
        self.publisher_com_.publish(msg)

    def get_current_state(self):
        #eventual more states will be added
        state = self.uav_position
        return state
    
    def UAV_state_listener_callback(self,msg):
        #positions
        x = msg.x
        y = msg.y
        z = msg.z
        q = msg.q
        self.uav_position = [x,y,z,q]

    def timestamp_listener_callback(self, msg):
        self.current_time = msg.timestamp
        #self.get_logger().info('I heard: "%s"' % msg.timestamp)
    
#    def publish(self,timestamp, vx = 0.0, vy = 0.0, vz = 0.0, yawspeed = 0.0):
#        msg = TrajectorySetpoint()
#        msg.timestamp = timestamp
#        msg.x = nan
#        msg.y = nan
#        msg.z = nan
#        msg.yaw =nan
#        msg.vx = vx 
#        msg.vy = vy
#        msg.vz = vz
#        msg.yawspeed = yawspeed
#        self.publisher_.publish(msg)

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


