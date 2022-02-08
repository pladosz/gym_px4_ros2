import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class ResetWorldServ(Node):
    def __init__(self):
        # Create a client    
        super().__init__('reset_world_serv')
        self.reset_world_cli = self.create_client(Empty, 'reset_world')
        self.reset_sim_cli = self.create_client(Empty, 'reset_simulation')
        # Check if the a service is available  
        while not self.reset_world_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.reset_sim_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_world = Empty.Request()
        self.req_sim = Empty.Request()
    
    def send_request(self):
        self.reset_world_cli.call_async(self.req_world)
        #self.reset_sim_cli.call_async(self.req_sim)
 