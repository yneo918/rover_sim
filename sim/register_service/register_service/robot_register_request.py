import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RegRobot
import sys

class RobotRegisterRequest(Node):
    def __init__(self):
        super().__init__('robot_register_req')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p1"),
                ('x', 0.0),
                ('y', 0.0),
                ('theta', 0.0),
            ]
        )
        self.cli = self.create_client(RegRobot, 'robot_register')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self):
        self.req = RegRobot.Request()
        self.req.reg = True
        self.req.robot_id = self.get_parameter('robot_id').value
        self.req.pose.x = self.get_parameter('x').value
        self.req.pose.y = self.get_parameter('y').value
        self.req.pose.theta = self.get_parameter('theta').value
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = RobotRegisterRequest()
    ret = node.send_request()
    if ret.success:
        node.get_logger().info(f'Registered: {node.req.robot_id}')
    else:
        node.get_logger().info(f'Failed to register: {node.req.robot_id}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
