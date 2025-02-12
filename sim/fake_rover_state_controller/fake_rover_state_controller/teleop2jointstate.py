import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

UPDATE_RATE = 0.05
ANGLE_UPDATE_RATE = 0.02
JOINT_NAMES = [
    'p1_w_to_x', 'p1_x_to_y', 'p1_y_to_t',
    'p2_w_to_x', 'p2_x_to_y', 'p2_y_to_t',
    'p3_w_to_x', 'p3_x_to_y', 'p3_y_to_t'
]

class JointStates(Node):

    def __init__(self):
        super().__init__('teleop_to_jointstate')
        self._subscriptions = [
            self.create_subscription(Twist, f'/sim/p{i}/cmd_vel', lambda msg, i=i: self.teleop_callback(msg, i), 10)
            for i in range(1, 4)
        ]

        self.positions = {
            'p1': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'p2': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'p3': {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        }

        self.joint_state_publisher_ = self.create_publisher(JointState, 'joint_states', 10)

    def update_position(self, index, lx, az):
        key = f'p{index}'
        self.positions[key]['theta'] -= ANGLE_UPDATE_RATE * az
        self.positions[key]['x'] += UPDATE_RATE * lx * math.cos(self.positions[key]['theta'])
        self.positions[key]['y'] += UPDATE_RATE * lx * math.sin(self.positions[key]['theta'])

    def publish_joint_states(self):
        jointstates_msg = JointState()
        t = time.time()
        jointstates_msg.header.stamp.sec = int(t // 1)
        jointstates_msg.header.stamp.nanosec = int((t % 1) * 1e9)
        jointstates_msg.name = JOINT_NAMES
        jointstates_msg.position = [
            self.positions['p1']['x'], self.positions['p1']['y'], self.positions['p1']['theta'],
            self.positions['p2']['x'], self.positions['p2']['y'], self.positions['p2']['theta'],
            self.positions['p3']['x'], self.positions['p3']['y'], self.positions['p3']['theta']
        ]
        self.joint_state_publisher_.publish(jointstates_msg)

    def teleop_callback(self, msg, index):
        self.update_position(index, msg.linear.x, msg.angular.z)
        self.publish_joint_states()


def main(args=None):
    rclpy.init(args=args)

    teleop_sub = JointStates()

    rclpy.spin(teleop_sub)

    teleop_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()