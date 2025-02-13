import rclpy
from rclpy.node import Node
from robot_register.srv import RegRobot

class RobotRegisterServer(Node):
    def __init__(self):
        super().__init__('robot_register')
        self.srv = self.create_service(RegRobot, 'robot_register', self.robot_register_callback)
        self.get_logger().info('AddTwoInts Server is ready.')

        self.robot = {}
        #self.create_publisher(RegRobot, 'robot_register', 10)

    def robot_register_callback(self, request, response):        
        if request.reg:
            robot_id = request.robot_id
            if robot_id in self.robot:
                response.success = False
                self.get_logger().info(f'Received request: Register / {request.robot_id} -> {response.success}')
                return response
            
            self.robot.update({robot_id: {'x': request.x_pos, 'y': request.y_pos, 'theta': request.t_pos}})
            response.success = True
            self.get_logger().info(f'Received request: Register / {request.robot_id} -> {response.success}')
            return response
        else:
            if request.robot_id not in self.robot:
                response.success = False
                self.get_logger().info(f'Received request: Remove / {request.robot_id} -> {response.success}')
                return response
            
            self.robot.pop(request.robot_id)
            response.success = True
            self.get_logger().info(f'Received request: Remove / {request.robot_id} -> {response.success}')
            return response

def main():
    rclpy.init()
    node = RobotRegisterServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
