import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.stop_twist = Twist()  # zero velocities

    def timer_callback(self):
        if emergency_condition():
            self.publisher_.publish(self.stop_twist)
            self.get_logger().info('Emergency stop activated!')

def main(args=None):
    rclpy.init(args=args)
    emergency_stop_node = EmergencyStopNode()
    rclpy.spin(emergency_stop_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
