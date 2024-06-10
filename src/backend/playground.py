import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, ttyclass TeleopTurtle(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()    def send_cmd_vel(self, linear_vel, angular_vel):
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = angular_vel
        self.publisher_.publish(self.twist_msg)
        print("Linear Vel: {}, Angular Vel: {}".format(linear_vel, angular_vel))def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return keydef main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtle()    while True:
        key = get_key()
        if key == '\x03': # CTRL-C
            break
        elif key == 'w':
            node.send_cmd_vel(0.2, 0.0)  # Move forward
            while get_key() == 'w':
                pass  # Espera até que a tecla seja solta
            node.send_cmd_vel(0.0, 0.0)  # Parar quando a tecla é solta
        elif key == 'x':
            node.send_cmd_vel(-0.2, 0.0)  # Move backward
            while get_key() == 'x':
                pass  # Espera até que a tecla seja solta
            node.send_cmd_vel(0.0, 0.0)  # Parar quando a tecla é solta
        elif key == 'd':
            node.send_cmd_vel(0.0, -0.5)  # Turn right
            while get_key() == 'd':
                pass  # Espera até que a tecla seja solta
            node.send_cmd_vel(0.0, 0.0)  # Parar quando a tecla é solta
        elif key == 'a':
            node.send_cmd_vel(0.0, 0.5)  # Turn left
            while get_key() == 'a':
                pass  # Espera até que a tecla seja solta
            node.send_cmd_vel(0.0, 0.0)  # Parar quando a tecla é solta    node.send_cmd_vel(0.0, 0.0)  # Parar o robô antes de encerrar
    rclpy.shutdown()if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()