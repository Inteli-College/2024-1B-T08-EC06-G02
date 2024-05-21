#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys, select, tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1
STOP_DISTANCE = 0.5

msg = """
Controle o robô da Atvos!
---------------------------
Para se mover:
    ↑
←       →
    ↓

↑ : aumenta a velocidade linear (~ 0.22)
↓ : diminui a velocidade linear (~ 0.22)
← : aumenta velocidade angular (~ 2.84)
→ : diminui velocidade angular (~ 2.84)

Tecla de espaço: força a pausa

Pressione S para encerrar
"""

e = """
Comunicação falhou
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)  # Read the next two characters
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return f"currently:\tlinear vel {target_linear_vel}\t angular vel {target_angular_vel}"

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    return input

def checkLinearLimitVelocity(vel):
    return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def checkAngularLimitVelocity(vel):
    return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

class TeleopAndLidarNode(Node):
    def __init__(self):
        super().__init__('teleop_and_lidar_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE)        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile)
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.obstacle_detected = False
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(msg)
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        key = getKey(self.settings)
        if key == '\x1b[A':  # Up arrow
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == '\x1b[B':  # Down arrow
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == '\x1b[D':  # Left arrow
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == '\x1b[C':  # Right arrow
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE)
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == ' ':
            self.target_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.control_linear_vel = 0.0
            self.control_angular_vel = 0.0
            self.get_logger().info(vels(self.target_linear_vel, self.target_angular_vel))
        else:
            if key.lower() == 's':  # S key
                print("Comunicação encerrada com o robô")
                rclpy.shutdown()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                return

        if self.obstacle_detected:
            self.control_linear_vel = 0.0
            self.control_angular_vel = 0.0
        else:
            self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
            self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))

        twist = Twist()
        twist.linear.x = self.control_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_angular_vel

        self.publisher.publish(twist)

    def lidar_callback(self, msg):
        # Print the minimum range detected by the LIDAR
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Min distance: {min_distance}')

        # Check if there's something within 0.15 meters
        if min_distance < STOP_DISTANCE:
            self.get_logger().warn(f'Objeto detectado em {STOP_DISTANCE} metros! Parando o robô.')
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

def main(args=None):
    rclpy.init(args=args)
    node = TeleopAndLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()
