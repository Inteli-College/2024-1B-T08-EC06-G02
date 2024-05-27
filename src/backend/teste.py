# # !/usr/bin/env python3
# import rclpy
# from geometry_msgs.msg import Twist
# import time
# from inquirer import prompt

# class TurtleController:
#     def __init__(self):
#         rclpy.init()
#         self.node = rclpy.create_node('turtle_controller')
#         self.publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

#     def move_turtle(self, linear, angular):
#         twist = Twist()
#         twist.linear.x = linear
#         twist.angular.z = angular
#         self.publisher.publish(twist)

#     def destroy(self):
#         self.node.destroy_node()
#         rclpy.shutdown()

# def main():
#     controller = TurtleController()

#     print("Controle da Tartaruga Turtlesim")

#     while True:
#         questions = [
#             {
#                 'type': 'list',
#                 'name': 'command',
#                 'message': 'Selecione uma ação:',
#                 'choices': [
#                     'Frente',
#                     'Trás',
#                     'Esquerda',
#                     'Direita',
#                     'Sair'
#                 ]
#             }
#         ]

#         answer = prompt(questions, raise_keyboard_interrupt=True)  

#         command = answer['command'].lower()

#         if command == 'sair':
#             break
#         elif command == 'frente':
#             controller.move_turtle(2.0, 0.0)
#         elif command == 'trás':
#             controller.move_turtle(-2.0, 0.0)
#         elif command == 'esquerda':
#             controller.move_turtle(0.0, 1.56)
#         elif command == 'direita':
#             controller.move_turtle(0.0, -1.56)

#     controller.destroy()

# if __name__ == '__main__':
#     main()
    
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from inquirer import prompt

class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()

    def send_cmd_vel(self, linear_vel, angular_vel):
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = angular_vel
        self.publisher_.publish(self.twist_msg)
        print("Linear Vel: {}, Angular Vel: {}".format(linear_vel, angular_vel))

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtle()

    print("Controle do TurtleBot3")

    while True:
        questions = [
            {
                'type': 'list',
                'name': 'command',
                'message': 'Selecione uma ação:',
                'choices': [
                    'Frente',
                    'Trás',
                    'Esquerda',
                    'Direita',
                    'Sair'
                ]
            }
        ]

        try:
            answer = prompt(questions)  
        except KeyboardInterrupt:
            break

        command = answer['command'].lower()

        if command == 'sair':
            break
        elif command == 'frente':
            node.send_cmd_vel(0.2, 0.0)
        elif command == 'trás':
            node.send_cmd_vel(-0.2, 0.0)
        elif command == 'esquerda':
            node.send_cmd_vel(0.0, 0.5)
        elif command == 'direita':
            node.send_cmd_vel(0.0, -0.5)

    node.send_cmd_vel(0.0, 0.0)  
    rclpy.shutdown()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()
