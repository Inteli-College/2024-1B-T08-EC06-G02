# !/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time
from inquirer import prompt

class TurtleController:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('turtle_controller')
        self.publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def move_turtle(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    controller = TurtleController()

    print("Controle da Tartaruga Turtlesim")

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

        answer = prompt(questions, raise_keyboard_interrupt=True)  

        command = answer['command'].lower()

        if command == 'sair':
            break
        elif command == 'frente':
            controller.move_turtle(2.0, 0.0)
        elif command == 'trás':
            controller.move_turtle(-2.0, 0.0)
        elif command == 'esquerda':
            controller.move_turtle(0.0, 1.56)
        elif command == 'direita':
            controller.move_turtle(0.0, -1.56)

    controller.destroy()

if __name__ == '__main__':
    main()