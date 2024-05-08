#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def move_turtle(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

def desenho(controller):
    # Movimentos para desenhar um quadrado
    movements = [
        (0.0, 0.0),    # Iniciar
        (2.0, 0.0),    # Lado 1
        (0.0, 1.56),    # Lado 2
        (2.0, 0.0),   # Lado 3
        (0.0, 1.56),   # Lado 4
        (2.0, 0.0),     # Voltar para o início
        (0.0, 1.56),     # Voltar para o início
        (2.0, 0.0),
        (0.0, 1.56)     # Voltar para o início
    ]

    # Executar os movimentos
    for linear, angular in movements:
        controller.move_turtle(linear, angular)
        time.sleep(1)  # Pausa de 1 segundo entre os movimentos


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()
    print("Desenhando um quadrado...")
    set_pen_color(255,0,255)
    desenho(controller)

    # Criação de uma nova tartaruga
    node = rclpy.create_node('spawn_turtle_client')
    client = node.create_client(Spawn, '/spawn')

    request = Spawn.Request()
    request.x = 5.0
    request.y = 5.0
    request.theta = 0.0
    request.name = 'my_turtle'

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        print('Turtaruga nova com sucesso', future.result().name)
    else:
        print('Failed to spawn turtle')

    time.sleep(2)

    # Matar a tartaruga
    kill_request = Kill.Request()
    kill_request.name = 'my_turtle'
    kill_client = node.create_client(Kill, 'kill')

    future_kill = kill_client.call_async(kill_request)
    rclpy.spin_until_future_complete(node, future_kill)

    if future_kill.result() is not None:
        print(1)
        # print('Turtle killed successfully:', future_kill.result().name)
    else:
        print('Failed to kill turtle')

    controller.destroy_node()
    rclpy.shutdown()

def set_pen_color(r, g, b):
    node = rclpy.create_node('set_pen_color')
    set_pen_client = node.create_client(SetPen, '/turtle1/set_pen')

    while not set_pen_client.wait_for_service(timeout_sec=1.0):
        print('Service not available, waiting again...')

    request = SetPen.Request()
    request.r = r
    request.g = g
    request.b = b
    request.width = 8  # Mantém a largura da linha inalterada
    request.off = 0    # Mantém a caneta ligada

    future = set_pen_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

if __name__ == '__main__':
    main()
