import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import typer
import inquirer

app = typer.Typer()

class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()

    def send_cmd_vel(self, linear_vel, angular_vel):
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = angular_vel
        self.publisher_.publish(self.twist_msg)
        print(f"Linear Vel: {linear_vel}, Angular Vel: {angular_vel}")

@app.command()
def control():
    rclpy.init()
    node = TeleopTurtle()
    print("Controle do TurtleBot3")
    questions = [inquirer.List(
        name='command',
        message='Selecione uma ação:',
        choices=['Frente', 'Trás', 'Esquerda', 'Direita', 'Emergência (Parar Funcionamento)', 'Sair'])]
    try:
        while True:
            command = inquirer.prompt(questions)['command']
            match command:
                case 'Sair':
                    break
                case 'Frente':
                    node.send_cmd_vel(0.2, 0.0) 
                case 'Trás':
                    node.send_cmd_vel(-0.2, 0.0)
                case 'Esquerda':
                    node.send_cmd_vel(0.0, 0.5)
                case 'Direita':
                    node.send_cmd_vel(0.0, -0.5)
                case 'Emergência (Parar Funcionamento)':
                    node.send_cmd_vel(0.0, 0.0)
    except Exception as e:
        print('Interface Fechada a Força, Parando movimentação do robô')
        node.send_cmd_vel(0.0, 0.0) 
    finally:
        node.send_cmd_vel(0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    app()
