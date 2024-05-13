---
sidebar_position : 1
---

# Documentação da Sprint 2

A seguir, este documento descreverá as implementações do grupo Repipe na Sprint 2. O foco ao longo dessas duas semanas foram, principalmente, a movimentação do Robô Turtlebot. Logo, falaremos sobre o setup do robô, a criação de uma interface navegável para interagir com o robô e as comunicações necessárias para isso acontecer. 

# Setup do Robô 

Este é um passo importanto do projeto, pois realiza as configurações iniciais do Turtlebot, robô utilizado neste projeto. O Turtlebot é um robô que possui um Raspberry Pi, um módulo driver e um sensor LiDAR. Com isso, tivemos que instalar um sistema operacional no cartão sd inserido para configurá-lo e instalar as dependências necessárias. Este foi o fluxo realizado nessa etapa:

* Conectamos a Raspberry Pi ao monitor;
* Instalamos a Raspberry Pi Imager;
* Instalamos o sistema operacional Ubuntu 22.04 LTS no micro cartão SD;
* Conectamos ao Raspberry Pi utilizando ssh para uma solução headless;
* Instalamos o ROS Humble;
* Instalamos os pacotes do Turtlebot3;
* Compilamos o pacote do LIDAR LDS-02;
* Fizemos o setup do OpenCR;

# Interface navegável 

Depois de ter instalado e configurado os pacotes do Turtlebot3, precisamos criar um método efetivo para controlar o robô. Para isso, nesta seção iremos abordar a comunicação com o robô e como faremos para isso chegar com uma boa usabilidade ao usuário final.

## Comunicação com o Robô 
Para controlar o robô, desenvolvemos um script em Python destinado a controlar o Turtlebot3 usando ROS 2 (Robot Operating System), especificamente para teleoperação. 

Os robôs 

Aqui está uma explicação detalhada de cada parte do código:

### Dependências 
Primeiro, este script precisou das seguintes bibliotecas Python, que são parte do ecossistema ROS 2:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
```

* rclpy: Módulo principal para interagir com ROS 2 em Python.
* geometry_msgs.msg: Pacote que contém mensagens para formas geométricas comuns e posições.
* Os módulos sys, select, termios, tty são utilizados para manipulação de entrada de teclado, mas não são demonstrados neste trecho específico do código.

### Classe TeleopTurtle
Depois, criamos uma classe `TeleopTurtle`, que é um nó ROS 2 projetado para facilitar a teleoperação do Turtlebot3. Sua implementação:

```python
class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()
```
* O publisher é configurado para publicar mensagens no tópico `cmd_vel`, que o Turtlebot3 escuta para receber comandos de movimento.

### Método send_cmd_vel
O método send_cmd_vel é usado para definir e enviar os comandos de velocidade para o robô. Como vamos fazer isso?

```python
    def send_cmd_vel(self, linear_vel, angular_vel):
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = angular_vel
        self.publisher_.publish(self.twist_msg)
        print("Linear Vel: {}, Angular Vel: {}".format(linear_vel, angular_vel))
```
* linear_vel e angular_vel são argumentos que determinam as velocidades linear e angular do robô, respectivamente.
* As velocidades são aplicadas ao objeto twist_msg e então publicadas no tópico cmd_vel.
* As velocidades atualmente enviadas são impressas no console para confirmação e debug.

### Enviando movimentos para o robô 
Certo. Depois de estabelecermos a estrutura necessária pra movimentar o nosso turtlebot, como iremos controlá-lo? Nesta parte da implementação, configuramos pra que o usuário em controle possa apertar teclas como `w, x, d, a` e consiga operar o robô. 

#### Método get_key
Este método serve para capturar uma única tecla pressionada pelo usuário diretamente do terminal. 
```python 
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
```
#### Método principal dos movimentos
```python
def main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtle()

    while True:
        key = get_key()
        if key == '\x03': # CTRL-C
            break
        elif key == 'w':
            node.send_cmd_vel(0.2, 0.0)  # Move forward
        elif key == 'x':
            node.send_cmd_vel(-0.2, 0.0)  # Move backward
        elif key == 'd':
            node.send_cmd_vel(0.0, -0.5)  # Turn right
        elif key == 'a':
            node.send_cmd_vel(0.0, 0.5)  # Turn left
        elif key == ' ' or key == 's':
            node.send_cmd_vel(0.0, 0.0)  # Stop

    node.send_cmd_vel(0.0, 0.0)  # Stop the robot before exiting
    rclpy.shutdown()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()
```

O botão: 
* w: Move o turtlebot pra frente
* x: Move ele para trás
* d: Move ele para direita
* a: Move ele para a esquerda
* espaço ou s: Para o robô
* ctrl + c: corta a comunicação e fecha nó. 

(Este código vai mudar um pouco pois: iremos implementar mudanças futuras ou integrar outras funções com a CLI)

## Decisões de design 
(Explicar a importância)

Para que o usuário consiga aproveitar ao máximo dessa comunicação com o robô, pensamos em duas soluções viáveis: 
* CLI (Interface por Linha de Comando) - para controlar o robô pelo terminal e executar funções pré-definidas de uma forma mais rápida e fácil;
* Interface gráfica - uma interface onde o usuário possa controlar através de uma tela, apertando botões e interagindo com o robô ao invés de comandos via terminal. 

### CLI 

Logo, nesta Sprint, achamos justo criarmos uma CLI para que seja possível navegar pelo robô. Esta abordagem é mais direta, rápida e fácil para o usuário. O ponto negativo é que talvez não seja tão intuitivo no começo como ter uma interface. 

(Explicar as funcionalidades da CLI)

#### Botão de emergência
Aqui surgiram questões importantes como - e se acontecer uma comunicação inesperada com o robô? E se quisermos interromper totalmente a comunicação com ele? Para isso, criamos um botão de emergência onde, em qualquer momento da atividade com o turtlebot, seja possível pará-lo. 

(Explicar como isso acontece)

### Wireframe 
Para avançarmos no desenvolvimento de uma interface gráfica, achamos necessários já planejar uma interface gráfica. Então, prototipamos um wireframe onde é possível ver as principais funcionalidades da aplicação. 

<!--
#### Imagem 1 do Wireframe - tela de login
![Imagem 1 do Wireframe - tela de login](/img/wireframe-login.png)

#### Imagem 2 do Wireframe - tela principal
![Imagem 2 do Wireframe - tela principal](/img/wireframe-principal.png)

#### Imagem 3 do Wireframe - tela de visualização
![Imagem 3 do Wireframe - tela de visualização](/img/wireframe-visu.png)

#### Imagem 4 do Wireframe - tela de limpo
![Imagem 4 do Wireframe - tela de limpo](/img/wireframe-limpo.png)

#### Imagem 5 do Wireframe - tela de obstruído
![Imagem 5 do Wireframe - tela de obstruído](/img/wireframe-obstruido.png)

(Explicaremos um por um)-->

# Como executar

(Explicar parte de rodar o Docker)

