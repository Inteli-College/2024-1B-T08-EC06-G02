---
sidebar_position : 4
---

# Metodologia Sprint 2

A seguir, este documento descreverá as atualizações da Sprint 2 do grupo Repipe. O foco ao longo dessas duas semanas foram, principalmente, a movimentação do Robô Turtlebot. Logo, será abordado sobre o setup do robô, a criação de uma interface navegável para interagir com o robô e as comunicações necessárias para isso acontecer. 

# Setup do Robô 

Este é um passo importante do projeto, pois realiza as configurações iniciais do Turtlebot, robô utilizado neste projeto. O Turtlebot é um robô que possui um Raspberry Pi, um módulo driver e um sensor LiDAR. Com isso, foi instalado um sistema operacional no cartão sd inserido para configurá-lo e instalar as dependências necessárias. Este foi o fluxo realizado nessa etapa:

* A Raspberry Pi foi conectada ao monitor;
* A Raspberry Pi Imager foi instalado;
* O sistema operacional Ubuntu 22.04 LTS foi instalado no micro cartão SD;
* A conexão com o Raspberry Pi foi realizada utilizando ssh para uma solução headless;
* O Ros Humble foi instalado;
* Os pacotes do Turtlebot3 foram instalados;
* O pacote do LIDAR LDS-02 foi compilado;
* O setup do OpenCR foi realizado;

# Interface navegável 


Depois de ter instalado e configurado os pacotes do Turtlebot3, é preciso criar um método efetivo para controlar o robô. Para isso, nesta seção será abordado a comunicação com o robô e como é feito para isso chegar com uma boa usabilidade ao usuário final. 

## Comunicação com o Robô 

A estrutura do ROS é baseada em nós, tópicos, mensagens e serviços, que juntos criam uma arquitetura flexível e poderosa para robótica do nosso projeto. 

#### Nós
No ROS 2, um nó representa um processo que realiza computação. Nós são entidades autônomas que podem se comunicar com outros nós por meio de tópicos ou serviços. No código mencionado abaixo, o TeleopTurtle é um nó que foi criado para enviar comandos de controle ao Turtlebot3.

#### Tópicos e Mensagens
Tópicos são canais onde os nós podem publicar ou se inscrever para ler dados. No nosso script, o nó TeleopTurtle publica mensagens no tópico cmd_vel. Essas mensagens são do tipo Twist, que é uma estrutura de dados definida em geometry_msgs.msg. Essa mensagem contém dois vetores principais: linear e angular. Esses vetores especificam a velocidade na direção linear (x, y, z) e angular (roll, pitch, yaw), respectivamente. 

Quando publica-se uma mensagem Twist no tópico cmd_vel, o Turtlebot3 recebe essa mensagem e a interpreta como uma instrução para mover-se de acordo com os parâmetros de velocidade especificados.

#### Serviços
Serviços no ROS 2 são outra forma de comunicação entre nós. Diferentemente dos tópicos, que são geralmente usados para transmissão contínua de dados, os serviços são mais adequados para interações de requisição-resposta. Neste script, especificamente, não estamos usando, mas, por exemplo, pode-se usar um serviço para configurar parâmetros do robô ou para iniciar/parar determinadas ações.

#### Rede Local e Comunicação
Além disso, é válido mencionar que o ROS 2 permite que os nós se comuniquem eficientemente em uma rede local ou mesmo distribuídos por várias máquinas. No cenário em questão, tanto o nó TeleopTurtle quanto o robô Turtlebot3 estão operando na mesma rede local, permitindo que as mensagens sejam transmitidas. 

Abaixo, segue uma explicação detalhada de cada parte do código. O grupo se inspirou neste repositório [aqui](https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key). Porém, achamos muito confuso e diminuímos/refatoramos algumas coisas. 


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
Depois, cria uma classe `TeleopTurtle`, que é um nó ROS 2 projetado para facilitar a teleoperação do Turtlebot3. Sua implementação:

```python
class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()
```
* O publisher é configurado para publicar mensagens no tópico `cmd_vel`, que o Turtlebot3 escuta para receber comandos de movimento. Um publisher é um componente que publica mensagens em um tópico específico. Ele é responsável por produzir dados que outros nós podem consumir. Aqui, a classe TeleopTurtle possui um publisher configurado para enviar mensagens ao tópico cmd_vel. Este é o mecanismo pelo qual o nó envia comandos de movimento para o Turtlebot3.

### Método send_cmd_vel
O método send_cmd_vel é usado para definir e enviar os comandos de velocidade para o robô. Como se faz isso?

```python
    def send_cmd_vel(self, linear_vel, angular_vel):
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = angular_vel
        self.publisher_.publish(self.twist_msg)
        print("Linear Vel: {}, Angular Vel: {}".format(linear_vel, angular_vel))
```
* **Configuração de Velocidades**: O método ajusta as propriedades linear.x e angular.z do objeto Twist chamado twist_msg. A propriedade linear.x afeta o movimento para frente ou para trás do robô, enquanto angular.z controla a rotação do robô sobre seu eixo vertical (yaw). Definindo linear.x como positivo, o robô avança; negativo, ele recua. Similarmente, um valor positivo de angular.z faz o robô girar para a esquerda, e um valor negativo para a direita.
* **Publicação da Mensagem**: Após definir as velocidades no twist_msg, o método utiliza self.publisher_, que é um objeto publisher, para enviar essa mensagem ao tópico cmd_vel. Este tópico é monitorado pelo robô, que aguarda as instruções de movimento.
**Feedback para Debug**: Finalmente, as velocidades são impressas no console. Essa saída é crucial para debug e verificação dos comandos enviados, permitindo a gente confirmar se os valores das velocidades estão conforme o esperado.

### Enviando movimentos para o robô 

Certo. Depois de estabelecer a estrutura necessária pra movimentar o turtlebot, como é controlado? Nesta parte da implementação, foi configurado para que o usuário em controle possa apertar teclas como `w, x, d, a` e consiga operar o robô. 

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
            node.send_cmd_vel(0.2, 0.0)  # Move pra frente
        elif key == 'x':
            node.send_cmd_vel(-0.2, 0.0)  # Move pra trás
        elif key == 'd':
            node.send_cmd_vel(0.0, -0.5)  # Vira à direita
        elif key == 'a':
            node.send_cmd_vel(0.0, 0.5)  # Vira à direita
        elif key == ' ' or key == 's':
            node.send_cmd_vel(0.0, 0.0)  # Para

    node.send_cmd_vel(0.0, 0.0)  # Para o robô antes de sair
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

### Resumo
De forma resumida, o script que foi desenvolvido utiliza os conceitos do ROS para implementar uma forma de teleoperação onde:

* Um nó (TeleopTurtle) é criado e configurado para enviar comandos.
* Mensagens do tipo Twist são usadas para encapsular as instruções de movimento.
* Tópicos servem como o meio de comunicação para enviar esses comandos ao robô.
* A interface do usuário (captura de teclas no terminal) é mapeada para comandos de movimento, tornando a interação com o robô intuitiva e responsiva.

## Decisões de design 
Para que o usuário consiga aproveitar ao máximo dessa comunicação com o robô, duas soluções viáveis foram consideradas: 

* CLI (Interface por Linha de Comando) - para controlar o robô pelo terminal e executar funções pré-definidas de uma forma mais rápida e fácil;
* Interface gráfica - uma interface onde o usuário possa controlar através de uma tela, apertando botões e interagindo com o robô ao invés de comandos via terminal. 

A escolha entre utilizar uma Interface de Linha de Comando (CLI) ou uma interface gráfica no controle de um robô AGV é crucial para definir a eficiência, a facilidade de uso e a acessibilidade do sistema. A CLI se destaca pela sua rapidez e eficácia em execuções, requer menos recursos do sistema, enquanto a interface gráfica aprimora a experiência do usuário com interações visuais intuitivas e feedback, embora exija mais do ponto de vista computacional e um desenvolvimento mais elaborado.


### CLI 

Logo, nesta Sprint, o grupo considerou válido criar uma CLI para que seja possível navegar pelo robô. Esta abordagem é mais direta, rápida e fácil para o usuário. O ponto negativo é que talvez não seja tão intuitivo quanto interface. 

Uma CLI é uma interface de linha de comando que permite aos usuários interagir com um programa ou sistema por meio de comandos de texto digitados em um terminal. Com uma CLI, os usuários podem executar tarefas, fornecer entrada e receber saída diretamente na linha de comando, sem a necessidade de uma interface gráfica.

No contexto desse projeto, a implementação de uma CLI permitiria que os usuários controlassem o robô Turtlebot3 por meio de comandos digitados no terminal. Isso proporcionaria uma forma direta e rápida de interagir com o robô, executando funções pré-definidas e controlando seu movimento.

Mantendo apenas a mesma classe TeleopTurtle com a função send_cmd_vel ao código que foi mencionado anteriormente, os principais ajustes que foram feitos para implementar uma CLI, foi:
* Importar bibliotecas para criar a CLI;
* Criar a função necessária de acordo com as entradas do usuário.

#### Implementação da CLI

```python
import typer
import inquirer

app = typer.Typer()
```

O código acima importa as bibliotecas necessárias. A biblioteca typer é uma biblioteca de linha de comando que permite criar facilmente interfaces de linha de comando interativas e amigáveis.

Já a biblioteca inquirer é uma biblioteca de interação de linha de comando que fornece uma maneira fácil de criar perguntas interativas para o usuário. Com o inquirer, pode-se criar perguntas de múltipla escolha e perguntas de entrada de texto.

Sendo assim, foi implementado a seguinte função ao nosso código:

```python
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
``` 

A função control() é definida como um comando do aplicativo. Dentro dessa função, o código inicializa o sistema de comunicação do ROS usando a função rclpy.init(). Em seguida, cria uma instância da classe TeleopTurtle, responsável por enviar comandos de movimento para o robô.

O programa exibe uma mensagem de boas-vindas para o usuário, e em seguida, cria uma lista de perguntas usando a biblioteca inquirer. Essa lista contém uma pergunta com opções para o usuário selecionar uma ação. Se o usuário selecionar "Sair", o loop é interrompido e o programa é encerrado. Se o usuário selecionar "Frente", "Trás", "Esquerda" ou "Direita", o programa chama o método send_cmd_vel() da instância node para enviar comandos de velocidade para o robô. Se o usuário selecionar "Emergência (Parar Funcionamento)", o programa envia um comando de velocidade nulo para parar o robô.

Se ocorrer qualquer exceção durante a execução do programa, a mensagem "Interface Fechada a Força, Parando movimentação do robô" é exibida e um comando de velocidade nulo é enviado para parar o robô. Finalmente, independentemente de qualquer exceção, o programa envia um comando de velocidade nulo e encerra o sistema de comunicação do ROS usando as funções node.send_cmd_vel(0.0, 0.0) e rclpy.shutdown().

#### Botão de emergência
Aqui surgiram questões importantes como: e se acontecer uma comunicação inesperada com o robô? E se quisermos interromper totalmente a comunicação com ele? Para isso, criamos um botão de emergência onde, em qualquer momento da atividade com o turtlebot, seja possível pará-lo. 

Isto é o que está presente no botão Emergência '(Parar Funcionamento)', uma opção de segurança para evitar qualquer movimento que irá prejudicar o robô ou a operação. 

```python 
case 'Emergência (Parar Funcionamento)':
                    node.send_cmd_vel(0.0, 0.0)
```
Se o usuário selecionar "Emergência (Parar Funcionamento)", o programa envia um comando de velocidade nulo para parar o robô.