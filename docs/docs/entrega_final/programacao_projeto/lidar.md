---
title: "Lidar"
sidebar_position: 2
---

# Sensor Lidar

Este documento detalha a integração e utilização do sensor de distância a laser 360 Laser Distance Sensor (LDS-02) para que, quando uma pessoa estiver controlando o robô, o sensor auxilia a evitar colisões e obter informações da distãncia do robô a um certo objeto. No decorrer do documento, será comentado a instalação e configuração do LDS-01, a coleta e análise dos dados de varredura do ambiente ao redor do robô e a implementação da interface de comunicação necessária para o funcionamento do sensor.

## O que é o Lidar?
O Lidar, que significa “Light Detection and Ranging” ou “Detecção e Medição por Luz”, é uma tecnologia de sensoriamento remoto que utiliza pulsos de luz laser para medir distâncias variadas a partir do sensor até um objeto ou superfície. Essa tecnologia é amplamente utilizada em diversas aplicações. 

O modelo específico deste projeto, o LDS-01, é um sensor Lidar desenvolvido pela empresa Hokuyo que está presente no Turtlebot. Ele é conhecido por sua capacidade de realizar mapeamento e localização simultânea (SLAM) em ambientes internos. Algumas características principais do LDS-01 incluem:

1.	**Alcance**: Geralmente, os Lidars desta categoria podem medir distâncias de até alguns metros com alta precisão.
2.	**Taxa de amostragem**: O LDS-01 pode realizar milhares de medições por segundo, o que permite criar mapas detalhados do ambiente em tempo real.
3.	**Campo de visão**: Este modelo possui um amplo campo de visão, o que é ideal para navegação e mapeamento em robótica.
4.	**Tamanho e portabilidade**: É compacto e leve, adequado para uso em pequenos robôs ou drones.

## Por que um Lidar neste projeto? 

O Lidar é um recurso que pode ser utilizado para evitar colisões do robô ao detectar a presença de objetos ou superfície. Logo, ele se faz de extrema importância neste projeto. 

## Setup do LDS-01 (LIDAR)

Como o Lidar é um scanner a laser 2D capaz de realizar varreduras de 360 graus, coletando um conjunto de dados ao redor do robô para ser utilizado em SLAM (mapeamento e localização simultânea), optou-se por utiliza-lo e fazer a comunicação para que ele enviasse as mensagens de distância dos objetos que estavam ao seu redor.

E para configurar o lidar, tem-se que seguir alguns passos que podem ser vistos aqui abaixo:
```cmd
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
```
Depois, clone o repositório:
```cmd
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
```
Volte para a pasta do workspace com:
```cmd
cd ~/turtlebot3_ws/
```
E por fim, ccompile o pacote com:
```cmd
colcon build --symlink-install
```
Após compilar o pacote, foi configurado o bashrc para dar source no arquivo de instalação do pacote com:
```cmd
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
```
Também precisa-se especificar o modelo do LIDAR em uma variável de ambiente. Rode:
```cmd
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
```
Lembre-se que para poder utilizar essas mudanças sem reiniciar o terminal ainda precisa rodar:
```cmd
source ~/.bashrc
```

## Implementação do sensor LIDAR na movimentação:
Na Sprint 3, foram implementadas mudanças na CLI. Agora, ao controlar o robô pelo terminal, é possível tirar proveito do Lidar. Dentro do ROS, é possível inscrever em um tópico para "escutar" as informações do Lidar presente no Turtlebot. 

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
```

Ao criar um nó do ROS, pode-se inscrever no tópico *scan* para receber as mensagens do tipo **LaserScan**, que neste caso é o nosso Lidar. Além disso, é preciso passar também criar um QOS Profile. 

No ROS, o QoS (Quality of Service) Profile define como as mensagens são gerenciadas entre os nós, otimizando a entrega de dados conforme a aplicação. O script, o QoS é configurado para manter apenas as últimas cinco mensagens (HistoryPolicy.KEEP_LAST), não garantir a entrega de todas as mensagens (ReliabilityPolicy.BEST_EFFORT), e não armazenar mensagens para assinantes futuros (DurabilityPolicy.VOLATILE). Essas configurações ajudam a manter o sistema eficiente e responsivo, crucial para aplicações de robótica que dependem de atualizações frequentes e rápidas de dados, como é o caso do uso de dados de Lidar em robôs móveis.

```python
def lidar_callback(self, msg):
    front_index = 0
    right_index = int(len(msg.ranges) / 4)
    back_index = int(len(msg.ranges) / 2)
    left_index = int(len(msg.ranges) * 3 / 4)

    front_distance = msg.ranges[front_index]
    left_distance = msg.ranges[left_index]
    back_distance = msg.ranges[back_index]
    right_distance = msg.ranges[right_index]

    self.get_logger().info(f'Front distance: {front_distance}, Left distance: {left_distance}, Back distance: {back_distance}, Right distance: {right_distance}')

    self.obstacle_detected_forward = front_distance < STOP_DISTANCE
    self.obstacle_detected_left = left_distance < STOP_DISTANCE
    self.obstacle_detected_right = right_distance < STOP_DISTANCE
    self.obstacle_detected_backward = back_distance < STOP_DISTANCE
```

Nesta função chamada "lidar_callback" as informações do lidar são enviadas a cada 0.1 segundos e faz-se uma verificação para verificar se há algo na frente, trás, esquerda ou direita do robô. Caso há, a variável criada no construtor que armazena se há um obstáculo ou não é acionada. 

```python 
    if self.obstacle_detected_backward or self.obstacle_detected_forward or self.obstacle_detected_left or self.obstacle_detected_right and self.target_linear_vel > 0:
        self.control_linear_vel = 0.0
    self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
    self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
```

Caso há, a velocidade de controle é colocada para zero. Ou seja, o robô imediatamente para, sendo só possível se mover para os lados, mas nunca em frente ao obstáculo. 

O código inteiro pode ser visualizado em: https://github.com/Inteli-College/2024-1B-T08-EC06-G02/blob/86-s3-robo-lidar/src/backend/lidar.py

## Sensor Lidar pela interface gráfica 
Como comentado, além da nova CLI implementada, também foi realizado o desenvolvimento da interface gráfica. Nela, caso o robô esteja prestes a entrar em colisão, um popup é exibido dizendo que está prestes a bater. Esta implementação é importante para evitar acidentes. 

Tendo em vista que está sendo utilizado ReactJS, uma biblioteca/framework para criação de interfaces gráficas com JavaScript, a biblioteca Roslibjs entra em conjunto para fazer a comunicação com o Rosbridge, quem vai nos ajudar a receber e enviar dados para o robô. 

```javascript
    useEffect(() => {
        var ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        var listener = new ROSLIB.Topic({
            ros: ros,
            name: '/scan',
            messageType: 'sensor_msgs/LaserScan'
        });

        listener.subscribe(function (message) {
            console.log('Mensagem recebida do ' + listener.name + ': ' + message.ranges);

            const validRanges = message.ranges.filter(range => !isNaN(range) && range !== null && range !== undefined);

            if (validRanges.length > 0) {
                var min_distance = Math.min(...validRanges);

                console.log('Menor valor do ' + listener.name + ': ' + min_distance);
                if (min_distance < LIDAR_RANGE) {
                    setMostrarAlerta(true);
                    setValorLidar(min_distance);
                } else {
                    setMostrarAlerta(false);
                }
            }
        });

        return () => {
            listener.unsubscribe();
            ros.close();
        };
    }, []);
```

Neste código, é criado o useEffect, um hook utilizado para ser executado apenas uma vez após o componente ser renderizado inicialmente. Os principais propósitos do useEffect neste contexto:

1.	**Inicialização da conexão**: Ao iniciar, o useEffect configura a conexão WebSocket com o ROS, usando a biblioteca roslibjs para se conectar ao servidor Rosbridge em ‘ws://localhost:9090’. Isso permite que o componente React interaja diretamente com o sistema robótico.
2.	**Subscrição a tópicos do ROS**: Dentro do useEffect, é criada uma subscrição ao tópico /scan, que recebe dados do tipo sensor_msgs/LaserScan provenientes do Lidar. Isso permite que o componente React processe dados sensoriais em tempo real.
3.	**Manipulação de mensagens**: Quando mensagens são recebidas através do tópico, o código processa esses dados para filtrar valores inválidos e calcular a distância mínima. Se a distância mínima estiver abaixo de um limiar predefinido (LIDAR_RANGE), o componente atualiza o estado para mostrar um alerta.
4.	**Limpeza e desconexão**: A função retornada pelo useEffect é um ‘cleanup’ que será chamado quando o componente for desmontado. Isso é importante para evitar vazamentos de memória e outros problemas, garantindo que a subscrição ao tópico seja cancelada e a conexão WebSocket seja fechada.

```javascript
    return (
        <Space direction="vertical" style={{ width: '50%' }}>
            {mostrarAlerta && valorLidar !== null && (
                <Alert
                    message="Cuidado"
                    description={"O robô está prestes a colidir com um objeto em " + valorLidar.toFixed(2) + " metros."}
                    type="warning"
                    showIcon
                    closable
                />
            )}
        </Space>
    );
```

E para exibir o Pop Up de alerta, no useEffect do lidar 'setamos' como true a variável para mostrar o alerta e passamos o valor atual (em metros) para ser exibido. Isto acontece quando o robô está abaixo do valor definido. Para a criação do Pop Up utilizamos a biblioteca Ant Design. 

O código inteiro pode ser visualizado em: https://github.com/Inteli-College/2024-1B-T08-EC06-G02/blob/96-s3-popup-colisao/src/frontend/src/components/home/popup-colisao.js

## Demonstração
![Demonstração Lidar](https://s3.ezgif.com/tmp/ezgif-3-04e50646e6.gif)

Neste exemplo teste de demonstração, o script da CLI rodando, além do frontend com a comunicação websocket. Nele, pode-se perceber o resultado da implementação: quando o robô chega a 0.2 metros de algum objeto (neste caso a parede), é exibido um PopUp de alerta para a colisão (canto esquerdo) e o robô é parado (canto direito). Caso ele volte a posição de segurança, o PopUp é retirado.
