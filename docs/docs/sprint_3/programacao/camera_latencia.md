---
title: "Câmera e Latência"
sidebar_position: 1
---

# Comunicação com a Câmera/Cálculo da Latência

O foco deste documento é explicitar como foi realizado durante essa sprint a implementação de uma webcam para observar o ambiente do robô e o cálculo da latência na transmissão das imagens. A seguir, será discutido o setup da webcam, a criação de uma interface para visualização das imagens da webcam e as comunicações necessárias para que isso aconteça.

## Comunicação e Integração

A integração entre o Turtlebot3, a webcam e a interface de visualização exigiu uma comunicação eficiente entre todos os componentes. Para facilitar essa comunicação, utilizamos o ROSBridge, uma ferramenta que permite a comunicação entre um sistema ROS e outras aplicações por meio de protocolos baseados na web, como WebSockets. Esta funcionalidade foi crucial, pois possibilitou a integração dos robôs controlados por ROS com aplicações web e outros sistemas externos, garantindo que as imagens fossem transmitidas e recebidas com a menor latência possível. Assim, essa abordagem assegura que o sistema opere de forma suave e responsiva, permitindo uma interação eficaz entre o robô e a interface web.

## Setup da Webcam

Para realizar as primeiras configurações para as transmissões das imagens, inicialmente adquirimos uma webcam modelo DOBOT Magician para ser integrada ao Turtlebot3. A conexão foi estabelecida através das portas USB disponíveis, permitindo a leitura dos dados capturados pela webcam.

Sendo assim, para configurar a transmissão dessas imagens através da rede, foi desenvolvido um código no arquivo denominado `sender.py`, utilizando o ROSBridge para criar um WebSocket que possibilitasse a conexão entre os equipamentos e o ROS.

O código `sender.py` foi especificamente projetado para transmitir as imagens da webcam conectada ao TurtleBot3 através de uma rede Wi-Fi. A seguir, encontra-se o código completo:<br/>

***sender.py***
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading

# Configuração das constantes da câmera
IM_WIDTH = 1280
IM_HEIGHT = 720

# Criação de classe para o gerenciamento do publisher da webcam 
class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  
        self.cap = cv2.VideoCapture(index=0)
        self.latency_thread = threading.Thread(target=self.latencia)
        self.latency_thread.start()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)

    # Função responsável por calcular a latência da webcam e publicar ela na tela de exibição 
    def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)

        # Configuração da resolução da webcam
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)

        # Obtenção da resolução real da webcam
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))

        # Inicializa variáveis para medir o tempo e contar frames
        prev_tick = cv2.getTickCount()
        frame_number, prev_change_frame = 0, 0

        while True:
            frame_number += 1
            ret, frame = self.cap.read()
            if not ret:
                break
            
            new = cv2.getTickCount()

            print("{:.3f} sec, {:.3f} frames".format(
                (new - prev_tick) / cv2.getTickFrequency(),
                frame_number - prev_change_frame))

            prev_tick = new
            prev_change_frame = frame_number

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

# Função principal que inicializa o código
def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
<h6 align="center"> Fonte: Elaboração grupo Repipe </h6>

Tendo esse código analisado observa-se que além de fazer as capturas das imagens da webcam, ele faz a compressão dessas imagens para JPEG e publica no tópico "/video_frames" utilizando o ROS. Assim, esse código permite a transmissão eficiente das imagens capturadas pela webcam, possibilitando uma integração suave e eficaz entre o Turtlebot3, a webcam e quaisquer interfaces de visualização.<br/>


## Interface de Visualização das Imagens

Para a visualização dos frames capturados pela webcam foi criado um arquivo HTML denominado `imagens.html`. Nesse sentido, utilizando o ROSBridge, foi estabelecido uma comunicação via WebSocket, permitindo o recebimento desses frames que foram então decodificadas e exibidas em tempo real na página web. A implementação dessa interface pode ser visualizada abaixo:

**imagens.html**
```html
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<!-- Bibliotecas do ROS -->
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>

<!-- Estabelecendo conexão via WebSocket -->
<script type="text/javascript">
  var ros = new ROSLIB.Ros({
    url : 'ws://10.128.0.17:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  // Topico para receber os frames do video
  var videoTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/video_frames',
    messageType : 'sensor_msgs/CompressedImage'
  });

  // Funcao para lidar rcom a entrada dos frames do video 
  videoTopic.subscribe(function(message) {
    var img = document.getElementById('videoStream');
    img.src = 'data:image/jpeg;base64,' + message.data;
  });

  window.onload = function() {
    // Subescrevendo os frames do video uma unica vez
    videoTopic.subscribe();
  };
</script>
</head>
<body>
  <h1>Real-time Video Stream from ROS2 Topic</h1>
  <img id="videoStream" alt="Video Stream" style="width: 640px; height: 480px;" />
</body>
</html>
```

## Frontend

O desenvolvimento do frontend foi realizado utilizando React JS para a construção de componentes e CSS puro para estilização, o que permitiu a criação de uma interface alinhada com as expectativas visuais do projeto. 

### Estrutura de Pastas

A organização dos arquivos no projeto segue uma estrutura que facilita a manutenção do código:



- **'components'**: Composta por componentes como botões (exemplo: botao-iniciar.js). Cada componente foi desenvolvido para ser independente, com suas próprias propriedades, facilitando a reutilização em diferentes páginas. 
- **'pages'**: Armazena os scripts em React JS das páginas da aplicação como a página de login (login.js), a tela principal (principal.js) e a tela de visualização (visualizar.js).
- **'static/partials'**: Inclui arquivos CSS específicos para cada componente e página, permitindo uma estilização em toda a aplicação. O CSS puro foi utilizado para colocar em prática o design do mockup. Arquivos como visualizar.css e login.css contêm as regras específicas para as páginas correspondentes, enquanto main.css define os estilos globais utilizados em toda a aplicação.

### Integração com o Backend
Utilizando o WebSocket com o ROSBridge, como descrito anteriormente na seção sobre comunicação e visualização de imagem, o frontend se inscreve em tópicos específicos do ROS para receber dados continuamente atualizados. Essa comunicação está sendo feita diretamente no script da página relacionada.

## Cálculo da latência

A latência representa o tempo de atraso entre o envio e o recebimento de dados. Para realizar esse cálculo no contexto do projeto, iniciamos a medição no momento em que os frames eram capturados pela webcam do DOBOT Magician. Assim, o cálculo da latência era interrompido somente quando os frames alcançavam o destino final da aplicação, ou seja, na página HTML onde as imagens capturadas pela webcam eram apresentadas.<br/>

Dessa maneira, este método de cálculo permitiu medir o tempo total necessário para que os frames percorressem o caminho completo desde a captura até a exibição, proporcionando uma visão precisa do desempenho do sistema.<br/>

Para uma compreensão detalhada da construção da latência no projeto, é crucial revisitar o código `sender.py`, onde toda a estrutura foi elaborada. Em especial, a função ***latencia()*** explicita como o cálculo da latência foi implementado. Com isso em mente, a função apresenta detalhes sobre como foi realizada a medição do tempo de atraso desde a captura dos frames pela webcam até a sua exibição na página HTML. Abaixo, pode-se ter uma melhor noção de como se deu sua implementação:<br/>

```python
def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)

        # Configuração da resolução da webcam
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)

        # Obtenção da resolução real da webcam
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))

        # Inicializa variáveis para medir o tempo e contar frames
        prev_tick = cv2.getTickCount()
        frame_number, prev_change_frame = 0, 0

        while True:
            frame_number += 1
            ret, frame = self.cap.read()
            if not ret:
                break
            
            new = cv2.getTickCount()

            print("{:.3f} sec, {:.3f} frames".format(
                (new - prev_tick) / cv2.getTickFrequency(),
                frame_number - prev_change_frame))

            prev_tick = new
            prev_change_frame = frame_number

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
```

Dentro dessa função, temos o loop, que se inicia no `While`, e é nele que a lógica para realizar o cálculo da latência é realmente implementado.<br/>
Ele funciona da seguinte maneira, a cada iteração, ele captura um frame da webcam e incrementa o contador de frames. Se a captura falhar, o loop é interrompido.<br/>

A função `cv2.getTickCount()` é usada para obter o número atual de ciclos de clock desde o início do sistema. Para calcular a latência, a diferença entre este valor e o valor anterior `(prev_tick)` é dividida pela frequência do clock do sistema, obtida com `cv2.getTickFrequency()`, que converte ciclos de clock em segundos.<br/>

A taxa de frames é calculada como a diferença entre o contador de frames atual `(frame_number)` e o contador da última medição `(prev_change_frame)`. Esses valores de latência e taxa de frames são impressos em tempo real, fornecendo informações sobre a performance da captura de vídeo.<br/>

O loop pode ser interrompido manualmente pressionando a tecla 'q'. Após sair do loop, a captura de vídeo é liberada e todas as janelas do OpenCV são fechadas, garantindo a liberação correta dos recursos.<br/>

### Conclusão
Portanto, este documento descreve a implementação de uma webcam para monitoramento do ambiente do robô TurtleBot3, juntamente com o cálculo da latência na transmissão das imagens. 

Utilizando o ROSBridge para comunicação entre o sistema ROS e outras aplicações via WebSockets, o código `sender.py` foi desenvolvido para capturar, comprimir e publicar as imagens da webcam no tópico "/video_frames" do ROS. 

A função `latencia` do código `sender.py` mediu a latência e a taxa de frames, fornecendo dados para monitorar a performance do sistema em tempo real. A interface de visualização, implementada em HTML, recebeu os frames via ROSBridge e os exibiu, garantindo uma experiência de monitoramento eficiente. 
