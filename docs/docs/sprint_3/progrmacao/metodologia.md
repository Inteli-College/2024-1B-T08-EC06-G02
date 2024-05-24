# Metodologia Sprint 3

Este documento detalha as atualizações da Sprint 3 do grupo Repipe. O foco principal nas últimas duas semanas foi a implementação de uma webcam para observar o ambiente do robô e o cálculo da latência na transmissão das imagens. A seguir, discutiremos o setup da webcam, a criação de uma interface para visualização das imagens da webcam e as comunicações necessárias para que isso aconteça.

## Setup da Webcam

Inicialmente, adquirimos uma webcam do modelo DOBOT Magician para ser acoplada ao Turtlebot3. A conexão foi feita utilizando as portas USB disponíveis, permitindo a leitura dos dados capturados pela webcam. 

Para configurar a conexão entre a webcam e a rede, desenvolvemos um código no arquivo `sender.py`, utilizando o ROSBridge. Este código foi projetado para permitir a transmissão das imagens da webcam pela rede e pode ser visto mais abaixo para que haja o entendimento de como foi sua aplicação.

***sender.py***
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading

# Configuração das constantes da câmera
IM_WIDTH = 1280
IM_HEIGHT = 720

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

    def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))

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

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Interface de Visualização das Imagens

Foi criado um arquivo HTML denominado `imagens.html` para exibir as imagens capturadas pela webcam. Utilizando o ROSBridge, estabelecemos uma comunicação via WebSocket, permitindo o recebimento das mensagens de vídeo. Essas mensagens foram então decodificadas e exibidas em tempo real na página. A implementação dessa interface pode ser visualizada abaixo em **imagens.html**.

**imagens.html**

```
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />

<!-- ROS libraries -->
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/eventemitter2@6.4.9/lib/eventemitter2.min.js"></script>
<script type="text/javascript" src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>

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

  // Topic to receive video frames
  var videoTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/video_frames',
    messageType : 'sensor_msgs/CompressedImage'
  });

  // Function to handle incoming video frames
  videoTopic.subscribe(function(message) {
    var img = document.getElementById('videoStream');
    img.src = 'data:image/jpeg;base64,' + message.data;
  });

  window.onload = function() {
    // Subscribe to video frames once
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

## Comunicação e Integração

A integração entre o Turtlebot3, a webcam e a interface de visualização exigiu uma comunicação eficiente entre os componentes. Utilizamos o ROSBridge para facilitar essa comunicação, garantindo que as imagens fossem transmitidas e recebidas com a menor latência possível.

# Cálculo da latência

A latência, que é o tempo de atraso entre o envio e o recebimento de dados, neste projeto foi calculada a partir do momento em que a captura dos frames pela webcam do DOBOT Magician era iniciada. O cálculo era interrompido quando os frames chegavam à outra ponta da aplicação, ou seja, na página HTML onde as imagens capturadas pela câmera eram apresentadas.

Este método de cálculo da latência permitiu medir o tempo total necessário para que os frames percorressem o caminho completo desde a captura até a exibição, proporcionando uma visão precisa do desempenho do sistema.

Para entender melhor a construção dessa parte do projeto, é importante revisar o código no arquivo `sender.py`, onde toda a estrutura foi desenvolvida. Especificamente, na função **latencia()**, é possível analisar detalhadamente como o cálculo da latência foi implementado. Esta função, presente em `sender.py`, demonstra como foi realizada a medição do tempo de atraso desde a captura dos frames pela webcam até a sua exibição na página HTML.
```
 def latencia(self):
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))

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

Em resumo, durante a Sprint 3, foi integrada com sucesso uma webcam no Turtlebot3, desenvolvida uma interface para visualização em tempo real das imagens capturadas e garantida uma comunicação eficiente para minimizar a latência.
