# Prova de Conceito: Controle de Movimento de um Robô Utilizando ROS e React

## Introdução

Este documento apresenta a prova de conceito para o controle de movimento de um robô utilizando ROS (Robot Operating System) integrado a uma aplicação web em React. O objetivo é demonstrar a capacidade de controlar um robô, visualizar seu vídeo em tempo real e monitorar a latência da comunicação entre o robô e a interface web.

## Estrutura do Código

O código foi desenvolvido utilizando a biblioteca ROSLIB para a comunicação com o ROS e a biblioteca React para a criação da interface do usuário. Abaixo, detalharemos os principais componentes e funcionalidades do código.

## Importações e Inicializações

```javascript
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import '../static/main.css';  // Caminho correto para o CSS
import BotaoIniciar from '../components/home/botao-iniciar'; // Corrige a importação para minúsculas
import BotaoVisualizar from '../components/home/botao-visualizar'; // Corrige a importação para minúsculas
import BotoesMover from '../components/home/botoes-mover'; // Corrige a importação para minúsculas
import { PopUpColisao } from '../components/home/popup-colisao';
import AbaVisualizar from '../components/home/aba-visualizar';
```

## Configuração dos Estados e Constantes

Os estados são utilizados para gerenciar a interface do usuário e monitorar colisões e latência. Constantes são definidas para os limites de velocidade linear e angular do robô.

```javascript
const [videoSrc, setVideoSrc] = useState('');
const [isAbaVisible, setIsAbaVisible] = useState(false);
const [colisaoFrente, setColisaoFrente] = useState(false);
const [colisaoTras, setColisaoTras] = useState(false);
const [colisaoEsquerda, setColisaoEsquerda] = useState(false);
const [colisaoDireita, setColisaoDireita] = useState(false);
const [latencyData, setLatencyData] = useState('');
const STOP_DISTANCE = 0.5; 

const MAX_LIN_VEL = 0.21; // m/s
const MIN_LIN_VEL = -0.21; // m/s
const MAX_ANG_VEL = 2.63; // rad/s
const MIN_ANG_VEL = -2.63; // rad/s
```

## Conexão com o ROS

Estabelecemos a conexão com o ROS utilizando WebSocket e configuramos os tópicos para vídeo, controle e monitoramento de latência.

```javascript
useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    const videoTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/video_frames',
      messageType: 'sensor_msgs/CompressedImage'
    });

    const controlTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    const fpsListener = new ROSLIB.Topic({
      ros: ros,
      name: '/fps',
      messageType : 'std_msgs/String'
    });

    fpsListener.subscribe((message) =>{
      setLatencyData(message.data);
      console.log(latencyData)
    });
```

## Monitoramento de Colisões

Utilizamos o tópico /scan para monitorar as distâncias detectadas pelo sensor LiDAR do robô e definir zonas de colisão.

```javascript
var listener = new ROSLIB.Topic({
      ros: ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
  });

    listener.subscribe(function (message) {
        const validRanges = message.ranges.filter(range => !isNaN(range) && range !== null && range !== undefined);

        if (validRanges.length > 0) {
            const front_index = 0;
            const left_index = Math.floor(validRanges.length / 4);
            const back_index = Math.floor(validRanges.length / 2);
            const right_index = Math.floor(validRanges.length * 3 / 4);

            const front_distance = validRanges[front_index];
            const right_distance = validRanges[right_index];
            const back_distance = validRanges[back_index];
            const left_distance = validRanges[left_index];

            setColisaoFrente(front_distance < STOP_DISTANCE);
            setColisaoDireita(right_distance < STOP_DISTANCE);
            setColisaoTras(back_distance < STOP_DISTANCE);
            setColisaoEsquerda(left_distance < STOP_DISTANCE);
        }
    });
```

## Controle de Movimento

Implementamos os eventos de teclado para controlar o movimento do robô, respeitando as zonas de colisão definidas.

```javascript
const handleKeyDown = (event) => {
    let key = event.key;

    switch (key) {
        case 'ArrowUp':
            if (!colisaoFrente){
                turtleBotVel.linear.x = MAX_LIN_VEL;  // Move forward
                turtleBotVel.angular.z = 0;
            } else {
                turtleBotVel.linear.x = 0;  // Stop any forward motion due to collision risk
                turtleBotVel.angular.z = 0;
                console.log("Collision ahead! Stopping.");
            }
            break;
        case 'ArrowDown':
            if (!colisaoTras){
                turtleBotVel.linear.x = MIN_LIN_VEL;  // Move backward
                turtleBotVel.angular.z = 0;
            } else {
                turtleBotVel.linear.x = 0;  // Stop any backward motion due to collision risk
                turtleBotVel.angular.z = 0;
                console.log("Collision behind! Stopping.");
            }
            break;
        case 'ArrowLeft':
            if (!colisaoEsquerda){
                turtleBotVel.angular.z = MAX_ANG_VEL;  // Turn left
                turtleBotVel.linear.x = 0;
            } else {
                turtleBotVel.angular.z = 0;  // Stop any left turn due to collision risk
                turtleBotVel.linear.x = 0;
                console.log("Collision on the left! Stopping.");
            }
            break;
        case 'ArrowRight':
            if (!colisaoDireita){
                turtleBotVel.angular.z = MIN_ANG_VEL;  // Turn right
                turtleBotVel.linear.x = 0;
            } else {
                turtleBotVel.angular.z = 0;  // Stop any right turn due to collision risk
                turtleBotVel.linear.x = 0;
                console.log("Collision on the right! Stopping.");
            }
            break;
        default:
            // If any other key is pressed, don't move the robot
            turtleBotVel.linear.x = 0;
            turtleBotVel.angular.z = 0;
            break;
    }
    
    console.log(turtleBotVel.linear.x, turtleBotVel.angular.z);
    controlTopic.publish(turtleBotVel);
};
```

## Interface do Usuário

A interface é composta por componentes que permitem iniciar a visualização, controlar o robô e visualizar a latência da comunicação.

```javascript
return (
    <div className="principal-container">
      <div className="content-box">
        <PopUpColisao />
        <img id="videoStream" alt="Video Stream" src={videoSrc} className="video-stream" />
        <div className="latency-container">
          <div className="latency-data">
            <p>Latency: {latencyData}</p>
            <p>Status: {latencyData}</p>
          </div>
        </div>
      </div>
      {!isAbaVisible && (
        <><div className="botao-iniciar">
          <BotaoIniciar onClick={() => console.log('Botão Iniciar clicado!')} />
        </div>
        <div className="navigation-buttons">
            <BotoesMover />
          </div></>
      )}
      <BotaoVisualizar onClick={toggleAbaVisualizar} />
      {isAbaVisible && <AbaVisualizar onClose={toggleAbaVisualizar} />}
    </div>
  );
```

## Conclusão

Esta prova de conceito demonstra como é possível integrar ROS com uma aplicação web utilizando React para controlar um robô, visualizar seu vídeo em tempo real e monitorar a latência da comunicação. O código é modular e permite futuras expansões, como a adição de novos sensores ou funcionalidades de controle mais avançadas.
