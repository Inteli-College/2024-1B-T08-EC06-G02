# Prova de Conceito: Monitoramento de Colisões com LiDAR e Botão de Emergência

## Introdução

Este documento apresenta a prova de conceito para o monitoramento de colisões de um robô utilizando um sensor LiDAR e a implementação de um botão de emergência. O objetivo é demonstrar como o sistema detecta possíveis colisões em diferentes direções e como a funcionalidade do botão de emergência interrompe imediatamente o movimento do robô.

## Estrutura do Código

O código foi desenvolvido utilizando a biblioteca ROSLIB para a comunicação com o ROS e a biblioteca React para a criação da interface do usuário. A seguir, detalhamos os componentes e funcionalidades principais relacionados ao monitoramento de colisões e ao botão de emergência.

## Importações e Inicializações

Os componentes e bibliotecas necessários são importados, e os estados são inicializados para monitorar colisões e gerenciar a interface do usuário.

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

Os estados são utilizados para gerenciar a interface do usuário e monitorar colisões. Constantes são definidas para os limites de velocidade linear e angular do robô.

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

Estabelecemos a conexão com o ROS utilizando WebSocket e configuramos os tópicos para controle e monitoramento de colisões.

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

    const controlTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

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

    // Definição da mensagem de controle de velocidade do robô
    let turtleBotVel = new ROSLIB.Message({
      linear: {
        x: 0,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: 0
      }
    });

    // Implementação do botão de emergência
    const handleEmergencyStop = () => {
        turtleBotVel.linear.x = 0;
        turtleBotVel.angular.z = 0;
        controlTopic.publish(turtleBotVel);
        console.log("Emergency stop activated!");
    };

    // Eventos de teclado para controle do robô
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
            case ' ':
                handleEmergencyStop();  // Ativar o botão de emergência
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

    const handleKeyUp = (event) => {
        let key = event.key;

        switch (key) {
            case 'ArrowUp':
            case 'ArrowDown':
                turtleBotVel.linear.x = 0;
                break;
            case 'ArrowLeft':
            case 'ArrowRight':
                turtleBotVel.angular.z = 0;
                break;
            default:
                break;
        }

        console.log(turtleBotVel.linear.x, turtleBotVel.angular.z);
        controlTopic.publish(turtleBotVel);
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
        window.removeEventListener('keydown', handleKeyDown);
        window.removeEventListener('keyup', handleKeyUp);
        listener.unsubscribe();
        ros.close();
    };
}, [colisaoDireita, colisaoEsquerda, colisaoFrente, colisaoTras, MAX_ANG_VEL, MAX_LIN_VEL, MIN_ANG_VEL, MIN_LIN_VEL]);
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
        <>
          <div className="botao-iniciar">
            <BotaoIniciar onClick={() => console.log('Botão Iniciar clicado!')} />
          </div>
          <div className="navigation-buttons">
            <BotoesMover />
          </div>
        </>
      )}
      <BotaoVisualizar onClick={toggleAbaVisualizar} />
      {isAbaVisible && <AbaVisualizar onClose={toggleAbaVisualizar} />}
    </div>
);
```

## Conclusão

Esta prova de conceito demonstra como o monitoramento de colisões pode ser implementado utilizando um sensor LiDAR, detectando obstáculos em diferentes direções e impedindo o movimento do robô em caso de risco de colisão. Além disso, a implementação de um botão de emergência permite interromper imediatamente o movimento do robô em situações críticas, aumentando a segurança do sistema. O código é modular e pode ser facilmente expandido para incluir funcionalidades adicionais de monitoramento e controle.