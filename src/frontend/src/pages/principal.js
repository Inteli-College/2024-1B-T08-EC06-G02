import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import '../static/main.css';  // Caminho correto para o CSS
import BotaoIniciar from '../components/home/botao-iniciar'; // Corrige a importação para minúsculas
import BotaoVisualizar from '../components/home/botao-visualizar'; // Corrige a importação para minúsculas
import BotoesMover from '../components/home/botoes-mover'; // Corrige a importação para minúsculas
import { PopUpColisao } from '../components/home/popup-colisao';
import AbaVisualizar from '../components/home/aba-visualizar';

const Principal = () => {
  const [videoSrc, setVideoSrc] = useState('');
  const [isAbaVisible, setIsAbaVisible] = useState(false);
  const [colisaoFrente, setColisaoFrente] = useState(false);
  const [colisaoTras, setColisaoTras] = useState(false);
  const [colisaoEsquerda, setColisaoEsquerda] = useState(false);
  const [colisaoDireita, setColisaoDireita] = useState(false);
  const STOP_DISTANCE = 0.5; 

  const MAX_LIN_VEL = 0.21 // m/s
  const MIN_LIN_VEL = -0.21 // m/s
  const MAX_ANG_VEL = 2.63 // rad/s
  const MIN_ANG_VEL = -2.63 // rad/s

  const toggleAbaVisualizar = () => {
    setIsAbaVisible(!isAbaVisible);
  };

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

    var listener = new ROSLIB.Topic({
      ros: ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
  });

    listener.subscribe(function (message) {
        // console.log('Mensagem recebida do ' + listener.name + ': ' + message.ranges);

        const validRanges = message.ranges.filter(range => !isNaN(range) && range !== null && range !== undefined);

        if (validRanges.length > 0) {
          if (validRanges.length > 0) {
            const front_index = 0;
            const left_index = Math.floor(validRanges.length / 4);
            const back_index = Math.floor(validRanges.length / 2);
            const right_index = Math.floor(validRanges.length * 3 / 4);
    
            const front_distance = validRanges[front_index];
            const right_distance = validRanges[right_index];
            const back_distance = validRanges[back_index];
            const left_distance = validRanges[left_index];
    
            // Log distances for debugging
            // console.log(`Front distance: ${front_distance}, Left distance: ${left_distance}, Back distance: ${back_distance}, Right distance: ${right_distance}`);
    
            setColisaoFrente(front_distance < STOP_DISTANCE);
            setColisaoDireita(right_distance < STOP_DISTANCE);
            setColisaoTras(back_distance < STOP_DISTANCE);
            setColisaoEsquerda(left_distance < STOP_DISTANCE);
        }
        }
    });

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

    const handleVideoFrame = (message) => {
      // console.log(message.data);  // debug
      setVideoSrc('data:image/jpeg;base64,' + message.data);
    };
    

    videoTopic.subscribe(handleVideoFrame);
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      videoTopic.unsubscribe(handleVideoFrame);
      ros.close();
    };
  }, [colisaoDireita, colisaoEsquerda, colisaoFrente, colisaoTras, MAX_ANG_VEL, MAX_LIN_VEL, MIN_ANG_VEL, MIN_LIN_VEL]);

  return (
    <div className={`principal-container ${isAbaVisible ? 'with-aba' : ''}`}>
      <div className="content-box">
        <PopUpColisao />
        <img id="videoStream" alt="Video Stream" src={videoSrc} className="video-stream" />
      </div>
      <div className={`control-buttons ${isAbaVisible ? 'with-aba' : ''}`}>
        <BotaoIniciar onClick={() => console.log('Botão Iniciar clicado!')} />
      </div>
      <div className={`navigation-buttons ${isAbaVisible ? 'with-aba' : ''}`}>
        <BotoesMover />
      </div>
      <BotaoVisualizar onClick={toggleAbaVisualizar} />
      {isAbaVisible && <AbaVisualizar onClose={toggleAbaVisualizar} />}
    </div>
  );
};

export default Principal;
