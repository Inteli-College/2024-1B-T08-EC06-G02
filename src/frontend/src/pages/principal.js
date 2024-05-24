import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import '../static/main.css';  // Caminho correto para o CSS
import BotaoIniciar from '../components/home/botao-iniciar'; // Corrige a importação para minúsculas
import BotaoVisualizar from '../components/home/botao-visualizar'; // Corrige a importação para minúsculas
import BotoesMover from '../components/home/botoes-mover'; // Corrige a importação para minúsculas
import AbaVisualizar from '../components/home/aba-visualizar'; // Caminho correto para o componente

const Principal = () => {
  const [videoSrc, setVideoSrc] = useState('');
  const [isAbaVisible, setIsAbaVisible] = useState(false);

  const toggleAbaVisualizar = () => {
    setIsAbaVisible(!isAbaVisible);
  };

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://10.128.0.17:9090'
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

    const handleVideoFrame = (message) => {
      setVideoSrc('data:image/jpeg;base64,' + message.data);
    };

    videoTopic.subscribe(handleVideoFrame);

    return () => {
      videoTopic.unsubscribe(handleVideoFrame);
      ros.close();
    };
  }, []);

  return (
    <div className={`principal-container ${isAbaVisible ? 'with-aba' : ''}`}>
      <div className="content-box">
        <img id="videoStream" alt="Video Stream" src={videoSrc} className="video-stream" />
      </div>
      <div className="control-buttons">
        <BotaoIniciar onClick={() => console.log('Botão Iniciar clicado!')} />
        <BotoesMover />
      </div>
      <BotaoVisualizar onClick={toggleAbaVisualizar} />
      {isAbaVisible && <AbaVisualizar />}
    </div>
  );
};

export default Principal;
