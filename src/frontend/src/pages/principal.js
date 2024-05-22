import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import '../static/principal.css';  // Caminho correto para o CSS

const Principal = () => {
  const [videoSrc, setVideoSrc] = useState('');

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
    <div className="principal-container">
      <div className="content-box">
        <img id="videoStream" alt="Video Stream" src={videoSrc} className="video-stream" />
      </div>
        <button className="view-button">
          <img src={`${process.env.PUBLIC_URL}/visu-botao.png`} alt="cima"/>
        </button>
        <button className="start-button">INICIAR</button>
        <div className="navigation-buttons">
          <button className="nav-button up">
            <img src={`${process.env.PUBLIC_URL}/seta-cima.png`} alt="cima"/>
          </button>
          <button className="nav-button left">
            <img src={`${process.env.PUBLIC_URL}/seta-esquerda.png`} alt="esquerda"/>
          </button>
          <button className="nav-button down">
            <img src={`${process.env.PUBLIC_URL}/seta-baixo.png`} alt="baixo"/>
          </button>
          <button className="nav-button right">
            <img src={`${process.env.PUBLIC_URL}/seta-direita.png`} alt="direita"/>
          </button>
        </div>
      </div>
  );
};

export default Principal;
