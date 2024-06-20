import React, { useState, useEffect } from 'react';
import { Alert, Space } from 'antd';
import ROSLIB from 'roslib';

export const PopUpColisao = (props) => {
    const LIDAR_RANGE = 0.5;
    const [mostrarAlerta, setMostrarAlerta] = useState(false);
    const [valorLidar, setValorLidar] = useState(null);

    // console.log("PopUp Colisão iniciado...");

    useEffect(() => {
        var ros = new ROSLIB.Ros({
            url: 'ws://grupo2.local:9090'
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
                var min_distance = Math.min(...validRanges);

                // console.log('Menor valor do ' + listener.name + ': ' + min_distance);
                if (min_distance < LIDAR_RANGE) {
                    setMostrarAlerta(true);
                    setValorLidar(min_distance);
                } else {
                    setMostrarAlerta(false);
                }
            }
        });

        // Clean up function to close ROS connection
        return () => {
            listener.unsubscribe();
            ros.close();
        };
    }, []);

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
};
