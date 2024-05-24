import React, { useState, useEffect } from 'react';
import { Alert, Space } from 'antd';
import ROSLIB from 'roslib';

export const PopUpColisao = (props) => {
    const LIDAR_RANGE = 0.5;
    const [lidar, setLidar] = useState(0);

    useEffect(() => {
        var ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        var listener = new ROSLIB.Topic({
            ros: ros,
            name: '/scan',
            messageType: 'std_msgs/String'
        });

        listener.subscribe(function (message) {
            console.log('Mensagem recebida do' + listener.name + ': ' + message.data);

            var min_distance = message.data;
            if (min_distance < LIDAR_RANGE) {
                setLidar(message.data);
            }

            listener.unsubscribe();
        });
    }, []);

    return (
        <Space direction="vertical" style={{ width: '50%' }}>
            {lidar < LIDAR_RANGE && (
                <Alert
                    message="Cuidado"
                    description={"O robô está prestes a colidir com um objeto em " + lidar.toString() + " metros."}
                    type="warning"
                    showIcon
                    closable
                />
            )}
        </Space>
    );
}
