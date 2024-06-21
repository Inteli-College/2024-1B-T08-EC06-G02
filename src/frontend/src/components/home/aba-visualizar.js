import React, { useState } from 'react';
import '../../static/partials/visualizar/visualizar.css'; 

const AbaVisualizar = ({ onClose }) => {
    const [logs, setLogs] = useState([]);

    const fetchLogs = async () => {
        try {
            const response = await fetch('http://127.0.0.1:8000/getLogs');
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            const data = await response.json();
            const reversedLogs = data.logs.reverse(); // Reverse the array of logs
            setLogs(reversedLogs);
            console.log('Logs fetched:', reversedLogs); // Optionally log the fetched logs
        } catch (error) {
            console.error('Failed to fetch logs:', error);
        }
    };

    return (
        <div className="aba">
            <button className="view-button" onClick={onClose}>
                <img src={`${process.env.PUBLIC_URL}/visu-botao.png`} alt="View" className="view-icon" />
            </button>
            <div className="titulo-grafico">
                <h2>Mapeamento de canos sujos</h2>
            </div>
            {/* <div className="grafico-container">
                <img src={`${process.env.PUBLIC_URL}/grafico-mock.png`} alt="Grafico" className="grafico" />
            </div> */}

            <button className="analise-button" onClick={fetchLogs}> 
                Atualizar
            </button>

            {/* <div className="titulo-status">
                <h2>Status do cano</h2>
            </div>
            <div className="status-container">
                <div className="status">
                    <p>OBSTRUÍDO</p>
                </div>
            </div> */}

            {/* Render fetched logs if available */}
            {logs.length > 0 && (
                <div className="log-container">
                    {logs.map((log, index) => (
                        <div key={index} className="log-entry">
                            <p>Date: {log.Date}</p>
                            <p>Status: {log.Status}</p>
                            <p>Reboiler ID: {log.reboiler_id}</p>
                        </div>
                    ))}
                </div>
            )}
        </div>
    );
}

export default AbaVisualizar;
