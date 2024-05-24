import React from 'react';
import '../../static/partials/visualizar/visualizar.css'; 

const AbaVisualizar = ({ onClose }) => {
    return (
        <div className="aba">
            <button className="view-button" onClick={onClose}>
                <img src={`${process.env.PUBLIC_URL}/visu-botao.png`} alt="View" className="view-icon" />
            </button>
            <div className="titulo-grafico">
                <h2>Mapeamento de canos sujos</h2>
            </div>
            <div className="grafico-container">
                <img src={`${process.env.PUBLIC_URL}/grafico-mock.png`} alt="Grafico" className="grafico" />
            </div>
            <button className="analise-button"> 
                Clique para mais visualizações
            </button>
            <div className="titulo-status">
                <h2>Status do cano</h2>
            </div>
            <div className="status-container">
                <div className="status">
                    <p>OBSTRUÍDO</p>
                </div>
            </div>
        </div>
    );
}

export default AbaVisualizar;
