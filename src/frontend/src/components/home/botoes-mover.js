import React from 'react';
// import '../../static/partials/home/botoes-mover.css'; // Importa o CSS específico para os botões de navegação

const BotoesMover = () => {
  return (
    <div className="navigation-buttons">
      <button className="nav-button left">
        <img src={`${process.env.PUBLIC_URL}/seta-esquerda.png`} alt="esquerda"/>
      </button>
      <button className="nav-button up">
        <img src={`${process.env.PUBLIC_URL}/seta-cima.png`} alt="cima"/>
      </button>
      <button className="nav-button right">
        <img src={`${process.env.PUBLIC_URL}/seta-direita.png`} alt="direita"/>
      </button>
      <button className="nav-button down">
        <img src={`${process.env.PUBLIC_URL}/seta-baixo.png`} alt="baixo"/>
      </button>
    </div>
  );
};

export default BotoesMover;
