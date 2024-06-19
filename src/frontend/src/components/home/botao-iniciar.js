import React from 'react';
// import '../../static/partials/home/botao-iniciar.css';  // Caminho correto para o CSS

const BotaoIniciar = ({ onClick }) => {
  return (
    <button className="botao-font" onClick={onClick}>
      PREVER
    </button>
  );
};

export default BotaoIniciar;
