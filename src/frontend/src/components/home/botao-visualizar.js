import React from 'react';
// import '../../static/partials/home/botao-visualizar.css'; // Importa o CSS específico para o botão de visualização

const BotaoVisualizar = ({ onClick }) => {
  return (
    <button className="view-button" onClick={onClick}>
      <img src={`${process.env.PUBLIC_URL}/visu-botao.png`} alt="View" className="view-icon" />
    </button>
  );
};

export default BotaoVisualizar;
