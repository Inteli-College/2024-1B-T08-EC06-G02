import React from 'react';
import '../static/login.css';  // Mantém o caminho correto para o CSS

const Login = () => {
    return (
        <div className="login-container">
            <div className="login-content">
                <div className="logo-container">
                    <img src={`${process.env.PUBLIC_URL}/logo-repipe1.png`} alt="Logo" className="logo" />
                </div>
                <div className="login-box">
                    <form className="login-form">
                        <input type="text" placeholder="Usuário" className="input-field" />
                        <input type="password" placeholder="Senha" className="input-field" />
                        <button type="submit" className="login-button">ENTRAR</button>
                    </form>
                </div>
            </div>
        </div>
    );
}

export default Login;
