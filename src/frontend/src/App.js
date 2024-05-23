import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Login from './pages/login';
import Principal from './pages/principal';
import Visualizacao from './pages/visualizar';

const App = () => {
    return (
        <Router>
            <div className="App">
                <Routes>
                    <Route path="/" element={<Login />} /> {/* Rota padrão */}
                    <Route path="/principal" element={<Principal />} />
                    <Route path="/visualizacao" element={<Visualizacao />} />
                </Routes>
            </div>
        </Router>
    );
}

export default App;
