import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Login from './pages/login';
import Principal from './pages/principal';

const App = () => {
    return (
        <Router>
            <div className="App">
                <Routes>
                    <Route path="/" element={<Login />} /> {/* Rota padrão */}
                    <Route path="/principal" element={<Principal />} />
                </Routes>
            </div>
        </Router>
    );
}

export default App;
