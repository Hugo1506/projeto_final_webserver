import React, { useState,useEffect } from 'react';
import { BrowserRouter as Router, Route, Routes, Navigate } from 'react-router-dom';
import Register from './Register';
import Login from './Login';
import Welcome from './Welcome';
import ChangePassword from './ChangePassword';
import DeleteUser from './DeleteUser';
import {jwtDecode} from 'jwt-decode';

const App = () => {
  const [loggedInUser, setLoggedInUser] = useState(() => {
    const token = localStorage.getItem('authToken');
    if (token) {
      try {
        const decodedToken = jwtDecode(token);
        if (decodedToken.exp * 1000 < Date.now()) {
          localStorage.removeItem('authToken');
          return null;
        }
        return decodedToken.username;
      } catch (error) {
        console.error('Invalid token:', error);
        localStorage.removeItem('authToken');
        return null;
      }
    }
    return null;
  });

  const handleLogin = (username, token) => {
    setLoggedInUser(username);
    localStorage.setItem('authToken', token);
  };

  const handleLogout = () => {
    localStorage.removeItem('authToken');
    setLoggedInUser(null);
  };

  const handleRegister = (username, token) => {
    setLoggedInUser(username);
    localStorage.setItem('authToken', token);
  };

  const handleJWT = () => {
    const token = localStorage.getItem('authToken');
    if (token) {
      try {
        const decodedToken = jwtDecode(token);
        if (decodedToken.exp * 1000 < Date.now()) {
          localStorage.removeItem('authToken');
          setLoggedInUser(null);
        } else {
          setLoggedInUser(decodedToken.username);
        }
      } catch (error) {
        console.error('Invalid token:', error);
        localStorage.removeItem('authToken');
        setLoggedInUser(null);
      }
    }
  };

  useEffect(() => {
    handleJWT();

    const interval = setInterval(() => {
      handleJWT();
    }, 60000);

    return () => clearInterval(interval);
  }, []);

  return (
    <Router>
      <Routes>
        <Route path="/register" element={<Register onRegister={handleRegister} />} />
        <Route path="/login" element={<Login onLogin={handleLogin} />} />
        <Route
          path="/welcome"
          element={loggedInUser ? <Welcome username={loggedInUser} onLogout={handleLogout} /> : <Navigate to="/login" />}
        />
       <Route path="/changepassword" element={<ChangePassword />} />
        <Route path="/deleteUser" element={<DeleteUser />} />
	  <Route path="/" element={<Navigate to={loggedInUser ? '/welcome' : '/login'} />} />
      </Routes>
    </Router>
  );
};

export default App; 
