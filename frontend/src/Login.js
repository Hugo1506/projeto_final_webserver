import React, { useState}  from 'react';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';
import './Form.css';

const Login = ({ onLogin }) => {
  const [formData, setFormData] = useState({
    username: '',
    password: ''
  });

  const [message, setMessage] = useState('');
  const navigate = useNavigate();

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({ ...formData, [name]: value });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      const response = await axios.post('http://localhost:3000/login', formData);

      if (response.status === 200) {
        const { token } = response.data;
        localStorage.setItem('authToken', token); // Store token in localStorage
        setMessage('Login successful');
        onLogin(formData.username, token); // Pass the token to onLogin
        navigate('/welcome');
      }
    } catch (error) {
    if (error.response) {
    console.log('Error response status:', error.response.status); // Log the status code

    if (error.response.status == 401) {
      setMessage('Invalid username or password');
    } else if (error.response.status == 404) {
      setMessage("User doesn't exist");
    } else {
      setMessage('An error occurred. Please try again.');
    }
  } else {
    console.log('No response property on error object');
    setMessage('An error occurred. Please try again.');
    }  
  }
  };

  return (
    <div className="form-container">
      <h2>Login</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="username">Username:</label>
          <input
            type="text"
            id="username"
            name="username"
            value={formData.username}
            onChange={handleChange}
            required
          />
        </div>
        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
          />
        </div>
        <div className="button-group">
          <button type="submit">Login</button>
          <button type="button" onClick={() => window.location.href = '/register'}>Go to Register</button>
        </div>
        {message && (
          <div className="message-container">
            <p>{message}</p>
          </div>
        )}
      </form>
    </div>
  );
};

export default Login;
