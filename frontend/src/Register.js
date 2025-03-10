import React, { useState}  from 'react';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';
import './Form.css';

const Register = ({ onRegister }) => {
  const [formData, setFormData] = useState({
    username: '',
    password: '',
    confirmPassword: ''
  });

  const [message, setMessage] = useState('');
  const navigate = useNavigate();

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({ ...formData, [name]: value });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (formData.password !== formData.confirmPassword) {
      setMessage("Passwords don't match");
      return;
    }

    try {
      const response = await axios.post('http://localhost:3000/addUser', {
        username: formData.username,
        password: formData.password
      });

      if (response.status === 201) {
        const { token, username } = response.data;
        setMessage('User registered successfully');
        localStorage.setItem('authToken', token);
        onRegister(username, token); // Update App state
        navigate('/welcome');
      }
    } catch (error) {
      if (error.response.status === 409) {
        setMessage('Username already exists');
      } else {
        setMessage('An error occurred. Please try again.');
      }
    }
  };

  return (
    <div className="form-container">
      <h2>Register</h2>
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
        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            required
          />
        </div>
        <div className="button-group">
          <button type="submit">Register</button>
          <button type="button" onClick={() => navigate('/login')}>Go to Login</button>
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

export default Register;
