import React, { useState}  from 'react';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';
import './Form.css';

const ChangePassword = ({ username }) => {
  const [formData, setFormData] = useState({
    currentPassword: '',
    newPassword: '',
    confirmNewPassword: ''
  });
  const [message, setMessage] = useState('');
  const navigate = useNavigate();

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({ ...formData, [name]: value });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (formData.newPassword !== formData.confirmNewPassword) {
      setMessage('New passwords do not match');
      return;
    }

    try {
      const response = await axios.post('http://localhost:3000/change-password', {
        username,
        currentPassword: formData.currentPassword,
        newPassword: formData.newPassword
      });

      if (response.status === 200) {
        setMessage('Password changed successfully');
        navigate('/welcome');
      }
    } catch (error) {
      if (error.response && error.response.data) {
        setMessage(error.response.data);
      } else {
        setMessage('An error occurred. Please try again.');
      }
    }
  };

  const handleGoBack = () => {
    navigate('/welcome');
  };

  return (
    <div className="form-container">
      <h2>Change Password</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="currentPassword">Current Password:</label>
          <input
            type="password"
            id="currentPassword"
            name="currentPassword"
            value={formData.currentPassword}
            onChange={handleChange}
            required
          />
        </div>
        <div className="form-group">
          <label htmlFor="newPassword">New Password:</label>
          <input
            type="password"
            id="newPassword"
            name="newPassword"
            value={formData.newPassword}
            onChange={handleChange}
            required
          />
        </div>
        <div className="form-group">
          <label htmlFor="confirmNewPassword">Confirm New Password:</label>
          <input
            type="password"
            id="confirmNewPassword"
            name="confirmNewPassword"
            value={formData.confirmNewPassword}
            onChange={handleChange}
            required
          />
        </div>
        <div className="button-group">
          <button type="submit">Change Password</button>
          <button type="button" onClick={handleGoBack}>Go Back</button>
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

export default ChangePassword;
