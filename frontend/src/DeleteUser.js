import React from 'react';
import { useNavigate } from 'react-router-dom';
import axios from 'axios';
import './DeleteUser.css'; // Import the CSS file

const DeleteUser = ({ onLogout }) => {
  const navigate = useNavigate();

  const handleDelete = async () => {
    const token = localStorage.getItem('authToken');
    if (!token) {
      console.error('No token found');
      return;
    }

    try {
      const response = await axios.delete('http://localhost:3000/deleteUser', {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });

      if (response.status === 200) {
        localStorage.removeItem('authToken'); // Remove token from localStorage
        onLogout(); // Perform logout
        navigate('/login'); // Navigate to login page
      } else {
        console.error('Failed to delete user');
      }
    } catch (error) {
      console.error('Error deleting user:', error);
    }
  };

  const handleCancel = () => {
    navigate('/welcome');
  };

  return (
    <div className="centered-container">
      <div className="confirmation-box">
        <h1>Are you sure you want to delete your account?</h1>
        <div className="button-group">
          <button onClick={handleDelete}>Yes</button>
          <button onClick={handleCancel}>No</button>
        </div>
      </div>
    </div>
  );
};

export default DeleteUser;
