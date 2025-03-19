import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import './Welcome.css';
import logo from './flyrobotics_logo.png'; // Import the logo

const Welcome = ({ username, onLogout }) => {
  const [dropdownVisible, setDropdownVisible] = useState(false);
  const [fileInputVisible, setFileInputVisible] = useState(false);
  const [files, setFiles] = useState({ cadFiles: null, windFiles: null });
  const [simulationNumber, setSimulationNumber] = useState(null);
  const [fadeOut, setFadeOut] = useState(false);
  const [GadenChoiseVisible, setGadenChoiseVisible] = useState(false);
  const [isNewSimulation, setIsNewSimulation] = useState(false);
	
  const navigate = useNavigate();

  const fetchSimulationNumber = async () => {
    try {
      const response = await fetch(`http://localhost:3000/getSimulationNumber?username=${username}`);
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      const data = await response.json();
      console.log('Received data:', data);
      setSimulationNumber(data.simulationNumber);
    } catch (error) {
      console.error('Error fetching simulation number:', error);
    }
  };

  useEffect(() => {
    fetchSimulationNumber();
  }, [username]);

  const handleDropdownToggle = () => {
    setDropdownVisible(!dropdownVisible);
  };

  const handleChangePassword = () => {
    navigate('/changepassword');
  };

  const handleDeleteUser = () => {
    navigate('/deleteUser');
  };

  const handleGadenClick = () => {
    setFadeOut(true);
    setTimeout(() => {
      setGadenChoiseVisible(true);
      setFadeOut(false);
    }, 500);
  };


  const handleNewSimulationClick = () => {
    setIsNewSimulation(true);
    setGadenChoiseVisible(false); 
  };

  const handleFileChange = (e) => {
    const { name, files } = e.target;
    setFiles((prevFiles) => ({
      ...prevFiles,
      [name]: files,
    }));
  };

  const handleFileSubmit = async (e) => {
    e.preventDefault();

    const formData = new FormData();
    formData.append('username', username);
    formData.append('simulationNumber', simulationNumber);

    if (files.cadFiles) {
      Array.from(files.cadFiles).forEach((file) =>
        formData.append('cadFiles', file)
      );
    }
    if (files.windFiles) {
      Array.from(files.windFiles).forEach((file) =>
        formData.append('windFiles', file)
      );
    }

    try {
      const response = await fetch('http://localhost:3000/uploadFiles', {
        method: 'POST',
        body: formData,
      });

      if (response.ok) {
        alert('Files uploaded successfully.');
        await fetchSimulationNumber(); // Fetch new simulation number after successful upload
      } else {
        alert('Failed to upload files.');
      }
    } catch (error) {
      console.error('Error:', error);
      alert('Error uploading files.');
    }

    setFileInputVisible(false);
  };

  const handleGoBack = () => {
    setFileInputVisible(false);
    setGadenChoiseVisible(false);
    setIsNewSimulation(false);
  };

  const handleGoBackGadenChoise = () => {
    setGadenChoiseVisible(false);
    setIsNewSimulation(false);
  };

  return (
    <div className="welcome-container">
      <div className="welcome-banner">
        <span className="username" onClick={handleDropdownToggle}>
          Hi {username}!
        </span>
        {dropdownVisible && (
          <div className="dropdown-menu">
            <button onClick={handleChangePassword}>Change Password</button>
            <button onClick={handleDeleteUser}>Delete User</button>
            <button onClick={onLogout}>Logout</button>
          </div>
        )}
        <img
          src={logo}
          alt="FlyRobotics Logo"
          className="flyrobotics-logo"
        />
      </div>
      <div className="main-content">
        {!fileInputVisible && !GadenChoiseVisible && !isNewSimulation ? (
          <button
            className={`gaden-button ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGadenClick}
          >
            Gaden
          </button>
        ) : null}

        {GadenChoiseVisible && (
          <div className="gaden-choice-buttons">
            <button className="new-simulation-button" onClick={handleNewSimulationClick}>
              New Simulation
            </button>
            <button className="saved-simulations-button">
              Saved Simulations
            </button>
	    <br />
	    <button className="go-back-gaden-choise" onClick= {handleGoBackGadenChoise}>
		Go Back
	    </button>
          </div>
        )}

        {isNewSimulation && (
          <form onSubmit={handleFileSubmit} className="file-upload-form">
            <div>
              <label htmlFor="cadFiles">CAD Files:</label>
              <input
                type="file"
                id="cadFiles"
                name="cadFiles"
                multiple
                onChange={handleFileChange}
              />
            </div>
            <div>
              <label htmlFor="windFiles">Wind Files:</label>
              <input
                type="file"
                id="windFiles"
                name="windFiles"
                multiple
                onChange={handleFileChange}
              />
            </div>
            <button type="submit" ClassName="submit-button" >Submit</button>
            <button type="button" onClick={handleGoBack} className="go-back-button">
              Go Back
            </button>
          </form>
        )}
      </div>
    </div>
  );
};

export default Welcome;
