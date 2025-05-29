import React, { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import './Welcome.css';
import logo from './flyrobotics_logo.png'; 

const Welcome = ({ username, onLogout }) => {
  const [dropdownVisible, setDropdownVisible] = useState(false);
  const [fileInputVisible, setFileInputVisible] = useState(false);
  const [files, setFiles] = useState({ innerCadFiles: null, outerCadFiles: null, windFiles: null });
  const [simulationNumber, setSimulationNumber] = useState(null);
  const [fadeOut, setFadeOut] = useState(false);
  const [GadenChoiseVisible, setGadenChoiseVisible] = useState(false);
  const [savedSimulationsVisible, setSavedSimulationsVisible] = useState(false);
  const [isNewSimulation, setIsNewSimulation] = useState(false);
	const [simulationName, setSimulationName] = useState('');
  const [plumeXLocation, setPlumeXLocation] = useState('');
  const [plumeYLocation, setPlumeYLocation] = useState('');
  const [plumeZLocation, setPlumeZLocation] = useState('');
  const [savedSimulations, setSavedSimulations] = useState([]);
  const [gadenSimulationClickVisible, setGadenSimulationClickVisible] = useState(false);
  const [gifs, setGifs] = useState([]);
  const [loadingGifs, setLoadingGifs] = useState(true);
  const [searchQuery, setSearchQuery] = useState('');
  const [filteredSimulations, setFilteredSimulations] = useState(savedSimulations);
  const [simulationDetail, setSimulationDetail] = useState(false);
  const [simulationStatus, setSimulationStatus] = useState('');
  const [showModal, setShowModal] = useState(false);
  const [simulationToDelete, setSimulationToDelete] = useState(null);
  const [clickedGif, setClickedGif] = useState(null);
  const [robotXlocation, setRobotXLocation] = useState('');
  const [robotYlocation, setRobotYLocation] = useState('');
  const [height, setHeight] = useState('');
  const [robotSpeed, setRobotSpeed] = useState('');
  const [activeButton, setActiveButton] = useState('gaden');
  const [showCheckboxes, setShowCheckboxes] = useState(true);
  const [robotSimulation, setRobotSimulation] = useState(true);
  const [simulationBounds, setSimulationBounds] = useState(null);
  const [showPlumeLocation,setShowPlumeLocation] = useState(false);
  const [ambientSimulator, setAmbientSimulator] = useState("");
  const navigate = useNavigate();
  const searchInputRef = useRef(null);
  const [selectedHeight, setSelectedHeight] = useState('');
  const [availableHeights, setAvailableHeights] = useState([]);
  const [plumeSimulationIsLoading, setPlumeSimulationIsLoading] = useState(false);
  const [plumeSimulationLoadingText, setPlumeSimulationLoadingText] = useState("Waiting for Simulation Results");
  const [robotSimulationIsLoading, setRobotSimulationIsLoading] = useState(false);
  const [robotSimulationLoadingText, setRobotSimulationLoadingText] = useState("Waiting for Robot Simulation Results");
  const [currentIteration, setCurrentIteration] = useState(0);
  const [maxIteration, setMaxIteration] = useState(0);


  const [checkedOptions, setCheckedOptions] = useState({
    all: false,
    heatmap: false,
    wind: false,
    contour: false,
  });

  const filteredGifs = (activeButton === 'robot')
    ? gifs.filter((gif) => gif.type === 'robot')
    : checkedOptions.all
    ? gifs
    : gifs
        .filter((gif) => checkedOptions[gif.type])
  
  // verifica qual é a iteração máxima 
  useEffect(() => {
    if (filteredGifs.length > 0) {
      const max = Math.max(...filteredGifs.map(g => g.iteration));
      setMaxIteration(max);
      console.log("max:",maxIteration)
    }
  }, [filteredGifs]);

  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentIteration(prev => (prev + 1) > maxIteration ? 0 : prev + 1);
      console.log("Current Iteration:", currentIteration);
    }, 500); // 500ms 

    return () => clearInterval(interval);
  }, [maxIteration]);

  useEffect(() => {
    if (!plumeSimulationIsLoading) return; // não corre se não estiver à espera dos resultados da simulação 

    let dotCount = 0;
    const interval = setInterval(() => {
      dotCount = (dotCount + 1) % 4; // faz reset da contagem quando chega a 4 
      setPlumeSimulationLoadingText(`Waiting for simulation Results${".".repeat(dotCount)}`);
    }, 500);

    return () => clearInterval(interval);
  }, [plumeSimulationIsLoading]);

  useEffect(() => {
    if (!robotSimulationIsLoading) return; // não corre se não estiver à espera dos resultados da simulação 

    let dotCount = 0;
    const interval = setInterval(() => {
      dotCount = (dotCount + 1) % 4; // faz reset da contagem quando chega a 4 
      setRobotSimulationLoadingText(`Waiting for simulation Results${".".repeat(dotCount)}`);
    }, 500);

    return () => clearInterval(interval);
  }, [robotSimulationIsLoading]);


  // modal que para confirmar algo
  const ConfirmationModal = ({ message, onConfirm, onCancel }) => {
    return (
      <div className="modal-overlay">
        <div className="modal">
          <p>{message}</p>
          <div className="modal-buttons">
            <button onClick={onConfirm}>YES</button>
            <button onClick={onCancel}>NO</button>
          </div>
        </div>
      </div>
    );
  };




  const fetchSimulationStatus = async (simulation) => {
    try {
      const response = await fetch(`http://localhost:3000/getSimulationStatus?simulation=${simulation}`);
      if (!response.ok) {
        throw new Error('Failed to fetch simulation status');
      }
      const data = await response.json();
      return data.status; 
    } catch (error) {
      console.error('Error fetching simulation status:', error);
      return null;
    }
  };
  

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

  const fetchSavedSimulations = async () => {
    try {
      const response = await fetch(`http://localhost:3000/getSimulations?username=${username}`);
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      const data = await response.json();
      setSavedSimulations(data.simulations);
      setFilteredSimulations(data.simulations);
    } catch (error) {
      console.error('Error fetching saved simulations:', error);
    }
  };

  const fetchBoundsStatus = async (simulation) => {
    try {
      const response = await fetch(`http://localhost:3000/getBoundsValues?simulation=${(simulation)}`);
      
      if (!response.ok) {
        if (response.status === 404) {
          console.warn('Simulation not found');
          return null;
        }
        throw new Error('Failed to fetch simulation bounds');
      }
  
      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error fetching bounds status:', error);
      return null;
    }
  };

  const fetchGifsFromResults = async (simulation) => {
    try {
      setLoadingGifs(true);
      const response = await fetch(`http://localhost:3000/getSimulationResultsGifs?simulation=${simulation}`);
      
      if (!response.ok) {
        throw new Error('Failed to fetch GIFs');
      }
  
      const gifsData = await response.json();
      
      const gifs = gifsData.map(({ gif, height,type,iteration,robot_path }) => {
        const byteCharacters = atob(gif);  
        const byteArrays = [];
  
        for (let offset = 0; offset < byteCharacters.length; offset += 1024) {
          const byteArray = new Uint8Array(Math.min(byteCharacters.length - offset, 1024));
          for (let i = 0; i < byteArray.length; i++) {
            byteArray[i] = byteCharacters.charCodeAt(offset + i);
          }
          byteArrays.push(byteArray);
        }
  
        const blob = new Blob(byteArrays, { type: 'image/gif' });
        const url = URL.createObjectURL(blob);

        return {
          url,
          height,
          type,
          simulation,
          iteration,
          robot_path: (() => {
          try {
            return typeof robot_path === 'string' ? JSON.parse(robot_path) : robot_path;
          } catch (e) {
            console.warn("Failed to parse robot_path:", e);
            return [];
          }
        })()
      };
      });
      

      const uniqueHeights = [...new Set(gifs.map(gif => gif.height))];

      setSelectedHeight(""); 

      setAvailableHeights(uniqueHeights);

      const gifTypes = gifs.map(g => g.type);

      setCheckedOptions({
        all: true,
        heatmap: gifTypes.includes('heatmap'),
        wind: gifTypes.includes('wind'),
        contour: gifTypes.includes('contour'),
        [selectedHeight]: true,
      });

      

      setGifs(gifs);
    } catch (error) {
      console.error('Error fetching GIFs:', error);
    } finally {
      setLoadingGifs(false); 
    }
  };


  useEffect(() => {
    fetchSimulationNumber();
  }, [username]);

  // foca a search box 
  useEffect(() => {
    if (savedSimulationsVisible && searchInputRef.current) {
      searchInputRef.current.focus();
    }
  }, [savedSimulationsVisible]);

  const handleCheckboxChange = (event) => {
    const { name, checked } = event.target;
  
    setCheckedOptions((prevState) => {
      if (name === 'all') {
        return {
          all: checked,
          heatmap: checked,
          wind: checked,
          contour: checked,
        };
      } else {
        const newState = {
          ...prevState,
          [name]: checked,
        };
  
        const allSelected = newState.heatmap && newState.wind && newState.contour;
        return {
          ...newState,
          all: allSelected,
        };
      }
    });
  };
  

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
      setAmbientSimulator("Simulatior: Gaden version 2.5.0");
      setFadeOut(false);
    }, 500);
  };


  const handleNewSimulationClick = () => {
    setFadeOut(true); 
    setTimeout(() => {
      setGadenChoiseVisible(false);
      setIsNewSimulation(true);
      setFadeOut(false);
      setSimulationName('');
      setPlumeXLocation('');
      setPlumeYLocation('');
      setPlumeZLocation(''); 
    }, 500);
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
    formData.append('simulationName', simulationName);

    if (files.innerCadFiles) {
      Array.from(files.innerCadFiles).forEach((file) => {
        formData.append('innerCadFiles', file);
      });
    }
  
    if (files.outerCadFiles) {
      Array.from(files.outerCadFiles).forEach((file) => {
        formData.append('outerCadFiles', file);
      });
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
        const simulation = username + "_" + simulationNumber;
  
        let status = "";
        while (true) {
          status = await fetchSimulationStatus(simulation);
  
          if (status === "IN_QUEUE") {
            alert("Simulation is in queue...");
          } else if (status === "IN SIMULATION") {
            alert("Simulation has started.");
            const bounds = await fetchBoundsStatus(simulation);
            setSimulationBounds(bounds);
            setShowPlumeLocation(true);
            break;
          }
  
          await new Promise(resolve => setTimeout(resolve, 3000));
        }
  
        await fetchSimulationNumber();
      } else {
        alert('Failed to upload files.');
      }
    } catch (error) {
      console.error('Error:', error);
      alert('Error uploading files.');
    }

    setFileInputVisible(false);
  };

  const handlePlumeSubmit = async (e) => {
    e.preventDefault();
    setPlumeSimulationIsLoading(true);

    const simulationCorrected = simulationNumber - 1;
    const formData = {
      username,
      simulationNumber: simulationCorrected,
      plumeXlocation: plumeXLocation,
      plumeYlocation: plumeYLocation,
      plumeZlocation: plumeZLocation,
    };

    
    console.log('Form data:', formData);
    try {
      const response = await fetch('http://localhost:3000/uploadPlumeLocation', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });
    } catch (error) {
      console.error('Error:', error);
    } finally{
      await fetchSavedSimulations();
      setPlumeSimulationIsLoading(false);
      setIsNewSimulation(false);
      setShowPlumeLocation(false);
      setSavedSimulationsVisible(true);
    }
  };

  const handleGoBack = () => {
    setFadeOut(true); 
    setTimeout(() => {
      setFileInputVisible(false);
      setGadenChoiseVisible(false);
      setIsNewSimulation(false);
      setSavedSimulationsVisible(false);
      setAmbientSimulator("");
      setFadeOut(false); 
    }, 500);
  };

  const handleGoBackGadenChoise = () => {
    setFadeOut(true); 
    setTimeout(() => {
      setGadenChoiseVisible(true);
      setIsNewSimulation(false);
      setSavedSimulationsVisible(false);
      setShowPlumeLocation(false);
      setFadeOut(false); 
    }, 500);
 
  };

  const handleSavedSimulationsClick = async () => {
    await fetchSavedSimulations();
    setFadeOut(true); 
    setTimeout(() => {
      setGadenChoiseVisible(false); 
      setIsNewSimulation(false); 
      setSavedSimulationsVisible(true);
      setFadeOut(false)
    }, 500);
  };


  const handleSimulationClick = async (simulation) => {
    setGifs([]);
    await fetchGifsFromResults(simulation);
  
    setFadeOut(true);
  
    setTimeout(() => {
      setSavedSimulationsVisible(false);
      setSimulationDetail(true);
      setFadeOut(false);
  
      let previousGifs = [];
  
      const intervalId = setInterval(async () => {
        try {
          const status = await fetchSimulationStatus(simulation);
          
          if (status === "DONE") {
            setSimulationStatus("Done");
            clearInterval(intervalId);
            return;
          } else {
            setSimulationStatus("Simulation still running");
          }
    
          const newGifs = await fetchGifsFromResults(simulation);
  
          if (JSON.stringify(newGifs) !== JSON.stringify(previousGifs)) {
            previousGifs = [...newGifs];
            setGifs(newGifs);
          }
          
        } catch (error) {
          console.error('Error fetching simulation status or GIFs:', error);
        }
      }, 10000);
  
      return () => clearInterval(intervalId);
    }, 500);
  };
  
  

  const handleGoBackSavedSimulations = () => {
    setFadeOut(true);
    setTimeout(() => {  
      setSavedSimulationsVisible(true);
      setSimulationDetail(false);
      setFadeOut(false)
      setSearchQuery('');
      setFilteredSimulations(savedSimulations);
    }, 500);
  } 

  const handleGifClick = (gifObj) => {
    setSimulationDetail(false);
    setGadenSimulationClickVisible(true);
    setClickedGif(gifObj);
    setHeight(gifObj.height);

  }

  const handleSearch = (event) => {
    const query = event.target.value.toLowerCase();
    setSearchQuery(query);
  
    if (query === '') {
      setFilteredSimulations(savedSimulations); 
      return;
    }
  
    const filtered = savedSimulations.filter((simulation) => {
      const simulationName = simulation.simulationName ? simulation.simulationName.toLowerCase() : '';
      const simulationDescription = simulation.simulation ? simulation.simulation.toLowerCase() : '';
  
      return (
        simulationName.includes(query) || 
        simulationDescription.includes(query)
      );
    });
  
    setFilteredSimulations(filtered);
  };

  const openDeleteModal = (simulation) => {
    setSimulationToDelete(simulation);
    setShowModal(true);
  };

  const closeModal = () => {
    setShowModal(false);
    setSimulationToDelete(null);
  };

  const confirmDelete = async () => {
    try {
      const response = await fetch(`http://localhost:3000/deleteSimulation?simulation=${simulationToDelete.simulation}`, {
        method: 'POST',
      });

      if (!response.ok) {
        const errorMessage = await response.text();
        alert(`Error: ${errorMessage}`);
      }
      closeModal();
      await fetchSavedSimulations();
    } catch (error) {
      console.error('Error deleting simulation:', error);
      alert('Failed to delete simulation');
    }   
  };

  const handleGoBackSimulationDetails = () => {
    setFadeOut(true);
    setTimeout(() => {
      setGadenSimulationClickVisible(false);
      setSimulationDetail(true);
      setFadeOut(false);
    }, 500);
  }


  const handleRobotSimulationSubmit = async (e,simulation) => {
    e.preventDefault();
    setRobotSimulationIsLoading(true);
    const formData = {
      username,
      simulation,
      height,
      robotSpeed,
      robotXposition: robotXlocation,
      robotYposition: robotYlocation
    };
  
    try {
      const response = await fetch('http://localhost:3000/robotSimulation', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });
  
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
  
      const result = await response.json();
      console.log('Simulation result:', result);
    } catch (error) {
      console.error('Error during simulation:', error);
    } finally {
      setRobotSimulationIsLoading(false);
      setGadenSimulationClickVisible(false);
      setSimulationDetail(true);
      handleToggleButton('robot');
      await handleSimulationClick(simulation);
    }
  }

  const handleToggleButton = (button) => {
    setActiveButton(button);
    if (button === 'gaden') {
      setActiveButton('gaden');
      setShowCheckboxes(true);
      setRobotSimulation(true);
    } else {
      setActiveButton('robot');
      setShowCheckboxes(false);
      setRobotSimulation(false);
    }
  }


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
        {!gadenSimulationClickVisible && !fileInputVisible && !GadenChoiseVisible && !isNewSimulation && !savedSimulationsVisible && !simulationDetail? (
          <button
            className={`gaden-button ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGadenClick}
          >
            Gaden <br /> version: 2.5.0
          </button>
        ) : null}
       {GadenChoiseVisible && (
         <div className="gaden-choice-buttons">
          <h1 className={'info-header'}>{ambientSimulator}</h1>
          <button
           className={`new-simulation-button ${fadeOut ? 'fade-out' : ''}`}
           onClick={handleNewSimulationClick}
          >
            New Simulation
          </button>
          <button
             className={`saved-simulations-button ${fadeOut ? 'fade-out' : ''}`}
             onClick={handleSavedSimulationsClick} 
          >
              Saved Simulations
           </button>
           <button
                className={`go-back-button ${fadeOut ? 'fade-out' : ''}`}
                onClick={handleGoBack}
              >
                Go Back
            </button>
           <br />
  	 </div>
    )}
      {isNewSimulation && !showPlumeLocation && (
        <form onSubmit={handleFileSubmit} className="file-upload-form">
          <div>
            <label htmlFor="simulationName">Name of the simulation (optional):</label>
            <input
              type="text"
              id="simulationName"
              name="simulationName"
              value={simulationName}
              onChange={(e) => setSimulationName(e.target.value)}
              placeholder="Enter simulation name"
            />
          </div>
          <div>
            <label htmlFor="innerCadFiles">Inner CAD Files:</label>
            <input
              type="file"
              id="innerCadFiles"
              name="innerCadFiles"
              multiple
              onChange={handleFileChange}
            />
          </div>
          <div>
            <label htmlFor="outerCadFiles">Outer CAD Files:</label>
            <input
              type="file"
              id="outerCadFiles"
              name="outerCadFiles"
              multiple
              onChange={handleFileChange}
            />
          </div>
          <div>
            <label htmlFor="windFiles">Wind Files:</label>
            <input 
            type="file" 
            multiple webkitdirectory="true" 
            directory="" 
            onChange={(e) => setFiles({ ...files, windFiles: e.target.files })} 
            />
          </div>
          <button type="submit" className={`submit-button ${fadeOut ? 'fade-out' : ''}`} >Submit</button>
          <button type="button" onClick={handleGoBackGadenChoise} className={`go-back-button ${fadeOut ? 'fade-out' : ''}`}>
            Go Back
          </button>
        </form> 
      )}
      {isNewSimulation && showPlumeLocation && (
        <div style={{ position: "relative" }}>
          <form onSubmit={handlePlumeSubmit} className="file-upload-form">
            <div>
              <h4 htmlFor="plumeXLocation">Plume location </h4>
              <label htmlFor="plumeXLocation">Range: {simulationBounds.xMin} and {simulationBounds.xMax}</label>
              <input
                type="float"
                id="plumeXLocation"
                name="plumeXLocation"
                value={plumeXLocation}
                onChange={(e) => setPlumeXLocation(e.target.value)}
                placeholder="Enter plume location x value X.X"
              />
              <label htmlFor="plumeYLocation">Range: {simulationBounds.yMin} and {simulationBounds.yMax}</label>
              <input
                type="float"
                id="plumeYLocation"
                name="plumeYLocation"
                value={plumeYLocation}
                onChange={(e) => setPlumeYLocation(e.target.value)}
                placeholder="Enter plume location y value X.X"
              />
              <label htmlFor="plumeZLocation">Range: {simulationBounds.zMin} and {simulationBounds.zMax} </label>
              <input
                type="float"
                id="plumeZLocation"
                name="plumeZLocation"
                value={plumeZLocation}
                onChange={(e) => setPlumeZLocation(e.target.value)}
                placeholder="Enter plume location z value X.X"
              />
            </div>
            <button type="submit" className={`submit-button ${fadeOut ? 'fade-out' : ''}`} >Submit</button>
            <button type="button" onClick={handleGoBackGadenChoise} className={`go-back-button ${fadeOut ? 'fade-out' : ''}`}>
              Go Back
            </button>
          </form>
          {plumeSimulationIsLoading && (
        <div className="popup-overlay">
            <div className="popup">
              <p>{plumeSimulationLoadingText}</p>
        </div>
      </div>
    )}
    </div>
      )}
        {savedSimulationsVisible && (
          <div className="saved-simulations-list" >
            <div className="saved-simulations-header">
              <h3>Saved Simulations</h3>
              <input
                ref={searchInputRef}
                type="text"
                className="search-bar"
                placeholder="Search by name or simulation"
                onChange={handleSearch}
              />
              <button
                className={`go-back-saved-simulations ${fadeOut ? 'fade-out' : ''}`}
                onClick={handleGoBackGadenChoise}
              >
                Go Back
            </button>
            </div>
            <div className="saved-simulations-list">
              {filteredSimulations.length > 0 ? (
                <ul>
                {filteredSimulations.map((simulation, index) => (
                  <li
                    key={index}
                    className={`simulation-item ${fadeOut ? 'fade-out' : ''}`}
                  >
                    <div className="simulation-content">
                      <div
                        className="simulation-details"
                        onClick={() => handleSimulationClick(simulation.simulation)}
                      >
                        <strong>Simulation Name:</strong> {simulation.simulationName || 'Unnamed Simulation'} <br />
                        <strong>Simulation:</strong> {simulation.simulation || 'No simulation description'}
                      </div>
                      <div
                        className="trash-icon"
                        onClick={(e) => {
                          e.stopPropagation();
                          openDeleteModal(simulation);
                        }}
                      >
                        🗑️
                      </div>
                    </div>
                  </li>
                ))}
              </ul>
              
              ) : (
                <p>No simulations found</p>
              )}
              {showModal && (
                <ConfirmationModal
                  message="Are you sure you want to delete this simulation?"
                  onConfirm={confirmDelete}
                  onCancel={closeModal}
                />
              )}
            </div>
          </div>
        )}
        {simulationDetail && (
        <>
          <div className="control-simulation-details">
            <button
              className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
              onClick={handleGoBackSavedSimulations}
            >
              Go Back
            </button>
            <div className="toggle-buttons">
              <button
                className={`toggle-button ${activeButton === 'gaden' ? 'active' : 'inactive'}`}
                onClick={() => handleToggleButton('gaden')}
              >
                Gaden Simulations
              </button>
              <button
                className={`toggle-button ${activeButton === 'robot' ? 'active' : 'inactive'}`}
                onClick={() => handleToggleButton('robot')}
              >
                Robot Simulations
              </button>
            </div>
            {showCheckboxes && (
              <div className="checkbox-filters">
                <label>
                  <input
                    type="checkbox"
                    name="all"
                    checked={checkedOptions.all}
                    onChange={handleCheckboxChange}
                  />
                  Show all simulations
                </label>

                <label>
                  <input
                    type="checkbox"
                    name="heatmap"
                    checked={checkedOptions.heatmap}
                    onChange={handleCheckboxChange}
                  />
                  Show heatmap
                </label>

                <label>
                  <input
                    type="checkbox"
                    name="wind"
                    checked={checkedOptions.wind}
                    onChange={handleCheckboxChange}
                  />
                  Show wind vectors
                </label>

                <label>
                  <input
                    type="checkbox"
                    name="contour"
                    checked={checkedOptions.contour}
                    onChange={handleCheckboxChange}
                  />
                  Show contour
                </label>
              
                <select value={selectedHeight} onChange={(e) => {
                  console.log('Selected Height:', e.target.value);
                  setSelectedHeight(e.target.value);
                }}>
                  <option value="">All heights</option>
                  {availableHeights.map((height, idx) => (
                    <option key={idx} value={height}>
                      {height ?? 'Unknown'}
                    </option>
                  ))}
                </select>   
              </div> 
          )}
          </div>
          <div className="gif-container">

            {!loadingGifs && filteredGifs.length === 0 && (
              <p>No GIFs available for the selected options.</p>
            )}

            {filteredGifs.length > 0 && (
              
              filteredGifs
                .slice()
                .sort((a, b) => {
                  const heightDiff = Number(a.height) - Number(b.height);
                  if (heightDiff !== 0) return heightDiff;
                  return a.type.localeCompare(b.type);
                })
                .filter((gifObj) => {
                  if (activeButton === 'gaden') {
                    return ['heatmap', 'wind', 'contour'].includes(gifObj.type); 
                  }
                  if (activeButton === 'robot') {
                    return gifObj.type === 'robot';  
                  }
                  return true;
                })
                .filter((gifObj) => {
                  if (!selectedHeight) {
                    return true;
                  }
                  return gifObj.height == selectedHeight;
                })

                .filter((gifObj) => gifObj.iteration === currentIteration)
                
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">
                    <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                    <img
                      src={gifObj.url}
                      alt={`Simulation GIF ${index + 1}`}
                      className="gif-image"
                      onClick={() => {handleGifClick(gifObj)}}
                      style={{ cursor: 'pointer' }}
                    />
                  </div>
                ))
            )}
          </div>

      </>
      )}
      </div>
      {gadenSimulationClickVisible && clickedGif && robotSimulation && (
        <div className="gaden-simulation-click">
          <button
            className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGoBackSimulationDetails}
          >
            Go Back
          </button>
          <div className="side-by-side-container">
            <div className="gif-description-robot">
              <h3>Height: {clickedGif.height ?? 'Unknown'}</h3>
              <img
                src={clickedGif.url}
                className="gif-image"
              />
            </div>

            <form onSubmit={(e) => handleRobotSimulationSubmit(e, clickedGif.simulation)} className="robot-simulation-form">              <div className="robot-simulation-inputs">
                <label>Robot Speed in meters </label>
                <input
                  type="float"
                  id="robotSpeed"
                  name="robotSpeed"
                  value={robotSpeed}
                  onChange={(e) => setRobotSpeed(e.target.value)}
                  placeholder="Enter a robot speed value X.X"
                />
                <label>Robot X coordinate </label>
                <input
                  type="float"
                  id="robotXlocation"
                  name="robotXlocation"
                  value={robotXlocation}
                  onChange={(e) => setRobotXLocation(e.target.value)}
                  placeholder="Enter a robot X location value X.X"
                />
                <label>Robot Y coordinate </label>
                <input
                  type="float"
                  id="robotYlocation"
                  name="robotYlocation"
                  value={robotYlocation}
                  onChange={(e) => setRobotYLocation(e.target.value)}
                  placeholder="Enter a robot Y location value X.X"
                />
                <button type="submit" className={`submit-button ${fadeOut ? 'fade-out' : ''}`} >Submit</button>
              </div>
            </form>
          </div>
          {robotSimulationIsLoading && (
        <div className="popup-overlay">
            <div className="popup">
              <p>{robotSimulationLoadingText}</p>
        </div>
        </div>
      )}
        </div>
      )}
      {gadenSimulationClickVisible && clickedGif && !robotSimulation && (
        <div className="gaden-simulation-click">
          <button
            className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGoBackSimulationDetails}
          >
            Go Back
          </button>

          <div className="content-container">
            <div className="gif-description-robot">
              <h3>Height: {clickedGif.height ?? 'Unknown'}</h3>
              <img
                src={clickedGif.url}
                className="gif-image"
                alt="Simulation"
              />
            </div>

            <div className="robot-path-list-container">
              <h4>Robot Path:</h4>
              {Array.isArray(clickedGif.robot_path) && clickedGif.robot_path.length > 0 ? (
                <ul className="robot-path-list">
                  {clickedGif.robot_path.map((point, index) => (
                    <li key={index} className="robot-path-item">
                      <strong>Position:</strong> (x: {point.robot_position.x.toFixed(2)}, y: {point.robot_position.y.toFixed(2)}, z: {point.robot_position.z})<br />
                      <strong>Concentration:</strong> {point.concentration}<br />
                      <strong>Current:</strong> (x: {point.wind_speed.x.toFixed(3)}, y: {point.wind_speed.y.toFixed(3)}, z: {point.wind_speed.z.toFixed(3)})
                    </li>
                  ))}
                </ul>
              ) : (
                <p>No robot path data available.</p>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  

  );
};

export default Welcome;
