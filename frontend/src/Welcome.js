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
  const [SavedSimulationsVisible, setSavedSimulationsVisible] = useState(false);
  const [isNewSimulation, setIsNewSimulation] = useState(false);
	const [simulationName, setSimulationName] = useState('');
  const [plumeXLocation, setPlumeXLocation] = useState('');
  const [plumeYLocation, setPlumeYLocation] = useState('');
  const [plumeZLocation, setPlumeZLocation] = useState('');
  const [savedSimulations, setSavedSimulations] = useState([]);
  const [gifs, setGifs] = useState([]);
  const [loadingGifs, setLoadingGifs] = useState(true);
  const [searchQuery, setSearchQuery] = useState('');
  const [filteredSimulations, setFilteredSimulations] = useState(savedSimulations);
  const [simulationDetail, setSimulationDetail] = useState(false);
  const [simulationStatus, setSimulationStatus] = useState(null);
  const [showModal, setShowModal] = useState(false);
  const [simulationToDelete, setSimulationToDelete] = useState(null);

  const navigate = useNavigate();
  const searchInputRef = useRef(null);

  const [checkedOptions, setCheckedOptions] = useState({
    all: false,
    heatmap: false,
    wind: false,
    contour: false,
  });
    

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

  const filteredGifs = checkedOptions.all
        ? gifs
        : gifs.filter((gif) => checkedOptions[gif.type]);


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
      
      const gifs = gifsData.map(({ gif, height,type }) => {
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
          type
        };
      });
      
      const gifTypes = gifs.map(g => g.type);

      setCheckedOptions({
        all: true,
        heatmap: gifTypes.includes('heatmap'),
        wind: gifTypes.includes('wind'),
        contour: gifTypes.includes('contour'),
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
    if (SavedSimulationsVisible && searchInputRef.current) {
      searchInputRef.current.focus();
    }
  }, [SavedSimulationsVisible]);

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
    formData.append('plumeXLocation', plumeXLocation);
    formData.append('plumeYLocation', plumeYLocation);
    formData.append('plumeZLocation', plumeZLocation);
    console.log(plumeXLocation, plumeYLocation, plumeZLocation);
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
          } else if (status === "OUT OF BOUNDS") {
            const boundsData = await fetchBoundsStatus(simulation);
            if (boundsData) {
              const { xMin, xMax, yMin, yMax, zMin, zMax } = boundsData;
              alert(`Simulation is out of bounds.\nBounds:\nX: ${xMin} - ${xMax}\nY: ${yMin} - ${yMax}\nZ: ${zMin} - ${zMax}`);
              break;
            } else {
              alert("Simulation is out of bounds, but bounds could not be retrieved.");
            }
            break;
          } else if (status === "IN SIMULATION") {
            alert("Simulation has started.");
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

  const handleGoBack = () => {
    setFadeOut(true); 
    setTimeout(() => {
      setFileInputVisible(false);
      setGadenChoiseVisible(false);
      setIsNewSimulation(false);
      setSavedSimulationsVisible(false);
      setFadeOut(false); 
    }, 500);
  };

  const handleGoBackGadenChoise = () => {
    setFadeOut(true); 
    setTimeout(() => {
      setGadenChoiseVisible(true);
      setIsNewSimulation(false);
      setSavedSimulationsVisible(false);
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
            simulationStatus = "DONE";
            clearInterval(intervalId);
          } else {
            simulationStatus = "Simulation still running";
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
        {!fileInputVisible && !GadenChoiseVisible && !isNewSimulation && !SavedSimulationsVisible && !simulationDetail? (
          <button
            className={`gaden-button ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGadenClick}
          >
            Gaden
          </button>
        ) : null}
       {GadenChoiseVisible && (
         <div className="gaden-choice-buttons">
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
      {isNewSimulation && (
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
            <label htmlFor="plumeXLocation">Plume location </label>
            <input
              type="float"
              id="plumeXLocation"
              name="plumeXLocation"
              value={plumeXLocation}
              onChange={(e) => setPlumeXLocation(e.target.value)}
              placeholder="Enter plume location x value X.X"
            />
            <input
              type="float"
              id="plumeYLocation"
              name="plumeYLocation"
              value={plumeYLocation}
              onChange={(e) => setPlumeYLocation(e.target.value)}
              placeholder="Enter plume location y value X.X"
            />
            <input
              type="float"
              id="plumeZLocation"
              name="plumeZLocation"
              value={plumeZLocation}
              onChange={(e) => setPlumeZLocation(e.target.value)}
              placeholder="Enter plume location z value X.X"
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
        {SavedSimulationsVisible && (
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
              className={`go-back-simulation-details`}
              onClick={handleGoBackSavedSimulations}
            >
              Go Back
            </button>

            <div className="checkbox-filters">
              <label>
                <input
                  type="checkbox"
                  name="all"
                  checked={checkedOptions.all}
                  onChange={handleCheckboxChange}
                />
                Show all the simulations
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
            </div>
          </div>
          <div className="gif-container">
            {!loadingGifs && filteredGifs.length === 0 && (
              <p>No GIFs available for the selected options.</p>
            )}

            {filteredGifs.length > 0 && (
              filteredGifs
                .slice()
                .sort((a, b) => a.height - b.height)
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">
                    <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                    <img
                      src={gifObj.url}
                      alt={`Simulation GIF ${index + 1}`}
                      className="gif-image"
                    />
                  </div>
                ))
            )}
          </div>

      </>
      )}
      </div>
    </div>
  );
};

export default Welcome;
