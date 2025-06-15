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
  const [finalRobotXlocation, setFinalRobotXLocation] = useState('');
  const [finalRobotYlocation, setFinalRobotYLocation] = useState('');
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
  const [minIteration, setMinIteration] = useState(0);
  const [imageLoaded, setImageLoaded] = useState({});
  const [relatedGifs, setRelatedGifs] = useState([]);
  const [isPausedGaden, setIsPausedGaden] = useState(false);
  const intervalRef = useRef(null);
  const [gadenSimulationOriginalSpeed, setGadenSimulationOriginalSpeed] = useState(500);
  const [gadenSimulationSpeed, setGadenSimulationSpeed] = useState(1);
  const [robotPathData, setRobotPathData] = useState({});
  const [showTotalStatsRobotSim, setShowTotalStatsRobotSim] = useState(false);
  const [selectedRobotNumber, setSelectedRobotNumber] = useState(1);
  const [selectedRobotFilter, setSelectedRobotFilter] = useState('all');
  const [robotNumbers, setRobotNumbers] = useState([]);

  const [robots, setRobots] = useState([
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
  ]);





  const handleRobotInputChange = (idx, field, value) => {
    setRobots(prev =>
      prev.map((robot, i) =>
        i === idx ? { ...robot, [field]: value } : robot
      )
    );
  };

  useEffect(() => {
    if (!relatedGifs || relatedGifs.length === 0) {
      setRobotNumbers([]);
      return;
    }
    const robotsAtZero = relatedGifs
      .filter(gifObj => gifObj.iteration === 0 && Array.isArray(gifObj.robot_path))
      .flatMap(gifObj => gifObj.robot_path.map(point => point.robot));
    const uniqueRobots = Array.from(new Set(robotsAtZero));
    setRobotNumbers(uniqueRobots);
  }, [relatedGifs]);


  const handleImageLoaded = (url) => {
    setImageLoaded((prevState) => ({
      ...prevState,
      [url]: true,
    }));
  };
  
  useEffect(() => {
    if (isPausedGaden) {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
      return; 
    }

    const speed = gadenSimulationOriginalSpeed / gadenSimulationSpeed;

    if (maxIteration > 0) {
      intervalRef.current = setInterval(() => {
        setCurrentIteration(prev => {
          if (prev < maxIteration) {
            return prev + 1; 
          } else {
            return minIteration;
          }
        });
      }, speed); 
    }
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current); 
      }
    };
  }, [relatedGifs, maxIteration, isPausedGaden, gadenSimulationSpeed, gadenSimulationOriginalSpeed]);


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
      const max = Math.max(...filteredGifs.map(g => g.iteration ));
    }
  }, [filteredGifs]);

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

      const gifs = gifsData.map(({ gif, height,type,iteration,robotSim_id,robot_path }) => {
  

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
          robotSim_id,
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

  const handleGifClick = (clickedGif) => {
    const { type, height, robotSim_id } = clickedGif;

    const filteredRelatedGifs = filteredGifs.filter(gifObj => gifObj.type === type && gifObj.height === height && gifObj.robotSim_id === robotSim_id);

    setRelatedGifs(filteredRelatedGifs);
    const maxIter = Math.max(...filteredRelatedGifs.map(g => g.iteration));
    setMaxIteration(maxIter);
    const minIter = Math.min(...filteredRelatedGifs.map(g => g.iteration));
    setMinIteration(minIter);

    setClickedGif(clickedGif);
    setHeight(clickedGif.height);
    
    setGadenSimulationClickVisible(true);
    setSimulationDetail(false);


    setCurrentIteration(0);

  };

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
      setRelatedGifs([]); 
      setCurrentIteration(0);
      setMaxIteration(0);
      setMinIteration(0);
    }, 500);
  }


  const handleRobotSimulationSubmit = async (e,simulation) => {
    e.preventDefault();
    setRobotSimulationIsLoading(true);
    const robotsToSend = robots
    .slice(0, selectedRobotNumber)
    .filter(robot =>
      robot.robotSpeed && robot.robotXlocation && robot.robotYlocation && robot.finalRobotXlocation && robot.finalRobotYlocation
    );

  if (robotsToSend.length === 0) {
    alert('Please fill in all fields for at least one robot.');
    setRobotSimulationIsLoading(false);
    return;
  }

  const formData = {
    username,
    simulation,
    height,
    robots: robotsToSend
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

  const handlePauseResume = () => {
    setIsPausedGaden(prev => !prev);
  };

  const handleIterationBackGaden = () => {
    if(currentIteration > minIteration && isPausedGaden){
      setCurrentIteration(prev => prev -1 )
    }
  }

  const handleIterationForwardGaden = () => {
    if(currentIteration < maxIteration && isPausedGaden){
      setCurrentIteration(prev => prev + 1)
    }   
  }

  const handleChangeSimulationSpeedGaden = () => {
    setGadenSimulationSpeed((prev) => {
      if (prev === 1) return 2;
      if (prev === 2) return 4;
      if (prev === 4) return 8;
      return 1;
    });
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
                .sort((a, b) => {a.type.localeCompare(b.type)})
                .filter((gifObj) => {
                  if (activeButton === 'gaden') {
                    return ['heatmap', 'wind', 'contour'].includes(gifObj.type) && gifObj.iteration === currentIteration; 
                  }
                  if (activeButton === 'robot') {
                    return gifObj.type === 'robot' && gifObj.iteration === 0;  
                  }
                  return true;
                })
                .filter((gifObj) => {
                  if (!selectedHeight) {
                    return true;
                  }
                  return gifObj.height == selectedHeight;
                })
                
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">
                    <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                    <img
                      src={gifObj.url}
                      alt={`Simulation GIF ${index + 1}`}
                      className="gif-image"
                      onClick={() => {handleGifClick(gifObj)}}
                      onLoad={() => handleImageLoaded(gifObj.url)}
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
          <div className="content-container">
            <div className="gif-description-robot">
              {relatedGifs
                .filter(gifObj => gifObj.iteration === currentIteration)
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">
                    <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                    <h3>Iteration: {gifObj.iteration}</h3>
                    <img
                      src={gifObj.url}
                      alt={`Related GIF ${index + 1}`}
                      className="gif-image"
                      style={{ cursor: 'pointer' }}
                    />
                    <div className="button-container-gaden-gif">
                      <div>
                      <button onClick={handleIterationBackGaden}>
                        {currentIteration > minIteration && isPausedGaden ? '⏮️' : '🚫'}
                      </button>
                      <button onClick={handlePauseResume}> 
                        {isPausedGaden ? '▶️' : '⏸️'} 
                      </button>
                      <button onClick={handleIterationForwardGaden}>
                        {currentIteration < maxIteration && isPausedGaden ? '⏭️' : '🚫'}
                      </button>
                      </div>
                      <div>
                        <button onClick={handleChangeSimulationSpeedGaden}>
                          {gadenSimulationSpeed}x
                        </button>
                        <button onClick={() => setCurrentIteration(minIteration)}>
                          ↻
                        </button>
                        </div>
                    </div>
                  </div>
                ))}
            </div>
            <div className="robot-simulation-form-container">
              <form onSubmit={(e) => handleRobotSimulationSubmit(e, clickedGif.simulation)} className="robot-simulation-form">              <div className="robot-simulation-inputs">
                <div className="robot-simulation-inputs">
                  {[1, 2, 3, 4].map((num) => (
                    <button
                      key={num}
                      type="button"
                      className={`number-of-robots-button${selectedRobotNumber === num ? ' selected' : ''}`}
                      onClick={() => setSelectedRobotNumber(num)}
                    >
                      {num}
                    </button>
                  ))}
                  </div>
                  <br />
                 {[...Array(selectedRobotNumber)].map((_, idx) => (
                  <div key={idx} className="robot-params-input">
                  <label>Number of robots</label>
                    <h4>Robot {idx + 1}</h4>
                    <label>Robot Speed in meters</label>
                    <input
                      type="float"
                      value={robots[idx].robotSpeed}
                      onChange={e => handleRobotInputChange(idx, 'robotSpeed', e.target.value)}
                      placeholder="Enter a robot speed value X.X"
                    />
                    <label>Initial robot X coordinate</label>
                    <input
                      type="float"
                      value={robots[idx].robotXlocation}
                      onChange={e => handleRobotInputChange(idx, 'robotXlocation', e.target.value)}
                      placeholder="Enter a robot X location value X.X"
                    />
                    <label>Initial robot Y coordinate</label>
                    <input
                      type="float"
                      value={robots[idx].robotYlocation}
                      onChange={e => handleRobotInputChange(idx, 'robotYlocation', e.target.value)}
                      placeholder="Enter a robot Y location value X.X"
                    />
                    <label>Final robot X coordinate</label>
                    <input
                      type="float"
                      value={robots[idx].finalRobotXlocation}
                      onChange={e => handleRobotInputChange(idx, 'finalRobotXlocation', e.target.value)}
                      placeholder="Enter final X location value X.X"
                    />
                    <label>Final robot Y coordinate</label>
                    <input
                      type="float"
                      value={robots[idx].finalRobotYlocation}
                      onChange={e => handleRobotInputChange(idx, 'finalRobotYlocation', e.target.value)}
                      placeholder="Enter final Y location value X.X"
                    />
                  </div>
                ))}
                  <button type="submit" className={`submit-button ${fadeOut ? 'fade-out' : ''}`} >Submit</button>
                </div>
                
              </form>
            </div>
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
              
              {relatedGifs
                .filter(gifObj => gifObj.iteration === currentIteration)
              
                .map((gifObj, index) => (
                    <div key={index} className="gif-description">
                      <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                      <h3>Iteration: {gifObj.iteration}</h3>
                      <img
                        src={gifObj.url}
                        alt={`Related GIF ${index + 1}`}
                        className="gif-image"
                        style={{ cursor: 'pointer' }}
                      />
                      <div className="button-container-gaden-gif">
                        <div>
                        <button onClick={handleIterationBackGaden}>
                          {currentIteration > minIteration && isPausedGaden ? '⏮️' : '🚫'}
                        </button>
                        <button onClick={handlePauseResume}> 
                          {isPausedGaden ? '▶️' : '⏸️'} 
                        </button>
                        <button onClick={handleIterationForwardGaden}>
                          {currentIteration < maxIteration && isPausedGaden ? '⏭️' : '🚫'}
                        </button>
                        </div>
                        <div>
                          <button onClick={handleChangeSimulationSpeedGaden}>
                            {gadenSimulationSpeed}x
                          </button>
                          <button onClick={() => setCurrentIteration(minIteration)}>
                            ↻
                          </button>
                          </div>
                      </div>
                    </div>
                ))
              }
            </div>
            <div className="robot-path-list-container">
              <h4>Robot Path:</h4>
              <div className="checkbox-filters">
              <label>
                <input
                  type="checkbox"
                  checked={showTotalStatsRobotSim}
                  onChange={() => setShowTotalStatsRobotSim((prev) => !prev)}
                />
                show total stats
              </label>
              <br />
              <select
                value={selectedRobotFilter}
                onChange={e => setSelectedRobotFilter(e.target.value)}
              >
                <option value="all">All Robots</option>
                {robotNumbers.map(robotNum => (
                  <option key={robotNum} value={robotNum}>
                    Robot {robotNum}
                  </option>
                ))}
              </select>
              </div>
              <ul className="robot-path-list">
                {(() => {
                  const seen = new Set();
                  return (showTotalStatsRobotSim
                    ? relatedGifs.filter(gifObj => Array.isArray(gifObj.robot_path))
                    : relatedGifs.filter(gifObj => gifObj.iteration === currentIteration)
                  )
                    .flatMap(gifObj =>
                      gifObj.robot_path
                        .filter(point =>
                          selectedRobotFilter === 'all' ? true : String(point.robot) === String(selectedRobotFilter)
                        )
                        .filter(point => {
                          const key = `${gifObj.iteration}-${point.robot}`;
                          if (seen.has(key)) return false;
                          seen.add(key);
                          return true;
                        })
                        .map((point, idx) => (
                          <li
                            key={`${gifObj.iteration}-${point.robot}-${point.robot_position.x}-${point.robot_position.y}-${point.robot_position.z}-${idx}`}
                            className="robot-path-item"
                          >
                            <strong>Robot:</strong> {point.robot} <br />
                            <strong>Position:</strong> (x: {point.robot_position.x.toFixed(2)}, y: {point.robot_position.y.toFixed(2)}, z: {point.robot_position.z})<br />
                            <strong>Concentration:</strong> {Number(point.concentration).toFixed(7)}<br />
                            <strong>Current:</strong> (x: {point.wind_speed.x.toFixed(3)}, y: {point.wind_speed.y.toFixed(3)}, z: {point.wind_speed.z.toFixed(3)})
                          </li>
                        ))
                    );
                })()}
              </ul>
            </div>
          </div>
        </div>
      )}
    </div>
  

  );
};

export default Welcome;
