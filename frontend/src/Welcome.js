import React, { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import './Welcome.css';
import logo from './flyrobotics_logo.png'; 
import GifWithGrid from './GifWithGrid';
import EnviromentGrid from './enviromentGrid';
import HoverComponent from './HoverComponent'

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
  const [robotSimulationMode, setRobotSimulationMode] = useState("linear");
  const [psoSimulationIterations, setPsoSimulationIterations] = useState("");
  const [startingIteration, setStartingIteration] = useState("");
  const [showGrid, setShowGrid] = useState(false);
  const [selectedRobotIdx, setSelectedRobotIdx] = useState(null);
  const [enviromentIsLoading,setEnviromentIsLoading] = useState(false);
  const [showFilamentOptions, setShowFilamentOptions] = useState(false);
  const [temperatureInC, setTemperatureInC] = useState("");
  const [ppmCenter, setPpmCenter] = useState("");
  const [numFilamentsSec, setNumFilamentsSec] = useState("");
  const [filamentInitialStd, setFilamentInitialStd] = useState("");
  const [filamentGrowth, setFilamentGrowth] = useState("");
  const [filamentNoise, setFilamentNoise] = useState("");
  const [nameSimulationSet, setNameSimulationSet] = useState("");
  const [numRobotSimulations, setNumRobotSimulations] = useState("");
  const [robotSetData, setRobotSetData] = useState("");
  const [robotSetSearch, setRobotSetSearch] = useState('');
  const [setToDelete, setSetToDelete] = useState('');
  const [filteredRobotSets, setFilteredRobotSets] = useState([]);
  const [gifsInSet, setGifsInSet] = useState([]); 
  const [selectedSetSimId, setSelectedSetSimId] = useState(null); 
  const [showRobotSetDetail, setShowRobotSetDetail] = useState(false);
  const [parentSimulationOfSet, setParentSimulationOfSet] = useState("");
  const [deviationSet, setDeviationSet] = useState("");
  const [useRos, setUseRos] = useState(false);
  const [medianTime, setMedianTime] = useState(null);

  const [robots, setRobots] = useState([
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
    { robotSpeed: '', robotXlocation: '', robotYlocation: '', finalRobotXlocation: '', finalRobotYlocation: '' },
  ]);



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
  




  const handleRobotInputChange = (idx, field, value) => {
    setRobots(prev =>
      prev.map((robot, i) =>
        i === idx ? { ...robot, [field]: value } : robot
      )
    );
  };

useEffect(() => {
  let sourceGifs = [];
  if (showRobotSetDetail && gifsInSet && gifsInSet.length > 0) {
    // Use only gifs from the selected set
    sourceGifs = gifsInSet.filter(gifObj => gifObj.robotSim_id === selectedSetSimId);
  } else if (relatedGifs && relatedGifs.length > 0) {
    sourceGifs = relatedGifs;
  } else {
    setRobotNumbers([]);
    return;
  }

  const robotsAtZero = sourceGifs
    .filter(gifObj => gifObj.iteration === 0 && Array.isArray(gifObj.robot_path))
    .flatMap(gifObj => gifObj.robot_path.map(point => point.robot));
  const uniqueRobots = Array.from(new Set(robotsAtZero));
  setRobotNumbers(uniqueRobots);
}, [relatedGifs, gifsInSet, selectedSetSimId, showRobotSetDetail]);


  const handleImageLoaded = (url) => {
    setImageLoaded((prevState) => ({
      ...prevState,
      [url]: true,
    }));
  };
  
useEffect(() => {
  // Always clear previous interval
  if (intervalRef.current) {
    clearInterval(intervalRef.current);
    intervalRef.current = null;
  }

  // Only run if not paused
  if (isPausedGaden) return;

  let minIter, maxIter;
  if (showRobotSetDetail) {
    const simGifs = gifsInSet.filter(g => g.robotSim_id === selectedSetSimId);
    if (!simGifs.length) return;
    minIter = Math.min(...simGifs.map(g => g.iteration));
    maxIter = Math.max(...simGifs.map(g => g.iteration));
  } else if (gadenSimulationClickVisible || simulationDetail) {
    if (!filteredGifs.length) return;
    minIter = Math.min(...filteredGifs.map(g => g.iteration));
    maxIter = Math.max(...filteredGifs.map(g => g.iteration));
  } else {
    return;
  }

  const speed = gadenSimulationOriginalSpeed / gadenSimulationSpeed;

  intervalRef.current = setInterval(() => {
    setCurrentIteration(prev => {
      if (prev < maxIter) {
        return prev + 1;
      } else {
        return minIter;
      }
    });
  }, speed);

  return () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
  };
}, [
  isPausedGaden,
  gadenSimulationSpeed,
  gadenSimulationOriginalSpeed,
  showRobotSetDetail,
  gifsInSet,
  selectedSetSimId,
  filteredGifs,
  gadenSimulationClickVisible,
  simulationDetail
]);



  

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
    if (!enviromentIsLoading) return; 

    let dotCount = 0;
    const interval = setInterval(() => {
      dotCount = (dotCount + 1) % 4; 
      setPlumeSimulationLoadingText(`Preprocessing the simulation${".".repeat(dotCount)}`);
    }, 500);
    return () => clearInterval(interval);
  }, [enviromentIsLoading]);

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


  const fetchEnviromentResults = async (simulation) => {
    try {
      setLoadingGifs(true);
      const response = await fetch(`http://localhost:3000/getEnviromentFrames?simulation=${simulation}`);
      
      if (!response.ok) {
        throw new Error('Failed to fetch Enviroment frames');
        console.log(response)
      }
      
      const gifsData = await response.json();

      const gifs = gifsData.map(({ gif, height,type,iteration }) => {
  

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
        };
      });
      

      const uniqueHeights = [...new Set(gifs.map(gif => gif.height))];
      const minHeight = Math.min(...uniqueHeights);
      setSelectedHeight(minHeight);


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

 const fetchGifsFromSet = async (set) => {
  try {
    setLoadingGifs(true);

    const response = await fetch(`http://localhost:3000/getGifsFromSimulation?set=${set.simulation_set}&&simulation=${set.simulation}`);
    if (!response.ok) {
      throw new Error('Failed to fetch GIFs');
    }

    const gifsData = await response.json();

    const gifs = gifsData.map(({ gif, height, type, iteration, robotSim_id,time, robot_path }) => {
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
        simulation: set.simulation,
        iteration,
        robotSim_id,
        time,
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
    const minHeight = Math.min(...uniqueHeights);
    setSelectedHeight(minHeight);
    setAvailableHeights(uniqueHeights);

    const gifTypes = gifs.map(g => g.type);
    setCheckedOptions({
      all: true,
      heatmap: gifTypes.includes('heatmap'),
      wind: gifTypes.includes('wind'),
      contour: gifTypes.includes('contour'),
      [minHeight]: true,
    });

    return gifs; 
  } catch (error) {
    console.error('Error fetching GIFs:', error);
    return []; 
  } finally {
    setLoadingGifs(false);
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

      const gifs = gifsData.map(({ gif, height,type,iteration,robotSim_id,time,robot_path }) => {
  

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
          time,
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
      const minHeight = Math.min(...uniqueHeights);
      setSelectedHeight(minHeight);

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

    setEnviromentIsLoading(true);
    try {
      const response = await fetch('http://localhost:3000/uploadFiles', {
        method: 'POST',
        body: formData,
      });  
      if (response.ok) {
        const simulation = username + "_" + simulationNumber;
  
     
          const bounds = await fetchBoundsStatus(simulation);
          await fetchEnviromentResults(simulation); 
          setSimulationBounds(bounds);
          setShowPlumeLocation(true);
          setEnviromentIsLoading(false);

  
          await new Promise(resolve => setTimeout(resolve, 3000));

        await fetchSimulationNumber();
      } else {
        setEnviromentIsLoading(false);
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

    let temperatureToSend;
    let ppmCenterToSend;
    let numFilamentsSecToSend;
    let filamentInitialStdToSend;
    let filamentGrowthToSend;
    let filamentNoiseToSend;

    const simulationCorrected = simulationNumber - 1;

    if (temperatureInC === ""){
      temperatureToSend = 298.0;
    }else{
      temperatureToSend = temperatureInC + 273.15;
    }

    if (ppmCenter === ""){
      ppmCenterToSend = 10.0;
    }else{
      ppmCenterToSend = ppmCenter;
    }

    if (numFilamentsSec === ""){
      numFilamentsSecToSend = 10;
    }else{
      numFilamentsSecToSend = numFilamentsSec;
    }

    if (filamentInitialStd === ""){
      filamentInitialStdToSend = 10.0;
    }else{
      filamentInitialStdToSend = filamentInitialStd;
    }

    if (filamentGrowth === ""){
      filamentGrowthToSend = 10.0;
    }else{
      filamentGrowthToSend = filamentGrowth;
    }

    if (filamentNoise === ""){
      filamentNoiseToSend = 0.02;
    }else{
      filamentNoiseToSend = filamentNoise;
    }

    const formData = {
      username,
      simulationNumber: simulationCorrected,
      plumeXlocation: plumeXLocation,
      plumeYlocation: plumeYLocation,
      plumeZlocation: plumeZLocation,
      temperatureInK: temperatureToSend,
      ppmCenter: ppmCenterToSend,
      numFilamentsSec: numFilamentsSecToSend,
      filamentInitialStd: filamentInitialStdToSend,
      filamentGrowth: filamentGrowthToSend,
      filamentNoise: filamentNoiseToSend,
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


  const handleSetClick = async (set) => {
    setGifs([]);
    setGifsInSet([]); 
    setSelectedSetSimId(null);
    setShowRobotSetDetail(true); 
    setSimulationDetail(false);
    setParentSimulationOfSet(set.simulation)

    const newGifs = await fetchGifsFromSet(set);
    setGifsInSet(newGifs);

    if (newGifs.length > 0) {
      setSelectedSetSimId(newGifs[0].robotSim_id);
      setGifs(newGifs.filter(g => g.robotSim_id === newGifs[0].robotSim_id));
    }
  };

  const handleGoBackRobotSetDetail = async () => {
    setShowRobotSetDetail(false);
    setGifsInSet([]);
    setGifs([]);
    setSelectedSetSimId(null);
    setSimulationDetail(true);
    await fetchGifsFromResults(parentSimulationOfSet);
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


  

  const handleGifClick = async (clickedGif) => {
    const { type, height, robotSim_id } = clickedGif;

    const filteredRelatedGifs = filteredGifs.filter(gifObj => gifObj.type === type && gifObj.height === height && gifObj.robotSim_id === robotSim_id);

    setRelatedGifs(filteredRelatedGifs);
    const maxIter = Math.max(...filteredRelatedGifs.map(g => g.iteration));
    setMaxIteration(maxIter);
    const minIter = Math.min(...filteredRelatedGifs.map(g => g.iteration));
    setMinIteration(minIter);


    const bounds = await fetchBoundsStatus(clickedGif.simulation);
      if (bounds) setSimulationBounds(bounds);

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

  const openSetDeleteModal = (set) => {
    setSetToDelete(set);
    setShowModal(true);
  };

  const closeSetModal = () => {
    setShowModal(false);
    setSetToDelete(null);
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

  const confirmSetDelete = async () => {
    console.log(setToDelete.simulation_set);
    try {
      const response = await fetch(`http://localhost:3000/deleteSimulationSet?set=${setToDelete.simulation_set}&&simulation=${setToDelete.simulation}`, {
        method: 'POST',
      });

      if (!response.ok) {
        const errorMessage = await response.text();
        alert(`Error: ${errorMessage}`);
      }
      closeModal();
      await fetchRobotSetData(setToDelete.simulation);
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


  const handleRobotSimulationSubmit = async (e, simulation) => {
    e.preventDefault();
    setRobotSimulationIsLoading(true);

    let url = "";
    const robotsToSend = robots
        .slice(0, selectedRobotNumber)
        .filter(robot => {
            if (robotSimulationMode === 'linear') {
                url = "http://localhost:3000/robotSimulation";
                return robot.robotSpeed && robot.robotXlocation && robot.robotYlocation &&
                    robot.finalRobotXlocation && robot.finalRobotYlocation;
            } else if (robotSimulationMode === "moth") {
                url = "http://localhost:3000/silkworm_moth_simulation";
                return robot.robotSpeed && robot.robotXlocation && robot.robotYlocation &&
                    robot.angle && robot.iterations;
            } else if (robotSimulationMode === "pso") {
                url = "http://localhost:3000/pso_simmulation";
                return robot.robotSpeed && robot.robotXlocation && robot.robotYlocation &&
                    psoSimulationIterations;
            }
        })
        .map(robot => ({
            ...robot,
            robotSpeed: parseFloat(robot.robotSpeed),
            robotXlocation: parseFloat(robot.robotXlocation),
            robotYlocation: parseFloat(robot.robotYlocation),
            angle: robotSimulationMode === 'moth' ? parseFloat(robot.angle) : undefined,
            iterations: robotSimulationMode === 'pso'
                ? parseInt(psoSimulationIterations)
                : robot.iterations !== undefined
                    ? parseInt(robot.iterations)
                    : undefined,

        }));

    if (robotsToSend.length === 0) {
        alert('Please fill in all fields for at least one robot.');
        setRobotSimulationIsLoading(false);
        return;
    }

    let startingIterationToSend 

    if (startingIteration === ""){
      startingIterationToSend = 0
    }else{
      startingIterationToSend = startingIteration
    }

    if (numRobotSimulations === ""){
      setNumRobotSimulations(1);
    }

    const formData = {
        username,
        simulation: simulation,
        nameOfSet: nameSimulationSet,
        numOfSim: numRobotSimulations,
        height: parseFloat(height),
        numberOfRobots: robotsToSend.length,
        robots: robotsToSend,
        simulationMode: robotSimulationMode,
        startingIteration: startingIterationToSend,
        deviation: parseFloat(deviationSet),
        useRos
    };

    try {
      const response = await fetch(url, {
          method: 'POST',
          headers: {
              'Content-Type': 'application/json',
          },
          body: JSON.stringify(formData)
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
        handleToggleButton('robot');
        setSimulationDetail(true);
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
      fetchRobotSetData(clickedGif?.simulation || (filteredGifs[0] && filteredGifs[0].simulation));
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


  const handlePsoIterationsInputChange = (iterations) => {
    setPsoSimulationIterations(iterations);
  }

  const handleStartingIterationInputChange = (iteration) => {
    setStartingIteration(iteration);
  }

  const toggleGrid = () =>{
    setShowGrid(!showGrid);
  }

  const  handleNumRobotSimulationsChange = (robotSim) => {
    setNumRobotSimulations(robotSim);
  }

  const handleNameSimulationSetChange = (name) => {
    setNameSimulationSet(name);
  }

  const handleDeviationSetChange = (deviation) => {
    setDeviationSet(deviation);
  }

  const fetchRobotSetData = async (simulation) => {
  try {
    const response = await fetch(`http://localhost:3000/getRobotSet?simulation=${simulation}`);
    if (!response.ok) {
      throw new Error('Failed to fetch robot simulation sets');
    }
    const data = await response.json();
    console.log(data);
    setRobotSetData(data);
  } catch (error) {
    console.error('Error fetching robot simulation sets:', error);
    setRobotSetData([]);
  }
};

  useEffect(() => {
    if (!robotSetData || !Array.isArray(robotSetData)) {
      setFilteredRobotSets([]);
      return;
    }

    const setMap = {};

    robotSetData.forEach(set => {
      if (!set.simulation_set) return;
      const [base, rest] = set.simulation_set.split('/');
      if (!base || !rest) return;

      let numberStr = rest.split(/[:/]/)[0];
      let number = parseInt(numberStr, 10);
      if (isNaN(number)) number = -Infinity;

      if (!setMap[base] || setMap[base].number < number) {
        setMap[base] = {
          ...set,
          number,
          simulation_set: set.simulation_set 
        };
      }
    });

  const sets = Object.values(setMap)
      .filter(set =>
        set.simulation_set &&
        set.simulation_set.split('/')[0].toLowerCase().includes(robotSetSearch.toLowerCase())
      )
      .sort((a, b) => b.number - a.number);

    setFilteredRobotSets(sets);
  }, [robotSetData, robotSetSearch]);

  useEffect(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }

    if (!robotSetData || !Array.isArray(robotSetData) || filteredRobotSets.length === 0) return;

    let shouldPoll = false;
    let pollSimulation = null;

    filteredRobotSets.forEach(set => {
      if (!set.simulation_set) return;
      const parts = set.simulation_set.split('/');
      if (parts.length < 2) return;
      const [xStr, yStr] = parts[1].includes(':') ? parts[1].split(':') : parts[1].split('/');
      const x = parseInt(xStr, 10);
      const y = parseInt(yStr, 10);
      if (x < y) {
        shouldPoll = true;
        pollSimulation = set.simulation;
      }
    });

    if (shouldPoll && pollSimulation) {
      intervalRef.current = setInterval(() => {
        fetchRobotSetData(pollSimulation);
      }, 5000);
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [robotSetData, filteredRobotSets]);


  useEffect(() => {
    if (activeButton === 'robot' && simulationDetail) {
      intervalRef.current = setInterval(() => {
        const simulation =
          clickedGif?.simulation ||
          (filteredGifs[0] && filteredGifs[0].simulation);
        if (simulation) {
          fetchRobotSetData(simulation);
        }
      }, 5000);
    }
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [activeButton, simulationDetail, clickedGif, filteredGifs]);
  

  const toggleRos = () =>{
    setUseRos(!useRos);
  }

  const getMedianTime = (objs) => {
    const times = objs.map(obj => obj.time).sort((a, b) => a - b);
    const mid = Math.floor(times.length / 2);
    return times.length % 2 === 0
      ? (times[mid - 1] + times[mid]) / 2
      : times[mid];
  };

  useEffect(() => {
    if (gifsInSet && gifsInSet.length > 0) {
      const median = getMedianTime(gifsInSet);
      setMedianTime(median);  
    }
  }, [gifsInSet]);

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
        {!showRobotSetDetail && !gadenSimulationClickVisible && !fileInputVisible && !GadenChoiseVisible && !isNewSimulation && !savedSimulationsVisible && !simulationDetail? (
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
        <>
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
         {enviromentIsLoading && (
          <div className="popup-overlay">
            <div className="popup">
              <p>{plumeSimulationLoadingText}</p>
            </div>
          </div>
        )}
        </>
      )}
      {isNewSimulation && showPlumeLocation && (
        <div className="content-container">
          {filteredGifs.length > 0 && (
            filteredGifs
              .slice()
              .sort((a, b) => a.type.localeCompare(b.type))
              .filter((gifObj) => {
                return gifObj.height == selectedHeight;
              })
              .map((gifObj, index) => (
                <div key={index} className="gif-description">
                  <div>
                  <h3>Height: </h3>
                   <select
                    value={selectedHeight}
                    onChange={(e) => setSelectedHeight(e.target.value)}
                  >
                    {availableHeights
                      .map((height) => parseFloat(height)) 
                      .sort((a, b) => a - b)
                      .map((height, idx) => (
                        <option key={idx} value={height}>
                          {height ?? 'Unknown'}
                        </option>
                      ))}
                  </select>
                  <div className="grid-toggle-div">
                  <label>
                        <input
                          type="checkbox"
                          className = "toggle-button"
                          checked={showGrid}
                          onChange={() => toggleGrid()}
                        />
                        Show Grid
                  </label>
                  </div>
                  </div> 
                  {showGrid ? (
                    <EnviromentGrid
                        gifObj={gifObj}
                        simulationBounds={simulationBounds}
                        grid={true}
                        height={selectedHeight}
                        onSetPlumeCoords={(x, y, z) => {
                          setPlumeXLocation(x.toFixed(1));
                          setPlumeYLocation(y.toFixed(1));
                          setPlumeZLocation(z.toFixed(1));
                        }}
                  />

                  ):(
                    <EnviromentGrid
                        gifObj={gifObj}
                        simulationBounds={simulationBounds}
                        grid={false}
                        height={selectedHeight}
                        onSetPlumeCoords={(x, y, z) => {
                          setPlumeXLocation(x.toFixed(1));
                          setPlumeYLocation(y.toFixed(1));
                          setPlumeZLocation(z.toFixed(1));
                        }}
                  />
                  )} 
                  
                </div>
              ))
          )}
          <form onSubmit={handlePlumeSubmit} className="file-upload-form">
            <div>
              <h4 htmlFor="plumeXLocation">Plume location </h4>
              <label 
                htmlFor="plumeXLocation">X coord Range: {simulationBounds.xMin} and {simulationBounds.xMax}
                <HoverComponent text="X value X.X for the plume location"/>
              </label>
              <input
                type="float"
                id="plumeXLocation"
                name="plumeXLocation"
                value={plumeXLocation}
                onChange={(e) => setPlumeXLocation(e.target.value)}
              />
              <label 
                htmlFor="plumeYLocation">Y coord Range: {simulationBounds.yMin} and {simulationBounds.yMax}
                <HoverComponent text="Y value X.X for the plume location"/>
              </label>
              <input
                type="float"
                id="plumeYLocation"
                name="plumeYLocation"
                value={plumeYLocation}
                onChange={(e) => setPlumeYLocation(e.target.value)}
              />
              <label 
                htmlFor="plumeZLocation"> Z coord Range: {simulationBounds.zMin} and {simulationBounds.zMax} 
                <HoverComponent text="Z value X.X for the plume location "/>
              </label>
              <input
                type="float"
                id="plumeZLocation"
                name="plumeZLocation"
                value={plumeZLocation}
                onChange={(e) => setPlumeZLocation(e.target.value)}
              />
              <label 
                htmlFor="temperatureInC">temperature
                <HoverComponent text="temperature in Celsius Default: 24.85" />
              </label>
              <input
                type="float"
                id="temperatureInC"
                step="0.01"
                min="−273.15"
                name="temperatureInC"
                value={temperatureInC}
                onChange={(e) => setTemperatureInC(e.target.value)}
              />
              </div>

              <div className="collapsible-section">
                <button
                  type="button"
                  onClick={() => setShowFilamentOptions(!showFilamentOptions)}
                  className="collapsible-toggle"
                >
                  {showFilamentOptions ? 'Hide' : 'Show'} Filament Options (optional){showFilamentOptions ? '▲' : '▼'}
                </button>

                {showFilamentOptions && (
                  <div className="filament-options">
                    <label 
                      htmlFor="ppmCenter">PPM
                      <HoverComponent text="ppm of the initial plume Default: 10ppm" />
                    </label>
                    <input
                      type="number"
                      step="1"
                      id="ppmCenter"
                      name="ppmCenter"
                      value={ppmCenter}
                      onChange={(e) => setPpmCenter(e.target.value)}
                    />
                    <label 
                      htmlFor="numFilamentsSec">Number of filaments per second
                      <HoverComponent text="Number of filaments released each second Default: 10 " />
                    </label>
                    <input
                      type="number"
                      step="1"
                      id="numFilamentsSec"
                      name="numFilamentsSec"
                      value={numFilamentsSec}
                      onChange={(e) => setNumFilamentsSec(e.target.value)}
                    />
                    <label 
                      htmlFor="filamentInitialStd">Sigma of the filament 
                      <HoverComponent text="Sigma of the filament at t=0 in cm Default: 10" />
                    </label>
                    <input
                      type="float"
                      step="0.1"
                      id="filamentInitialStd"
                      name="filamentInitialStd"
                      value={filamentInitialStd}
                      onChange={(e) => setFilamentInitialStd(e.target.value)}
                    />
                    <label 
                      htmlFor="filamentGrowth">Filament Growth
                      <HoverComponent text="Growth ratio of the filament_std in cm²/s Default: 10" />
                    </label>
                    <input
                      type="float"
                      step="0.1"
                      id="filamentGrowth"
                      name="filamentGrowth"
                      value={filamentGrowth}
                      onChange={(e) => setFilamentGrowth(e.target.value)}
                    />
                    <label 
                      htmlFor="filamentNoise"> Filament Noise
                      <HoverComponent text="Range of the white noise added on each iteration in m Default: 0.02" />
                    </label>
                    <input
                      type="float"
                      step="0.01"
                      id="filamentNoise"
                      name="filamentNoise"
                      value={filamentNoise}
                      onChange={(e) => setFilamentNoise(e.target.value)}
                    />

                  </div>
                )}
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
             {activeButton === 'robot' && robotSetData && robotSetData.length > 0 && (
              <div className={`saved-simulations-list ${fadeOut ? 'fade-out' : ''}`}>
                <div className="robot-sim-header">
                  <input
                    ref={searchInputRef}
                    type="text"
                    className="search-bar"
                    placeholder="Search by name of the set"
                    value={robotSetSearch}
                    onChange={e => setRobotSetSearch(e.target.value)}
                  />
                  <h3 className="simulation-title">Robot Simulation Sets</h3>
                </div>
                <ul>
                  {filteredRobotSets
                  .map((set, idx) => (
                    <li
                      key={idx}
                      className={`simulation-item ${fadeOut ? 'fade-out' : ''}`}
                    >
                      <div className="simulation-content">
                        <div 
                          className="simulation-details"
                          onClick={() => handleSetClick(set)}
                        >
                          <strong>Name:</strong> {set.simulation_set ? set.simulation_set.split('/')[0] : 'Unnamed Set'} <br />
                          <strong>Number of Simulations:</strong> {set.simulation_set ? set.simulation_set.split('/').slice(1).join('/') : '0'}                        </div>
                        <div
                          className="trash-icon"
                          onClick={(e) => {
                            e.stopPropagation();
                            openSetDeleteModal(set);
                          }}
                        >
                          🗑️
                        </div>
                      </div>
                    </li>
                  ))}
                </ul>
              </div>

            )}
            {showModal && (
                <ConfirmationModal
                  message="Are you sure you want to delete this set?"
                  onConfirm={confirmSetDelete}
                  onCancel={closeSetModal}
                />
              )}
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
              
                 <select
                    value={selectedHeight}
                    onChange={(e) => setSelectedHeight(e.target.value)}
                  >
                    {availableHeights
                      .map((height) => parseFloat(height)) 
                      .sort((a, b) => a - b)
                      .map((height, idx) => (
                        <option key={idx} value={height}>
                          {height ?? 'Unknown'}
                        </option>
                      ))}
                  </select>
              </div> 
          )}
          </div>
          <div className="gif-container">
            {!loadingGifs && filteredGifs.length === 0 && !activeButton === 'robot' && (
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
                })
                .filter((gifObj) => {
                  if (gifObj.type !== 'robot') {
                    return gifObj.height == selectedHeight;
                  }
                  return true;
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
      {showRobotSetDetail && (
        <div className="gaden-simulation-click">
          <button
            className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGoBackRobotSetDetail}
          >
            Go Back
          </button>
          <div className="content-container">
            <div className="gif-description-robot">
              {gifsInSet
                .filter(gifObj => gifObj.robotSim_id === selectedSetSimId && gifObj.iteration === currentIteration)
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">
                  <div className="Simulation-set-select-container">
                    <label>
                      Select Simulation in Set:&nbsp;
                      <select
                        value={selectedSetSimId}
                        onChange={e => {
                          const simId = Number(e.target.value);
                          setSelectedSetSimId(simId);
                          setCurrentIteration(0);
                          setGifs(gifsInSet.filter(g => g.robotSim_id === simId));
                        }}
                      >
                        {Array.from(new Set(gifsInSet.map(g => g.robotSim_id)))
                          .filter(id => id !== undefined && id !== null)
                          .map((id, index, array) => (
                            <option key={id} value={id}>
                              Simulation {index + 1}
                            </option>
                          ))}
                      </select>
                    </label>
                  </div>
                    <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                    <h3>Median time per iteration: {medianTime} </h3>
                    <h3>Iteration: {gifObj.iteration} - took: {gifObj.time} s</h3>
                    <GifWithGrid
                      gifObj={gifObj}
                      simulationBounds={simulationBounds}
                      robots={robots}
                      selectedRobotIdx={selectedRobotIdx}
                      grid={showGrid}
                      deviation={deviationSet}
                      numberOfRobots={selectedRobotNumber}
                      type={robotSimulationMode}
                      onSetRobotCoords={() => {}}
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
                  const simGifs = gifsInSet.filter(g => g.robotSim_id === selectedSetSimId);
                  return (showTotalStatsRobotSim
                    ? simGifs.filter(gifObj => Array.isArray(gifObj.robot_path))
                    : simGifs.filter(gifObj => gifObj.iteration === currentIteration)
                  )
                    .flatMap(gifObj =>
                      (gifObj.robot_path || [])
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
                            <strong>Current:</strong> (x: {point.wind_speed.x.toFixed(3)}, y: {point.wind_speed.y.toFixed(3)}, z: {point.wind_speed.z.toFixed(3)})<br />
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
                    <div className="grid-toggle-div">
                      <label>
                        <input
                          type="checkbox"
                          className = "toggle-button"
                          checked={showGrid}
                          onChange={() => toggleGrid()}
                        />
                        Show Grid
                      </label>
                    </div>
                    {!showGrid ? (
                    <>
                      <div className="robot-simulation-inputs">
                        <label>Select the robot to set the coordinates</label>
                        <br />
                        {robots.map((robot, idx) => {
                          if (selectedRobotNumber > idx) {
                            return (
                              <button
                                className="number-of-robots-button"
                                key={idx}
                                onClick={() => setSelectedRobotIdx(idx)}
                                style={{
                                  background: selectedRobotIdx === idx ? 'lightblue' : 'white',
                                }}
                              >
                              {idx + 1} 
                              </button>
                            );
                          }
                          return null; 
                        })}
                      </div>
 
                   <GifWithGrid
                    gifObj={gifObj}
                    simulationBounds={simulationBounds}
                    robots={robots}
                    selectedRobotIdx={selectedRobotIdx}
                    grid={false}
                    deviation={deviationSet}
                    numberOfRobots={selectedRobotNumber}
                    type={robotSimulationMode}
                    onSetRobotCoords={(x, y, xFinal, yFinal) => {
                      if (selectedRobotIdx !== null) {
                        if (x !== null) {
                          handleRobotInputChange(selectedRobotIdx, 'robotXlocation', x.toFixed(1));
                          handleRobotInputChange(selectedRobotIdx, 'robotYlocation', y.toFixed(1));
                          
                        } else {
                          handleRobotInputChange(selectedRobotIdx, 'finalRobotXlocation', xFinal.toFixed(1));
                          handleRobotInputChange(selectedRobotIdx, 'finalRobotYlocation', yFinal.toFixed(1));
                        }
                      }
                    }}
                  />
                  </>
                    ) : (
                      <>
                      <div className="robot-simulation-inputs">
                        <label>Select the robot to set the coordinates</label>
                        <br />
                        {robots.map((robot, idx) => {
                          if (selectedRobotNumber > idx) {
                            return (
                              <button
                                className="number-of-robots-button"
                                key={idx}
                                onClick={() => setSelectedRobotIdx(idx)}
                                style={{
                                  background: selectedRobotIdx === idx ? 'lightblue' : 'white',
                                }}
                              >
                              {idx + 1} 
                              </button>
                            );
                          }
                          return null; 
                        })}
                      </div>
 
                      <GifWithGrid
                        gifObj={gifObj}
                        simulationBounds={simulationBounds}
                        robots={robots}
                        selectedRobotIdx={selectedRobotIdx}
                        grid={true}
                        numberOfRobots={selectedRobotNumber}
                        type={robotSimulationMode}
                        deviation={deviationSet}
                        onSetRobotCoords={(x, y, xFinal, yFinal) => {
                          if (selectedRobotIdx !== null) {
                            if (x !== null) {
                              handleRobotInputChange(selectedRobotIdx, 'robotXlocation', x.toFixed(1));
                              handleRobotInputChange(selectedRobotIdx, 'robotYlocation', y.toFixed(1));
                              
                            } else {
                              handleRobotInputChange(selectedRobotIdx, 'finalRobotXlocation', xFinal.toFixed(1));
                              handleRobotInputChange(selectedRobotIdx, 'finalRobotYlocation', yFinal.toFixed(1));
                            }
                          }
                        }}
                      />
                 </>
                    )}
                    
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
                  <label>Number of robots</label>
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
                  <div className="robot-simulation-modes">
                      <button
                        type="button"
                        className={`simulation-mode-button ${robotSimulationMode === 'linear' ? 'selected' : ''}`}
                        onClick={() => setRobotSimulationMode('linear')}
                      >
                        Linear Simulation
                      </button>
                      <button
                        type="button"
                        className={`simulation-mode-button ${robotSimulationMode === 'moth' ? 'selected' : ''}`}
                        onClick={() => setRobotSimulationMode('moth')}
                      >
                        Silkworm Moth Simulation
                      </button>
                      <button
                        type="button"
                        className={`simulation-mode-button ${robotSimulationMode === 'pso' ? 'selected' : ''}`}
                        onClick={() => setRobotSimulationMode('pso')}
                      >
                        Particle swarm optimization
                      </button>
                    </div>
                  </div>
                  <br />
                  <label>
                    Starting ambient Iteration
                    <HoverComponent text="Iteration of the plume simulation that the robot will start Default: 0" />  
                  </label>
                    <input 
                      type="integer"
                      value={startingIteration}
                      onChange={e => handleStartingIterationInputChange(e.target.value)}
                    />
                  <label>
                    Number of simulations
                    <HoverComponent text="Number of robot Simulations Default: 1" />  
                  </label>
                    <input 
                      type="integer"
                      value={numRobotSimulations}
                      onChange={e => handleNumRobotSimulationsChange(e.target.value)}
                    />
                  <label>
                    Name of the simulations set
                    <HoverComponent text="Name that the set of simulations will have" />  
                  </label>
                    <input 
                      type="text"
                      value={nameSimulationSet}
                      onChange={e => handleNameSimulationSetChange(e.target.value)}
                    />
                    <label>
                    Deviation from selected points
                    <HoverComponent text="Random deviation from the point that the simulation will have Default: 0" />  
                  </label>
                    <input 
                      type="number"
                      step="0.1"
                      value={deviationSet}
                      onChange={e => handleDeviationSetChange(e.target.value)}
                    />
                   {['pso'].includes(robotSimulationMode) && (
                        <>
                          <label>
                            Final iteration
                            <HoverComponent text="Iteration when the robot simulation stops" />
                          </label>
                          <input 
                            type="integer"
                            value={psoSimulationIterations}
                            onChange={e => handlePsoIterationsInputChange(e.target.value)}
                          />
                          <div className="ros-checkbox-label">
                            <input 
                              className="toggle-button"
                              type="checkbox"
                              checked={useRos}  
                              onChange={() => toggleRos()} 
                            />
                            <label className="ros-lable">ROS <HoverComponent text="Use ROS in the communication between robots (slower)" /></label>
                          </div>
                        </>
                      )}
                  {[...Array(selectedRobotNumber)].map((_, idx) => (
                    <div key={idx} className="robot-params-input">
                      <h4>Robot {idx + 1}</h4>

                      <label>
                        Robot Speed
                        <HoverComponent text="Speed of the robot in m/s" />
                      </label>
                      <input
                        type="number"
                        value={robots[idx].robotSpeed}
                        step="0.1"
                        onChange={e => handleRobotInputChange(idx, 'robotSpeed', e.target.value)}
                      />
                      <label>
                        Initial robot X coordinate, range: {simulationBounds.xMin} : {simulationBounds.xMax}
                        <HoverComponent text="X coordinate where the robot begins the simulation" />
                      </label>
                      <input
                        type="number"
                        value={robots[idx].robotXlocation}
                        step="0.1"
                        min = {simulationBounds.xMin}
                        max = {simulationBounds.xMax}
                        onChange={e => handleRobotInputChange(idx, 'robotXlocation', e.target.value)}
                      />
                      <label>
                        Initial robot Y coordinate, range: {simulationBounds.yMin} : {simulationBounds.yMax}
                        <HoverComponent text="Y coordinate where the robot begins the simulation" />
                      </label>
                      <input
                        type="number"
                        value={robots[idx].robotYlocation}
                        step="0.1"
                        min = {simulationBounds.yMin}
                        max = {simulationBounds.yMax}
                        onChange={e => handleRobotInputChange(idx, 'robotYlocation', e.target.value)}
                      />
                      
                      {robotSimulationMode === 'linear' && (
                        <>
                          <label>
                            Final robot X coordinate, range: {simulationBounds.xMin} : {simulationBounds.xMax}
                            <HoverComponent text="X coordinate where the robot ends the simulation" />
                          </label>
                          <input
                            type="number"
                            value={robots[idx].finalRobotXlocation}
                            step="0.1"
                            min = {simulationBounds.xMin}
                            max = {simulationBounds.xMax}
                            onChange={e => handleRobotInputChange(idx, 'finalRobotXlocation', e.target.value)}
                          />
                          <label>
                            Final robot Y coordinate, range: {simulationBounds.yMin} : {simulationBounds.yMax}
                            <HoverComponent text="Y coordinate where the robot ends the simulation" />
                          </label>
                          <input
                            type="number"
                            value={robots[idx].finalRobotYlocation}
                            step="0.1"
                            min = {simulationBounds.yMin}
                            max = {simulationBounds.yMax}
                            onChange={e => handleRobotInputChange(idx, 'finalRobotYlocation', e.target.value)}
                          />
                        </>
                      )}
                      {robotSimulationMode === 'moth' && (
                        <>
                          <label>
                            Angle
                            <HoverComponent text="Angle the robot will move in relation with the currrent vector in radians" />
                          </label>
                          <input
                            type="number"
                            value={robots[idx].angle}
                            onChange={e => handleRobotInputChange(idx, 'angle', e.target.value)}
                            min="0"
                            max={2 * Math.PI}
                            step="0.01"
                          />
                        </>
                      )}
                      {['moth'].includes(robotSimulationMode) && (
                        <>
                          <label>Final iteration</label>
                          <input 
                            type="integer"
                            value={robots[idx].iterations}
                            onChange={e => handleRobotInputChange(idx, 'iterations', e.target.value)}
                            placeholder="Iteration when the robot will stop"
                          />
                        </>
                      )}
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
