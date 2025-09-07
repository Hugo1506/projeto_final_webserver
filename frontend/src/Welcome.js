import React, { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import './Welcome.css';
import logo from './flyrobotics_logo.png'; 
import PagePath from './PagePath';
import PlumeOrExperiences from './PlumeOrExperiences';
import GadenChoiseButtons from './GadenChoiseButtons';
import AmbientSubmit from './AmbientSubmit';
import PlumeLocationForm from './PlumeLocationForm';
import SavedSimulationsPage from './SavedSimulations';
import SimulationDetail from './SimulationDetail';
import RobotSetDetail from './RobotSetDetail';
import GadenSimulationClick from './GadenSimulationClick';
import SimulationDetailNoRobot from './SimulationDetailNoRobot'
import GadenSimulationClickNoRobot from './GadenSimulationClickNoRobot';

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
  const [showInfoModal, setShowInfoModal] = useState(false);
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
  const [pagePath, setPagePath] = useState(["Home"]);
  const [plumeOrExperienciesVisible, setPlumeOrExperienciesVisible] = useState(false);
  const [showRobotSimulationsSet, setShowRobotSimulationSet] = useState(false);
  const [activeRobotButton, setActiveRobotButton] = useState("visual");

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
    setPagePath(prev => [...prev, "Gaden"])
    setFadeOut(true);
    setTimeout(() => {
      setPlumeOrExperienciesVisible(true);
      setAmbientSimulator("Simulator: Gaden version 2.5.0");
      setFadeOut(false);
    }, 500);
  };


  const handleNewSimulationClick = () => {
    setPagePath([...pagePath,"New Simulation"])
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
      setPagePath([...pagePath, "Saved Simulations"]);
    }
  };

  const handleGoBack = () => {
    setFileInputVisible(false);
    setGadenChoiseVisible(false);
    setIsNewSimulation(false);
    setSavedSimulationsVisible(false);
    setAmbientSimulator("");
    setSimulationDetail(false);
    setShowRobotSetDetail(false);
    setPlumeOrExperienciesVisible(true)
  };

  const handleGoBackGadenChoise = () => {
    if(pagePath.includes("Experiences")){
      setPlumeOrExperienciesVisible(true);   
      setPagePath(prev => [...prev.slice(0, -2)]);
    }else{
      setGadenChoiseVisible(true);
    }
    
    setIsNewSimulation(false);
    setSavedSimulationsVisible(false);
    setShowPlumeLocation(false);
    setFadeOut(false); 
    setSimulationDetail(false);
    setShowRobotSetDetail(false);
  };

  const handleSavedSimulationsClick = async () => {
    await fetchSavedSimulations();
    setPagePath(prev => [...prev, "Saved Simulations"]);
    setFadeOut(true); 
    setTimeout(() => {
      setGadenChoiseVisible(false); 
      setIsNewSimulation(false); 
      setPlumeOrExperienciesVisible(false);
      setSavedSimulationsVisible(true);
      setFadeOut(false)
    }, 500);
  };


  const handlePlumeClick = async () => {
    await fetchSavedSimulations();
    setPagePath(prev => [...prev, "Plume Simulation"]);
    setShowRobotSimulationSet(false);
    setFadeOut(true); 
    setTimeout(() => {
      setIsNewSimulation(false); 
      setPlumeOrExperienciesVisible(false);
      setGadenChoiseVisible(true);
      setFadeOut(false)
    }, 500);
  }

  const handleExperiencesClick = async () => {
    setPagePath(prev => [...prev, "Experiences"]);
    await fetchSavedSimulations();
    setShowRobotSimulationSet(true);
    setFadeOut(true); 
    setTimeout(() => {
      setIsNewSimulation(false); 
      handleSavedSimulationsClick();
      setFadeOut(false)
    }, 500);
  }
  const handleSetClick = async (set) => {
    setGifs([]);
    setGifsInSet([]); 
    setSelectedSetSimId(null);
    setShowRobotSetDetail(true); 
    setSimulationDetail(false);
    setParentSimulationOfSet(set.simulation)

    setShowInfoModal(true);
    const newGifs = await fetchGifsFromSet(set);
    setGifsInSet(newGifs);
    setShowInfoModal(false);
    setPagePath(prev => [...prev, "Simulation Set"]);

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

    setShowInfoModal(true);
    await fetchGifsFromResults(simulation);
    setShowInfoModal(false);
    setFadeOut(true);
    setPagePath(prev => [...prev, "Enviroment Simulation"]);
    setTimeout(() => {
      setSavedSimulationsVisible(false);
      setPlumeOrExperienciesVisible(false);
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
    setSavedSimulationsVisible(true);
    setSimulationDetail(false);
    setFadeOut(false)
    setSearchQuery('');
    setFilteredSimulations(savedSimulations);
    setShowRobotSetDetail(false);
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

      setPagePath(prev => [...prev, "Clicked Gaden Simulation"]);
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
    setPagePath(prev => [...prev.slice(0, -1)]);  
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
    } catch (error) {
        console.error('Error during simulation:', error);
    } finally {
        setRobotSimulationIsLoading(false);
        setGadenSimulationClickVisible(false);
        handleToggleButton('robot');   
        setSimulationDetail(true);
        await handleSimulationClick(simulation); 
        
        setPagePath(prev => [...prev.slice(0, -1)]);  
         
    }
    
}

  const handleToggleButton = (button) => {
    setActiveButton(button);
    if (button === 'gaden') {
      setActiveButton('gaden');
      setShowCheckboxes(true);
      setRobotSimulation(true);
      setPagePath(prevPath => [...prevPath.slice(0, -1), "Enviroment Simulation"]);
    } else {
      setActiveButton('robot');
      setShowCheckboxes(false);
      setRobotSimulation(false);
      fetchRobotSetData(clickedGif?.simulation || (filteredGifs[0] && filteredGifs[0].simulation));
      setPagePath(prevPath => [...prevPath.slice(0, -1), "Robot Simulations"]);
    }
  }

  const handleRobotToggleButton = (button) => {
    setActiveRobotButton(button);
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
    if (gifsInSet && gifsInSet.length > 0 && selectedSetSimId != null) {
      const selectedGifs = gifsInSet.filter(g => g.robotSim_id === selectedSetSimId);
      const median = getMedianTime(selectedGifs);
      setMedianTime(median);
      setMaxIteration(Math.max(...filteredGifs.map(g => g.iteration)))
    }
  }, [gifsInSet, selectedSetSimId]);

    const handlePathClick = (index) => {
        setPagePath(pagePath.slice(0, index + 1));

        switch (pagePath[index]) {
                case "Home":
                    handleGoBack();
                    break;
                
                case "Gaden":
                    handleGoBackGadenChoise();
                    break;

                case "Saved Simulations":
                    handleGoBackSavedSimulations();
                    break;

                case "Enviroment Simulation":
                    handleGoBackRobotSetDetail();
                    handleGoBackSimulationDetails();
                    break;
                
                default:
                    break;

        };
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
        <PagePath
          path={pagePath}
          onPathClick={handlePathClick}
        />
        <img
          src={logo}
          alt="FlyRobotics Logo"
          className="flyrobotics-logo"
        />
      </div>
      <div className="main-content">
        {!plumeOrExperienciesVisible && !showRobotSetDetail && !gadenSimulationClickVisible && !fileInputVisible && !GadenChoiseVisible && !isNewSimulation && !savedSimulationsVisible && !simulationDetail? (
          <button
            className={`gaden-button ${fadeOut ? 'fade-out' : ''}`}
            onClick={handleGadenClick}
          >
            Gaden <br /> version: 2.5.0
          </button>
        ) : null}
        {plumeOrExperienciesVisible && (
          <PlumeOrExperiences
            ambientSimulator={ambientSimulator}
            fadeOut={fadeOut}
            handlePlumeClick={handlePlumeClick}
            handleExperiencesClick={handleExperiencesClick}
            setPlumeOrExperienciesVisible={setPlumeOrExperienciesVisible}
            pagePath={pagePath}
            setPagePath={setPagePath}
            setShowRobotSimulationSet={setShowRobotSimulationSet}
          />
        )}
       {GadenChoiseVisible && (
        <GadenChoiseButtons 
          ambientSimulator={ambientSimulator}
          fadeOut={fadeOut}
          handleNewSimulationClick={handleNewSimulationClick}
          handleSavedSimulationsClick={handleSavedSimulationsClick}
          handleGoBack={handleGoBack}
          setPagePath={setPagePath}
        />
    )}
      {isNewSimulation && !showPlumeLocation && (
        <AmbientSubmit
          simulationName={simulationName}
          setSimulationName={setSimulationName}
          handleFileSubmit={handleFileSubmit}
          handleFileChange={handleFileChange}
          handleGoBackGadenChoise={handleGoBackGadenChoise}
          setPagePath={setPagePath}
          pagePath={pagePath}
          fadeOut={fadeOut}
          enviromentIsLoading={enviromentIsLoading}
          plumeSimulationLoadingText={plumeSimulationLoadingText}
          setFiles={setFiles}
          files={files} 
      />
      )}
      {isNewSimulation && showPlumeLocation && (
        <PlumeLocationForm
          filteredGifs={filteredGifs}
          selectedHeight={selectedHeight}
          setSelectedHeight={setSelectedHeight}
          availableHeights={availableHeights}
          showGrid={showGrid}
          toggleGrid={toggleGrid}
          simulationBounds={simulationBounds}
          setPlumeXLocation={setPlumeXLocation}
          setPlumeYLocation={setPlumeYLocation}
          setPlumeZLocation={setPlumeZLocation}
          plumeXLocation={plumeXLocation}
          plumeYLocation={plumeYLocation}
          plumeZLocation={plumeZLocation}
          temperatureInC={temperatureInC}
          setTemperatureInC={setTemperatureInC}
          handlePlumeSubmit={handlePlumeSubmit}
          showFilamentOptions={showFilamentOptions}
          setShowFilamentOptions={setShowFilamentOptions}
          ppmCenter={ppmCenter}
          setPpmCenter={setPpmCenter}
          numFilamentsSec={numFilamentsSec}
          setNumFilamentsSec={setNumFilamentsSec}
          filamentInitialStd={filamentInitialStd}
          setFilamentInitialStd={setFilamentInitialStd}
          filamentGrowth={filamentGrowth}
          setFilamentGrowth={setFilamentGrowth}
          filamentNoise={filamentNoise}
          setFilamentNoise={setFilamentNoise}
          fadeOut={fadeOut}
          plumeSimulationIsLoading={plumeSimulationIsLoading}
          plumeSimulationLoadingText={plumeSimulationLoadingText}
          handleGoBackGadenChoise={handleGoBackGadenChoise}
          setPagePath={setPagePath}
          pagePath={pagePath}
        />
      )}
        {savedSimulationsVisible && (
          <SavedSimulationsPage
            fadeOut={fadeOut}
            filteredSimulations={filteredSimulations}
            handleSearch={handleSearch}
            handleGoBackGadenChoise={handleGoBackGadenChoise}
            setPagePath={setPagePath}
            pagePath={pagePath}
            handleSimulationClick={handleSimulationClick}
            openDeleteModal={openDeleteModal}
            showModal={showModal}
            confirmDelete={confirmDelete}
            closeModal={closeModal}
            showInfoModal={showInfoModal}
          />
        )}
        {simulationDetail && !showRobotSimulationsSet && (
          <SimulationDetailNoRobot
            simulationDetail={simulationDetail}
            fadeOut={fadeOut}
            activeButton={activeButton}
            setActiveButton={setActiveButton}
            robotSetData={robotSetData}
            filteredRobotSets={filteredRobotSets}
            filteredGifs={filteredGifs}
            checkedOptions={checkedOptions}
            selectedHeight={selectedHeight}
            availableHeights={availableHeights}
            loadingGifs={loadingGifs}
            currentIteration={currentIteration}
            robotSetSearch={robotSetSearch}
            setRobotSetSearch={setRobotSetSearch}
            showModal={showModal}
            showCheckboxes={showCheckboxes}
            setPagePath={setPagePath}
            setSelectedHeight={setSelectedHeight}
            handleGoBackSavedSimulations={handleGoBackSavedSimulations}
            handleSetClick={handleSetClick}
            openSetDeleteModal={openDeleteModal}
            confirmSetDelete={confirmSetDelete}
            closeSetModal={closeSetModal}
            handleCheckboxChange={handleCheckboxChange}
            handleGifClick={handleGifClick}
            handleImageLoaded={handleImageLoaded}
            handleToggleButton={handleToggleButton}
          />
        )}
        {simulationDetail && showRobotSimulationsSet && (
          <SimulationDetail
            simulationDetail={simulationDetail}
            fadeOut={fadeOut}
            activeButton={activeButton}
            setActiveButton={setActiveButton}
            robotSetData={robotSetData}
            filteredRobotSets={filteredRobotSets}
            filteredGifs={filteredGifs}
            checkedOptions={checkedOptions}
            selectedHeight={selectedHeight}
            availableHeights={availableHeights}
            loadingGifs={loadingGifs}
            currentIteration={currentIteration}
            robotSetSearch={robotSetSearch}
            setRobotSetSearch={setRobotSetSearch}
            showModal={showModal}
            showCheckboxes={showCheckboxes}
            setPagePath={setPagePath}
            setSelectedHeight={setSelectedHeight}
            handleGoBackSavedSimulations={handleGoBackSavedSimulations}
            handleSetClick={handleSetClick}
            openSetDeleteModal={openDeleteModal}
            confirmSetDelete={confirmSetDelete}
            closeSetModal={closeSetModal}
            handleCheckboxChange={handleCheckboxChange}
            handleGifClick={handleGifClick}
            handleImageLoaded={handleImageLoaded}
            handleToggleButton={handleToggleButton}
          />
        )}
      {showRobotSetDetail && (
        <RobotSetDetail
          showRobotSetDetail={showRobotSetDetail}
          fadeOut={fadeOut}
          handleGoBackRobotSetDetail={handleGoBackRobotSetDetail}
          pagePath={pagePath}
          gifsInSet={gifsInSet}
          selectedSetSimId={selectedSetSimId}
          setSelectedSetSimId={setSelectedSetSimId}
          currentIteration={currentIteration}
          handleIterationBackGaden={handleIterationBackGaden}
          handlePauseResume={handlePauseResume}
          handleIterationForwardGaden={handleIterationForwardGaden}
          handleChangeSimulationSpeedGaden={handleChangeSimulationSpeedGaden}
          gadenSimulationSpeed={gadenSimulationSpeed}
          minIteration={minIteration}
          maxIteration={maxIteration}
          isPausedGaden={isPausedGaden}
          showGrid={showGrid}
          deviationSet={deviationSet}
          robots={robots}
          selectedRobotIdx={selectedRobotIdx}
          selectedRobotNumber={selectedRobotNumber}
          robotSimulationMode={robotSimulationMode}
          selectedRobotFilter={selectedRobotFilter}
          setSelectedRobotFilter={setSelectedRobotFilter}
          robotNumbers={robotNumbers}
          showTotalStatsRobotSim={showTotalStatsRobotSim}
          setShowTotalStatsRobotSim={setShowTotalStatsRobotSim}
          showInfoModal={showInfoModal}
          setPagePath={setPagePath}
          setCurrentIteration={setCurrentIteration}
          setGifs={setGifs}
          medianTime={medianTime}
          simulationBounds={simulationBounds}
          activeRobotButton={activeRobotButton}
          handleRobotToggleButton={handleRobotToggleButton}
        />
      )}
      </div>
      {gadenSimulationClickVisible && clickedGif && robotSimulation && !showRobotSimulationsSet &&(
        <GadenSimulationClickNoRobot
          gadenSimulationClickVisible={gadenSimulationClickVisible}
          clickedGif={clickedGif}
          robotSimulation={robotSimulation}
          fadeOut={fadeOut}
          handleGoBackSimulationDetails={handleGoBackSimulationDetails}
          pagePath={pagePath}
          setPagePath={setPagePath}
          relatedGifs={relatedGifs}
          currentIteration={currentIteration}
          minIteration={minIteration}
          maxIteration={maxIteration}
          showGrid={showGrid}
          toggleGrid={toggleGrid}
          robots={robots}
          selectedRobotNumber={selectedRobotNumber}
          selectedRobotIdx={selectedRobotIdx}
          setSelectedRobotIdx={setSelectedRobotIdx}
          simulationBounds={simulationBounds}
          deviationSet={deviationSet}
          robotSimulationMode={robotSimulationMode}
          handleRobotInputChange={handleRobotInputChange}
          handleIterationBackGaden={handleIterationBackGaden}
          isPausedGaden={isPausedGaden}
          handlePauseResume={handlePauseResume}
          handleIterationForwardGaden={handleIterationForwardGaden}
          gadenSimulationSpeed={gadenSimulationSpeed}
          handleChangeSimulationSpeedGaden={handleChangeSimulationSpeedGaden}
          setCurrentIteration={setCurrentIteration}
          robotSimulationIsLoading={robotSimulationIsLoading}
          robotSimulationLoadingText={robotSimulationLoadingText}
          handleRobotSimulationSubmit={handleRobotSimulationSubmit}
          startingIteration={startingIteration}
          handleStartingIterationInputChange={handleStartingIterationInputChange}
          numRobotSimulations={numRobotSimulations}
          handleNumRobotSimulationsChange={handleNumRobotSimulationsChange}
          nameSimulationSet={nameSimulationSet}
          handleNameSimulationSetChange={handleNameSimulationSetChange}
          handleDeviationSetChange={handleDeviationSetChange}
          psoSimulationIterations={psoSimulationIterations}
          handlePsoIterationsInputChange={handlePsoIterationsInputChange}
          useRos={useRos}
          toggleRos={toggleRos}
          setSelectedRobotNumber={setSelectedRobotNumber}
          setRobotSimulationMode={setRobotSimulationMode}
        />
      )}
       {gadenSimulationClickVisible && clickedGif && robotSimulation && showRobotSimulationsSet &&(
        <GadenSimulationClick
          gadenSimulationClickVisible={gadenSimulationClickVisible}
          clickedGif={clickedGif}
          robotSimulation={robotSimulation}
          fadeOut={fadeOut}
          handleGoBackSimulationDetails={handleGoBackSimulationDetails}
          pagePath={pagePath}
          setPagePath={setPagePath}
          relatedGifs={relatedGifs}
          currentIteration={currentIteration}
          minIteration={minIteration}
          maxIteration={maxIteration}
          showGrid={showGrid}
          toggleGrid={toggleGrid}
          robots={robots}
          selectedRobotNumber={selectedRobotNumber}
          selectedRobotIdx={selectedRobotIdx}
          setSelectedRobotIdx={setSelectedRobotIdx}
          simulationBounds={simulationBounds}
          deviationSet={deviationSet}
          robotSimulationMode={robotSimulationMode}
          handleRobotInputChange={handleRobotInputChange}
          handleIterationBackGaden={handleIterationBackGaden}
          isPausedGaden={isPausedGaden}
          handlePauseResume={handlePauseResume}
          handleIterationForwardGaden={handleIterationForwardGaden}
          gadenSimulationSpeed={gadenSimulationSpeed}
          handleChangeSimulationSpeedGaden={handleChangeSimulationSpeedGaden}
          setCurrentIteration={setCurrentIteration}
          robotSimulationIsLoading={robotSimulationIsLoading}
          robotSimulationLoadingText={robotSimulationLoadingText}
          handleRobotSimulationSubmit={handleRobotSimulationSubmit}
          startingIteration={startingIteration}
          handleStartingIterationInputChange={handleStartingIterationInputChange}
          numRobotSimulations={numRobotSimulations}
          handleNumRobotSimulationsChange={handleNumRobotSimulationsChange}
          nameSimulationSet={nameSimulationSet}
          handleNameSimulationSetChange={handleNameSimulationSetChange}
          handleDeviationSetChange={handleDeviationSetChange}
          psoSimulationIterations={psoSimulationIterations}
          handlePsoIterationsInputChange={handlePsoIterationsInputChange}
          useRos={useRos}
          toggleRos={toggleRos}
          setSelectedRobotNumber={setSelectedRobotNumber}
          setRobotSimulationMode={setRobotSimulationMode}
          setRobots={setRobots}
        />
      )}
    </div>
  

  );
};

export default Welcome;
