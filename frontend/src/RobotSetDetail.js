import React, { useState,useEffect } from 'react';
import GifWithGrid from './GifWithGrid'; 
import InfoModal from './InfoModal';  
import './RobotSetDetail.css'
import { Line , Bar} from 'react-chartjs-2';
import SimpleBarChart from './SimpleBarChart';
import { Chart, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend } from 'chart.js';
import Trajectory from './Trajectory';
import PositionConcentrationChart from './PositionConcentrationChart';
import CollapseSectionGroup from './CollapseSection';
import RobotDensityHeatmap from './RobotDensityHeatmap';
import CompassRose from './CompassRose';
import axios from 'axios';
Chart.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend);


const RobotSetDetail = ({
  showRobotSetDetail,
  fadeOut,
  handleGoBackRobotSetDetail,
  pagePath,
  gifsInSet,
  selectedSetSimId,
  setSelectedSetSimId,
  currentIteration,
  handleIterationBackGaden,
  handlePauseResume,
  handleIterationForwardGaden,
  handleChangeSimulationSpeedGaden,
  gadenSimulationSpeed,
  minIteration,
  maxIteration,
  isPausedGaden,
  showGrid,
  deviationSet,
  robots,
  selectedRobotIdx,
  selectedRobotNumber,
  robotSimulationMode,
  selectedRobotFilter,
  setSelectedRobotFilter,
  robotNumbers,
  showTotalStatsRobotSim,
  setShowTotalStatsRobotSim,
  showInfoModal,
  setPagePath,
  setCurrentIteration,
  setGifs,
  medianTime,
  simulationBounds,
  activeRobotButton,
  handleRobotToggleButton
}) => {
  const [distanceFromSource, setDistanceFromSource] = useState(0);
  const [plumeLocation, setPlumeLocation] = useState(null);
  const [robotCloseToSourceStats, setRobotCloseToSourceStats] = useState({});
  const [activeStatsSection, setActiveStatsSection] = useState(0);


  const simGifs = gifsInSet.filter(gifObj => gifObj.robotSim_id !== undefined && gifObj.robotSim_id !== null);

  const totalTime = simGifs.reduce((acc, gif) => acc + (gif.time ?? 0), 0);
  const totalIterations = simGifs.length;
  const avgTime = totalIterations > 0 ? (totalTime / totalIterations).toFixed(2) : 0;

  const simIds = Array.from(new Set(simGifs.map(g => g.robotSim_id)));
  const totalIterationsPerSim = simIds.reduce((acc, simId) => {
    const simIterations = simGifs.filter(g => g.robotSim_id === simId);
    const maxIteration = Math.max(...simIterations.map(g => g.iteration));
    return acc + maxIteration;
  }, 0);
  const avgIterationsPerSim = simIds.length > 0 ? (totalIterationsPerSim / simIds.length).toFixed(2) : 0;

  const simGifsForSelected = gifsInSet
  .filter(gifObj => gifObj.robotSim_id === selectedSetSimId)
  .sort((a, b) => a.iteration - b.iteration);
  const robotConcentrationData = simGifsForSelected.flatMap(gifObj =>
    (gifObj.robot_path)
      .filter(point => gifObj.iteration === currentIteration && (selectedRobotFilter === 'all' || String(point.robot) === String(selectedRobotFilter)))
      .map(point => ({
        robot: point.robot,
        concentration: Number(point.concentration)
      }))
  );

  const robotPathsMap = {};
  simGifsForSelected.forEach(gifObj => {
    (gifObj.robot_path || []).forEach(point => {
      if (
        selectedRobotFilter === 'all' ||
        String(point.robot) === String(selectedRobotFilter)
      ) {
        if (!robotPathsMap[point.robot]) robotPathsMap[point.robot] = [];
        robotPathsMap[point.robot].push(point);
      }
    });
  });
  const robotPaths = Object.values(robotPathsMap);
  const robotDistances = {};
  const robotRealDistances = {};

  if (simGifsForSelected.length > 0 || selectedSetSimId === -1) {
    const relevantGifs = selectedSetSimId === -1
      ? gifsInSet.filter(g => g.robotSim_id !== undefined && g.robotSim_id !== null)
      : simGifsForSelected;

    const groupedBySim = {};

    relevantGifs.forEach(gif => {
      const simId = gif.robotSim_id;
      if (!groupedBySim[simId]) groupedBySim[simId] = [];
      groupedBySim[simId].push(gif);
    });

    const allRobotDistances = {};
    const allRobotTraveled = {};

    Object.entries(groupedBySim).forEach(([simId, simGifs]) => {
      const simIterations = simGifs.sort((a, b) => a.iteration - b.iteration);
      const allIterations = simIterations.map(gif => gif.robot_path || []);
      const firstIteration = allIterations[0];
      const lastIteration = allIterations[allIterations.length - 1];

      if (!firstIteration || !lastIteration) return;

      firstIteration.forEach(startPoint => {
        const robotId = startPoint.robot;
        const endPoint = lastIteration.find(p => p.robot === robotId);
        if (!endPoint) return;

        const dx = endPoint.robot_position.x - startPoint.robot_position.x;
        const dy = endPoint.robot_position.y - startPoint.robot_position.y;
        const dz = endPoint.robot_position.z - startPoint.robot_position.z;
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

        if (!allRobotDistances[robotId]) allRobotDistances[robotId] = [];
        allRobotDistances[robotId].push(dist);

        let traveled = 0;
        for (let i = 1; i < allIterations.length; i++) {
          const prev = allIterations[i - 1].find(p => p.robot === robotId);
          const curr = allIterations[i].find(p => p.robot === robotId);
          if (prev && curr) {
            const dx = curr.robot_position.x - prev.robot_position.x;
            const dy = curr.robot_position.y - prev.robot_position.y;
            const dz = curr.robot_position.z - prev.robot_position.z;
            traveled += Math.sqrt(dx * dx + dy * dy + dz * dz);
          }
        }

        if (!allRobotTraveled[robotId]) allRobotTraveled[robotId] = [];
        allRobotTraveled[robotId].push(traveled);
      });
    });

    Object.entries(allRobotDistances).forEach(([robot, distances]) => {
      const avg = distances.reduce((a, b) => a + b, 0) / distances.length;
      robotDistances[robot] = avg.toFixed(2);
    });

    Object.entries(allRobotTraveled).forEach(([robot, distances]) => {
      const avg = distances.reduce((a, b) => a + b, 0) / distances.length;
      robotRealDistances[robot] = avg.toFixed(2);
    });
  }

  useEffect(() => {
    if (selectedSetSimId === -1 && plumeLocation && distanceFromSource) {
      const [plumeX, plumeY, plumeZ] = plumeLocation.split('/').map(Number);

      const newRobotStats = {};

      gifsInSet.forEach((gif) => {
        const { robotSim_id, iteration, robot_path } = gif;

        if (iteration === Math.max(...gifsInSet.filter(g => g.robotSim_id === robotSim_id).map(g => g.iteration))) {
          
          robot_path.forEach((point) => {
            const { robot, robot_position } = point;

            const dx = robot_position.x - plumeX;
            const dy = robot_position.y - plumeY;
            const dz = robot_position.z - plumeZ;
            const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (!newRobotStats[robot]) {
              newRobotStats[robot] = { closeCount: 0, totalCount: 0 };
            }

            newRobotStats[robot].totalCount++;

            if (distance <= distanceFromSource) {
              newRobotStats[robot].closeCount++;
            }
          });
        }
      });

      const updatedStats = Object.entries(newRobotStats).reduce((acc, [robotId, { closeCount, totalCount }]) => {
        const percentage = totalCount > 0 ? ((closeCount / totalCount) * 100).toFixed(2) : 0;
        acc[robotId] = { closeCount, totalCount, percentage };
        return acc;
      }, {});

      setRobotCloseToSourceStats(updatedStats);
    }
  }, [gifsInSet, selectedSetSimId, plumeLocation, distanceFromSource]);

   useEffect(() => {
    if (gifsInSet.length > 0 && selectedSetSimId === -1) {
      const simulationId = gifsInSet[0]?.simulation;
      axios
        .get(`http://localhost:3000/getPlumeLocation?simulation=${simulationId}`)
        .then(response => {
          setPlumeLocation(response.data.plume_location);
        })
        .catch(error => {
          console.error('Error fetching plume location:', error);
          setPlumeLocation(null);
        });
    }
  }, [gifsInSet, selectedSetSimId]);

  useEffect(() => {
    if (activeRobotButton !== "stats" && selectedSetSimId === -1) {
      const validSimIds = Array.from(
        new Set(gifsInSet.map(g => g.robotSim_id).filter(id => id !== undefined && id !== null))
      );
      if (validSimIds.length > 0) {
        const firstSimId = validSimIds[0];
        setSelectedSetSimId(firstSimId);
        setCurrentIteration(0);
        setGifs(gifsInSet.filter(g => g.robotSim_id === firstSimId));
      }
    }
  }, [activeRobotButton, selectedSetSimId, gifsInSet]);



  const calculateConcentrationPercentage = (simGifs) => {
    const robotConcentrationPercentages = {};

    simGifs.forEach(gifObj => {
      if (gifObj.robot_path) {
        gifObj.robot_path.forEach(point => {
          const robotId = point.robot;
          const concentrationAboveZero = point.concentration > 0;

          if (!robotConcentrationPercentages[robotId]) {
            robotConcentrationPercentages[robotId] = { total: 0, withConcentration: 0 };
          }

          robotConcentrationPercentages[robotId].total += 1;
          if (concentrationAboveZero) {
            robotConcentrationPercentages[robotId].withConcentration += 1;
          }
        });
      }
    });

    Object.entries(robotConcentrationPercentages).forEach(([robotId, data]) => {
      const { total, withConcentration } = data;
      const percentage = total > 0 ? ((withConcentration / total) * 100).toFixed(2) : 0;
      robotConcentrationPercentages[robotId].percentage = percentage;
    });

    return robotConcentrationPercentages;
  };


  const concentrationPercentages = selectedSetSimId === -1
    ? calculateConcentrationPercentage(gifsInSet)
    : calculateConcentrationPercentage(simGifsForSelected);
  return (
    <>
      {showRobotSetDetail && (
        <div className="gaden-simulation-click">
          <button
            className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
            onClick={() => {
              handleGoBackRobotSetDetail();
              setPagePath(pagePath.slice(0, -1));
            }}
          >
            Go Back
          </button>
          <div className="toggle-buttons">
            <button
              className={`toggle-button ${activeRobotButton === 'visual' ? 'active' : 'inactive'}`}
              onClick={() => handleRobotToggleButton('visual')}
            >
              Visual Path
            </button>
            <button
              className={`toggle-button ${activeRobotButton === 'path' ? 'active' : 'inactive'}`}
              onClick={() => handleRobotToggleButton('path')}
            >
              Path
            </button>
            <button
              className={`toggle-button ${activeRobotButton === 'stats' ? 'active' : 'inactive'}`}
              onClick={() => handleRobotToggleButton('stats')}
            >
              Stats
            </button>
          </div>
          <div className="content-container">
            
            <div className="gif-description-robot">
              {activeRobotButton === "stats" && (
                      <label>
                      Select Simulation in Set:&nbsp;
                      <select
                        value={selectedSetSimId}
                        onChange={e => {
                          const simId = Number(e.target.value);
                          setSelectedSetSimId(simId);
                          setCurrentIteration(0);
                          setGifs(gifsInSet.filter(g => simId === -1 || g.robotSim_id === simId));
                        }}
                      >
                         <option value={-1}>All Experiences</option>
                        {Array.from(new Set(gifsInSet.map(g => g.robotSim_id)))
                          .filter(id => id !== undefined && id !== null)
                          .map((id, index) => (
                            <option key={id} value={id}>
                              Simulation {index + 1}
                            </option>
                          ))}
                      </select>
                    </label>      
                        )}
                        {activeRobotButton == "stats" && (
                        <>
                          <h3>Average Iterations per Simulation: {avgIterationsPerSim}</h3>
                        </>
                      )}
              {gifsInSet
                
                .filter(gifObj => gifObj.robotSim_id === selectedSetSimId && gifObj.iteration === currentIteration)
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">                      
                      <div className="Simulation-set-select-container">
                    {activeRobotButton != "stats" &&(
                      <label>
                      Select Simulation in Set:&nbsp;
                      <select
                        value={selectedSetSimId}
                        onChange={e => {
                          const simId = Number(e.target.value);
                          setSelectedSetSimId(simId);
                          setCurrentIteration(0);
                          setGifs(gifsInSet.filter(g => simId === -1 || g.robotSim_id === simId));
                        }}
                      >

                        {Array.from(new Set(gifsInSet.map(g => g.robotSim_id)))
                          .filter(id => id !== undefined && id !== null)
                          .map((id, index) => (
                            <option key={id} value={id}>
                              Simulation {index + 1}
                            </option>
                          ))}
                      </select>
                    </label>
                    )}
 
                    </div>
                    {activeRobotButton == 'visual' && (
                      <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                    )}
                    {activeRobotButton == 'stats' &&(
                      <h3>Average time per iteration: {medianTime}</h3>
                    )}
                    {activeRobotButton == "visual" &&(
                      <>
                    <h3>Iteration: {gifObj.iteration}</h3>
                    <div className="visual-container">
                      
                                     
                    <div className="gif-control-container">
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
                    <div className="bar-chart-container">
                      <SimpleBarChart data={robotConcentrationData} />
                    </div> 
                    </div>               
                    </>
                    )}

                  </div> 
                ))}
                 
            </div>   
            </div>
            {activeRobotButton == "path" && (
          
              <div className="path-visualization-container">
             <div className="chart-with-buttons">
              <PositionConcentrationChart
                gifsInSet={gifsInSet}
                selectedSetSimId={selectedSetSimId}
                currentIteration={currentIteration}
                selectedRobotFilter={selectedRobotFilter}
                showTotalStatsRobotSim={showTotalStatsRobotSim}
                simulationBounds={simulationBounds}
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
              <div className="robot-path-list-container">
                <h4>Robot Path:</h4>
                <div className="checkbox-filters">
                  <label>
                    <input
                      type="checkbox"
                      checked={showTotalStatsRobotSim}
                      onChange={() => setShowTotalStatsRobotSim(prev => !prev)}
                    />
                    show total stats
                  </label>
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
            )}
         {activeRobotButton === 'stats' && (
        <CollapseSectionGroup
          sections={[
            {
              buttonLabel: "Iteration Time Stats",
              children: (
                <div className="stats-graph-container">
                  <Line
                    data={{
                      labels: selectedSetSimId === -1
                        ? Array.from(
                            new Set(gifsInSet.map(g => g.robotSim_id))
                          ).map((simId, idx) => `Simulation ${idx + 1}`)
                        : gifsInSet
                            .filter(gifObj => gifObj.robotSim_id === selectedSetSimId)
                            .sort((a, b) => a.iteration - b.iteration)
                            .map(gifObj => gifObj.iteration),
                      datasets: [
                        {
                          label: selectedSetSimId === -1 ? 'Total Time per Simulation' : 'Time per Iteration',
                          data: selectedSetSimId === -1
                            ? Array.from(new Set(gifsInSet.map(g => g.robotSim_id)))
                                .map(simId =>
                                  gifsInSet
                                    .filter(g => g.robotSim_id === simId)
                                    .reduce((sum, g) => sum + (g.time ?? 0), 0)
                                )
                            : gifsInSet
                                .filter(gifObj => gifObj.robotSim_id === selectedSetSimId)
                                .map(gifObj => gifObj.time),
                          borderColor: 'rgba(190, 11, 11, 1)',
                          backgroundColor: 'rgba(75,192,192,0.2)',
                          fill: true,
                          tension: 0.1,
                        }
                      ]
                    }}
                    options={{
                      responsive: true,
                      plugins: {
                        legend: { display: true },
                        title: {
                          display: true,
                          text: selectedSetSimId === -1
                            ? 'Total Time per Simulation'
                            : 'Time per Iteration'
                        },
                      },
                      scales: {
                        x: {
                          title: {
                            display: true,
                            text: selectedSetSimId === -1 ? 'Simulation' : 'Iteration',
                          },
                        },
                        y: {
                          title: {
                            display: true,
                            text: 'Time (s)',
                          },
                        },
                      },
                    }}
                  />
                </div>
              ),
            },
            {
              buttonLabel: "Density Map",
              children: (
                <div className='density-container'>
                  <RobotDensityHeatmap
                    gifsInSet={
                      selectedSetSimId === -1
                        ? gifsInSet.filter(g => g.robotSim_id !== undefined && g.robotSim_id !== null)
                        : gifsInSet.filter(g => g.robotSim_id === selectedSetSimId)
                    }
                    selectedSetSimId={selectedSetSimId}
                    simulationBounds={simulationBounds}
                  />
                </div>
              ),
            },
            {
              buttonLabel: "Average Robot Direction",
              children: (
                <CompassRose
                  gifsInSet={gifsInSet}
                  selectedSetSimId={selectedSetSimId}
                  selectedRobotFilter={'all'}
                />
              ),
            },
            {
              buttonLabel: "Distance Stats",
              children: (
                <div className="distance-stats-container">
                  <h3>Robot Distances:</h3>
                  <ul>
                    {Object.entries(robotDistances).map(([robot, distance]) => (
                      <li key={robot}>
                        Robot {robot}: Straight-line = {distance} m, Traveled = {robotRealDistances[robot]} m
                      </li>
                    ))}
                  </ul>
                </div>
              ),
            },
            {
              buttonLabel: "Plume Detection Stats",
              children: (
                <div className="concentration-stats-container">
                  <h3>Robot Concentration Stats:</h3>
                  <ul>
                    {Object.entries(concentrationPercentages).map(([robot, data]) => (
                      <li key={robot}>
                        Robot {robot}: {data.withConcentration}/{data.total} iterations had concentration &gt; 0 ({data.percentage}%)
                      </li>
                    ))}
                  </ul>
                </div>
              ),
            },
            ...(selectedSetSimId === -1 && plumeLocation
              ? [{
                  buttonLabel: "Experience Success Rate",
                  children: (
                    <div className="success-rate-container">
                      <div>
                        <label htmlFor="distance-from-source">Maximum distance to be considered a successful experience</label>
                        <input
                          type="number"
                          id="distance-from-source"
                          value={distanceFromSource}
                          onChange={(e) => setDistanceFromSource(e.target.value)}
                          placeholder="Enter distance"
                        />
                      </div>
                      <div className="robot-close-to-source-stats">
                        <h3>Robot Success Rate (Percentage of Successful Experiences):</h3>
                        <ul>
                          {Object.entries(robotCloseToSourceStats).map(([robotSimId, { closeCount, totalCount, percentage }]) => (
                            <li key={robotSimId}>
                              Robot {robotSimId}: {closeCount}/{totalCount} successful experiences ({percentage}%) 
                            </li>
                          ))}
                        </ul>
                      </div>
                    </div>
                  ),
                }]
              : []
            ),
          ]}
          activeSection={activeStatsSection}
          setActiveSection={setActiveStatsSection}
        />
      )}

      {showInfoModal && (
        <InfoModal
          message="Loading Set"
        />
      )}
        </div>
      )}
    </>
  );
};

export default RobotSetDetail;