import React, { useState,useEffect } from 'react';
import GifWithGrid from './GifWithGrid'; 
import InfoModal from './InfoModal';  
import './RobotSetDetail.css'
import { Line , Bar} from 'react-chartjs-2';
import SimpleBarChart from './SimpleBarChart';
import { Chart, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend } from 'chart.js';
import Trajectory from './Trajectory'
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
  if (simGifsForSelected.length > 0) {
    const firstIteration = simGifsForSelected[0].robot_path || [];
    const lastIteration = simGifsForSelected[simGifsForSelected.length - 1].robot_path || [];

    firstIteration.forEach(startPoint => {
      const endPoint = lastIteration.find(p => p.robot === startPoint.robot);
      if (endPoint) {
        const dx = endPoint.robot_position.x - startPoint.robot_position.x;
        const dy = endPoint.robot_position.y - startPoint.robot_position.y;
        const dz = endPoint.robot_position.z - startPoint.robot_position.z;
        const distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
        robotDistances[startPoint.robot] = distance.toFixed(2);
      }
    });
  }


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
              {gifsInSet
                .filter(gifObj => gifObj.robotSim_id === selectedSetSimId && gifObj.iteration === currentIteration)
                .map((gifObj, index) => (
                  <div key={index} className="gif-description">
                    <div className="Simulation-set-select-container">
                      {activeRobotButton == "stats" && (
                        <>
                          <h3>Average Time per Iteration: {avgTime} s</h3>
                          <h3>Average Iterations per Simulation: {avgIterationsPerSim}</h3>
                        </>
                      )}
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
                    {activeRobotButton != 'path' &&(
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
            {activeRobotButton == "path" && (
              <>
              <Trajectory
                robotPaths={robotPaths}
                simulationBounds={simulationBounds}
                width={600}
                height={600}
                currentIteration={currentIteration}
              />
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
            </>
            )}
            {activeRobotButton === 'stats' && (
              <div className="stats-graph-container">
                <h3>Robot Distances (first → last iteration):</h3>
                <ul>
                  {Object.entries(robotDistances).map(([robot, distance]) => (
                    <li key={robot}>Robot {robot}: {distance} m</li>
                  ))}
                </ul>
                <Line
                  data={{
                    labels: gifsInSet
                      .filter(gifObj => gifObj.robotSim_id === selectedSetSimId)
                      .sort((a, b) => a.iteration - b.iteration)
                      .map(gifObj => gifObj.iteration),
                    datasets: [
                      {
                        label: 'Time (s)',
                        data: gifsInSet
                          .filter(gifObj => gifObj.robotSim_id === selectedSetSimId)
                          .map(gifObj => gifObj.time),
                        borderColor: 'rgba(190, 11, 11, 1)',
                        backgroundColor: 'rgba(75,192,192,0.2)',
                        fill: true,
                        tension: 0.1,
                      },
                    ],
                  }}
                  options={{
                    responsive: true,
                    plugins: {
                      legend: { display: true },
                      title: { display: true, text: 'Time per iteration' },
                    },
                   scales: {
                      x: {
                        title: { display: true, text: 'Iteration' },
                      },
                      y: {
                        title: { display: true, text: 'Time (s)' },
                        ticks: {
                          autoSkip: false,   
                        },
                      },
                    },
                  }}
                />
              </div>
            )}
          </div>
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
