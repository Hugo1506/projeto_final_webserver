import React, { useState } from 'react';
import GifWithGrid from './GifWithGrid'; 
import InfoModal from './InfoModal';  

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
  simulationBounds
}) => {

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
                    <h3>Median time per iteration: {medianTime}</h3>
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
                    onChange={() => setShowTotalStatsRobotSim(prev => !prev)}
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
