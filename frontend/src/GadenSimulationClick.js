import React from 'react';
import GifWithGrid from './GifWithGrid';
import HoverComponent from './HoverComponent';

const GadenSimulationClick = ({
  clickedGif,
  fadeOut,
  handleGoBackSimulationDetails,
  pagePath,
  setPagePath,
  relatedGifs,
  currentIteration,
  minIteration,
  maxIteration,
  showGrid,
  toggleGrid,
  robots,
  selectedRobotNumber,
  selectedRobotIdx,
  setSelectedRobotIdx,
  simulationBounds,
  deviationSet,
  robotSimulationMode,
  handleRobotInputChange,
  handleIterationBackGaden,
  isPausedGaden,
  handlePauseResume,
  handleIterationForwardGaden,
  gadenSimulationSpeed,
  handleChangeSimulationSpeedGaden,
  setCurrentIteration,
  robotSimulationIsLoading,
  robotSimulationLoadingText,
  handleRobotSimulationSubmit,
  startingIteration,
  handleStartingIterationInputChange,
  numRobotSimulations,
  handleNumRobotSimulationsChange,
  nameSimulationSet,
  handleNameSimulationSetChange,
  handleDeviationSetChange,
  psoSimulationIterations,
  handlePsoIterationsInputChange,
  useRos,
  toggleRos,
  setSelectedRobotNumber,
  setRobotSimulationMode
}) => {
  return (
    <>
        <div className="gaden-simulation-click">
          <button
            className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
            onClick={() => {
                handleGoBackSimulationDetails();
                setPagePath(pagePath.slice(0, -1));
              }}
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
    </>
  );
};

export default GadenSimulationClick;