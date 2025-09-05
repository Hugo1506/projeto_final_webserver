import React from 'react';
import GifWithGrid from './GifWithGrid';

const GadenSimulationClickNoRobot = ({
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
            </div>
        </div>
    </>
  );
};

export default GadenSimulationClickNoRobot;