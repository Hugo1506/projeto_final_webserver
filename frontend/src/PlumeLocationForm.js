import React from 'react';
import EnviromentGrid from './enviromentGrid';  
import HoverComponent from './HoverComponent';  

const PlumeLocationForm = ({
  filteredGifs,
  selectedHeight,
  setSelectedHeight,
  availableHeights,
  showGrid,
  toggleGrid,
  simulationBounds,
  setPlumeXLocation,
  setPlumeYLocation,
  setPlumeZLocation,
  plumeXLocation,
  plumeYLocation,
  plumeZLocation,
  temperatureInC,
  setTemperatureInC,
  handlePlumeSubmit,
  showFilamentOptions,
  setShowFilamentOptions,
  ppmCenter,
  setPpmCenter,
  numFilamentsSec,
  setNumFilamentsSec,
  filamentInitialStd,
  setFilamentInitialStd,
  filamentGrowth,
  setFilamentGrowth,
  filamentNoise,
  setFilamentNoise,
  fadeOut,
  plumeSimulationIsLoading,
  plumeSimulationLoadingText,
  handleGoBackGadenChoise,
  setPagePath,
  pagePath
}) => {
  return (
      <div className="content-container">
        {filteredGifs.length > 0 && (
          filteredGifs
            .slice()
            .sort((a, b) => a.type.localeCompare(b.type))
            .filter((gifObj) => gifObj.height == selectedHeight)
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
                        className="toggle-button"
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
                ) : (
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
            <h4 htmlFor="plumeXLocation">Plume location</h4>
            <label htmlFor="plumeXLocation">
              X coord Range: {simulationBounds.xMin} and {simulationBounds.xMax}
              <HoverComponent text="X value X.X for the plume location" />
            </label>
            <input
              type="float"
              id="plumeXLocation"
              name="plumeXLocation"
              value={plumeXLocation}
              onChange={(e) => setPlumeXLocation(e.target.value)}
            />
            <label htmlFor="plumeYLocation">
              Y coord Range: {simulationBounds.yMin} and {simulationBounds.yMax}
              <HoverComponent text="Y value X.X for the plume location" />
            </label>
            <input
              type="float"
              id="plumeYLocation"
              name="plumeYLocation"
              value={plumeYLocation}
              onChange={(e) => setPlumeYLocation(e.target.value)}
            />
            <label htmlFor="plumeZLocation">
              Z coord Range: {simulationBounds.zMin} and {simulationBounds.zMax}
              <HoverComponent text="Z value X.X for the plume location" />
            </label>
            <input
              type="float"
              id="plumeZLocation"
              name="plumeZLocation"
              value={plumeZLocation}
              onChange={(e) => setPlumeZLocation(e.target.value)}
            />
            <label htmlFor="temperatureInC">
              Temperature
              <HoverComponent text="Temperature in Celsius (Default: 24.85)" />
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
              {showFilamentOptions ? 'Hide' : 'Show'} Filament Options (optional)
              {showFilamentOptions ? '▲' : '▼'}
            </button>

            {showFilamentOptions && (
              <div className="filament-options">
                <label htmlFor="ppmCenter">
                  PPM
                  <HoverComponent text="PPM of the initial plume (Default: 10ppm)" />
                </label>
                <input
                  type="number"
                  step="1"
                  id="ppmCenter"
                  name="ppmCenter"
                  value={ppmCenter}
                  onChange={(e) => setPpmCenter(e.target.value)}
                />
                <label htmlFor="numFilamentsSec">
                  Number of filaments per second
                  <HoverComponent text="Number of filaments released each second (Default: 10)" />
                </label>
                <input
                  type="number"
                  step="1"
                  id="numFilamentsSec"
                  name="numFilamentsSec"
                  value={numFilamentsSec}
                  onChange={(e) => setNumFilamentsSec(e.target.value)}
                />
                <label htmlFor="filamentInitialStd">
                  Sigma of the filament
                  <HoverComponent text="Sigma of the filament at t=0 in cm (Default: 10)" />
                </label>
                <input
                  type="float"
                  step="0.1"
                  id="filamentInitialStd"
                  name="filamentInitialStd"
                  value={filamentInitialStd}
                  onChange={(e) => setFilamentInitialStd(e.target.value)}
                />
                <label htmlFor="filamentGrowth">
                  Filament Growth
                  <HoverComponent text="Growth ratio of the filament_std in cm²/s (Default: 10)" />
                </label>
                <input
                  type="float"
                  step="0.1"
                  id="filamentGrowth"
                  name="filamentGrowth"
                  value={filamentGrowth}
                  onChange={(e) => setFilamentGrowth(e.target.value)}
                />
                <label htmlFor="filamentNoise">
                  Filament Noise
                  <HoverComponent text="Range of the white noise added on each iteration in m (Default: 0.02)" />
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

          <button type="submit" className={`submit-button ${fadeOut ? 'fade-out' : ''}`}>
            Submit
          </button>
          <button
            type="button"
            onClick={() => {
              handleGoBackGadenChoise();
              setPagePath(pagePath.slice(0, -1));
            }}
            className={`go-back-button ${fadeOut ? 'fade-out' : ''}`}
          >
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
    )
};

export default PlumeLocationForm;
