import React from 'react';

const AmbientSubmit = ({
  simulationName,
  setSimulationName,
  handleFileSubmit,
  handleFileChange,
  handleGoBackGadenChoise,
  setPagePath,
  pagePath,
  fadeOut,
  enviromentIsLoading,
  plumeSimulationLoadingText,
  setFiles,
  files
}) => {
  return (
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
              multiple 
              webkitdirectory="true" 
              directory="" 
              onChange={(e) => setFiles({ ...files, windFiles: e.target.files })} 
            />
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
        {enviromentIsLoading && (
          <div className="popup-overlay">
            <div className="popup">
              <p>{plumeSimulationLoadingText}</p>
            </div>
          </div>
        )}
      </>
    )
};

export default AmbientSubmit;
