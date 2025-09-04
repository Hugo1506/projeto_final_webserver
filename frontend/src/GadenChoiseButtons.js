import React from 'react';

const GadenChoiceButtons = ({ ambientSimulator, fadeOut, handleNewSimulationClick, handleSavedSimulationsClick, handleGoBack, setPagePath, pagePath }) => {
  return (
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
          onClick={() => {
            handleGoBack();
            setPagePath(pagePath.slice(0, -1));
          }}
        >
          Go Back
        </button>
        <br />
      </div>
    )
};

export default GadenChoiceButtons;
