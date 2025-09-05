import React from 'react';

const PlumeOrExperiences = ({
  ambientSimulator,
  fadeOut,
  handlePlumeClick,
  handleExperiencesClick,
  setPlumeOrExperienciesVisible,
  pagePath,
  setPagePath,
}) => {
  return (
    <div className="plumeOrExperiences-choice-buttons">
      <h1 className={'info-header'}>{ambientSimulator}</h1>
      <button
        className={`Plume-button ${fadeOut ? 'fade-out' : ''}`}
        onClick={handlePlumeClick}
      >
        Plume
      </button>
      <button
        className={`Experience-button ${fadeOut ? 'fade-out' : ''}`}
        onClick={handleExperiencesClick}
      >
        Experiences
      </button>
      <button
        className={`go-back-button ${fadeOut ? 'fade-out' : ''}`}
        onClick={() => {
          setPlumeOrExperienciesVisible(false);
          setPagePath(pagePath.slice(0, -1));
        }}
      >
        Go Back
      </button>
      <br />
    </div>
  );
};

export default PlumeOrExperiences;