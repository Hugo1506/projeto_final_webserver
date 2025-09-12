import React from 'react';

const NewOrSavedExperiences = ({
  ambientSimulator,  
  fadeOut,
  handlePlumeClick,
  handleSavedExperiencesClick,
  setNewOrSavedExperiencesVisible,
  pagePath,
  setPagePath,
  setPlumeOrExperienciesVisible
}) => {
  return (
    <div className="plumeOrExperiences-choice-buttons">
    <h1 className={'info-header'}>{ambientSimulator}</h1>
      <button
        className={`New-experience-button ${fadeOut ? 'fade-out' : ''}`}
        onClick={handlePlumeClick}
      >
        New Experience
      </button>
      <button
        className={`Saved-xperience-button ${fadeOut ? 'fade-out' : ''}`}
        onClick={handleSavedExperiencesClick}
      >
        Saved Experiences
      </button>
      <button
        className={`go-back-button ${fadeOut ? 'fade-out' : ''}`}
        onClick={() => {
          setNewOrSavedExperiencesVisible(false);
          setPlumeOrExperienciesVisible(true);
          setPagePath(pagePath.slice(0, -1));
        }}
      >
        Go Back
      </button>
      <br />
    </div>
  );
};

export default NewOrSavedExperiences;