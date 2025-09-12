import React, { useState } from 'react';
import ConfirmationModal from './ConfirmationModal';
import './SimulationDetailNoRobot.css'
const SimulationDetail = ({
  fadeOut,
  activeButton,
  filteredGifs,
  checkedOptions,
  selectedHeight,
  availableHeights,
  loadingGifs,
  currentIteration,
  showCheckboxes,
  setPagePath,
  setSelectedHeight,
  handleGoBackSavedSimulations,
  handleCheckboxChange,
  handleGifClick,
  handleImageLoaded,
  handleToggleButton
}) => {
  return (
      <>
        <div className="control-simulation-details">
          <button
            className={`go-back-simulation-details ${fadeOut ? 'fade-out' : ''}`}
            onClick={() => {
              handleGoBackSavedSimulations();
              setPagePath((prev) => prev.slice(0, -1));
            }}
          >
            Go Back
          </button>
          <div className="toggle-buttons">
            <button
              className={`toggle-button-no-robot ${activeButton === 'gaden' ? 'active' : 'inactive'}`}
              onClick={() => handleToggleButton('gaden')}
            >
              Gaden Simulations
            </button>
          </div>
          {showCheckboxes && (
            <div className="checkbox-filters">
              <label>
                <input
                  type="checkbox"
                  name="all"
                  checked={checkedOptions.all}
                  onChange={handleCheckboxChange}
                />
                Show all simulations
              </label>
              <label>
                <input
                  type="checkbox"
                  name="heatmap"
                  checked={checkedOptions.heatmap}
                  onChange={handleCheckboxChange}
                />
                Show heatmap
              </label>
              <label>
                <input
                  type="checkbox"
                  name="wind"
                  checked={checkedOptions.wind}
                  onChange={handleCheckboxChange}
                />
                Show wind vectors
              </label>
              <label>
                <input
                  type="checkbox"
                  name="contour"
                  checked={checkedOptions.contour}
                  onChange={handleCheckboxChange}
                />
                Show contour
              </label>
            </div>
          )}
        </div>

        <div className="gif-container">
          {!loadingGifs && filteredGifs.length === 0 && activeButton !== 'robot' && (
            <p>No GIFs available for the selected options.</p>
          )}

          {filteredGifs.length > 0 &&
            filteredGifs
              .slice()
              .sort((a, b) => a.type.localeCompare(b.type))
              .filter((gifObj) => {
                if (activeButton === 'gaden') {
                  return (
                    ['heatmap', 'wind', 'contour'].includes(gifObj.type) && gifObj.iteration === currentIteration
                  );
                }
              })
              .filter((gifObj) => {
                if (gifObj.type !== 'robot') {
                  return gifObj.height == selectedHeight;
                }
                return true;
              })
              .map((gifObj, index) => (
                <div key={index} className="gif-description">
                  <h3>Height: {gifObj.height ?? 'Unknown'}</h3>
                  <img
                    src={gifObj.url}
                    alt={`Simulation GIF ${index + 1}`}
                    className="gif-image"
                    onClick={() => handleGifClick(gifObj)}
                    onLoad={() => handleImageLoaded(gifObj.url)}
                    style={{ cursor: 'pointer' }}
                  />
                </div>
              ))}
        </div>
      </>
    )
};

export default SimulationDetail;
