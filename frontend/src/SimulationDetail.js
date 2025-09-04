import React, { useState } from 'react';
import ConfirmationModal from './ConfirmationModal';

const SimulationDetail = ({
  fadeOut,
  activeButton,
  robotSetData,
  filteredRobotSets,
  filteredGifs,
  checkedOptions,
  selectedHeight,
  availableHeights,
  loadingGifs,
  currentIteration,
  robotSetSearch,
  setRobotSetSearch,
  showModal,
  showCheckboxes,
  setPagePath,
  setSelectedHeight,
  handleGoBackSavedSimulations,
  handleSetClick,
  openSetDeleteModal,
  confirmSetDelete,
  closeSetModal,
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
              className={`toggle-button ${activeButton === 'gaden' ? 'active' : 'inactive'}`}
              onClick={() => handleToggleButton('gaden')}
            >
              Gaden Simulations
            </button>
            <button
              className={`toggle-button ${activeButton === 'robot' ? 'active' : 'inactive'}`}
              onClick={() => handleToggleButton('robot')}
            >
              Robot Simulations
            </button>
          </div>
          {activeButton === 'robot' && robotSetData && robotSetData.length > 0 && (
            <div className={`saved-simulations-list ${fadeOut ? 'fade-out' : ''}`}>
              <div className="robot-sim-header">
                <input
                  type="text"
                  className="search-bar"
                  placeholder="Search by name of the set"
                  value={robotSetSearch}
                  onChange={(e) => setRobotSetSearch(e.target.value)}
                />
                <h3 className="simulation-title">Robot Simulation Sets</h3>
              </div>
              <ul>
                {filteredRobotSets.map((set, idx) => (
                  <li key={idx} className={`simulation-item ${fadeOut ? 'fade-out' : ''}`}>
                    <div className="simulation-content">
                      <div
                        className="simulation-details"
                        onClick={() => handleSetClick(set)}
                      >
                        <strong>Name:</strong>{' '}
                        {set.simulation_set ? set.simulation_set.split('/')[0] : 'Unnamed Set'} <br />
                        <strong>Number of Simulations:</strong> {set.simulation_set ? set.simulation_set.split('/').slice(1).join('/') : '0'}
                      </div>
                      <div
                        className="trash-icon"
                        onClick={(e) => {
                          e.stopPropagation();
                          openSetDeleteModal(set);
                        }}
                      >
                        🗑️
                      </div>
                    </div>
                  </li>
                ))}
              </ul>
            </div>
          )}
          {showModal && (
            <ConfirmationModal
              message="Are you sure you want to delete this set?"
              onConfirm={confirmSetDelete}
              onCancel={closeSetModal}
            />
          )}
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
