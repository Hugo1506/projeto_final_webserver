import React from 'react';
import ConfirmationModal from './ConfirmationModal'; 
import InfoModal from './InfoModal'; 

const SavedSimulationsPage = ({
  fadeOut,
  filteredSimulations,
  handleSearch,
  handleGoBackGadenChoise,
  setPagePath,
  pagePath,
  handleSimulationClick,
  openDeleteModal,
  showModal,
  confirmDelete,
  closeModal,
  showInfoModal,
}) => {
  return (
      <div className="saved-simulations-list">
        <div className="saved-simulations-header">
          <h3>Saved Plume Simulations</h3>
          <input
            type="text"
            className="search-bar"
            placeholder="Search by name or simulation"
            onChange={handleSearch}
          />
          <button
            className={`go-back-saved-simulations ${fadeOut ? 'fade-out' : ''}`}
            onClick={() => {
              handleGoBackGadenChoise();
              setPagePath(pagePath.slice(0, -1));
            }}
          >
            Go Back
          </button>
        </div>
        <div className="saved-simulations-list">
          {filteredSimulations.length > 0 ? (
            <ul>
              {filteredSimulations.map((simulation, index) => (
                <li
                  key={index}
                  className={`simulation-item ${fadeOut ? 'fade-out' : ''}`}
                >
                  <div className="simulation-content">
                    <div
                      className="simulation-details"
                      onClick={() => handleSimulationClick(simulation.simulation)}
                    >
                      <strong>Simulation Name:</strong> {simulation.simulationName || 'Unnamed Simulation'} <br />
                      <strong>Simulation:</strong> {simulation.simulation || 'No simulation description'}
                    </div>
                    <div
                      className="trash-icon"
                      onClick={(e) => {
                        e.stopPropagation();
                        openDeleteModal(simulation);
                      }}
                    >
                      🗑️
                    </div>
                  </div>
                </li>
              ))}
            </ul>
          ) : (
            <p>No simulations found</p>
          )}
          {showModal && (
            <ConfirmationModal
              message="Are you sure you want to delete this simulation?"
              onConfirm={confirmDelete}
              onCancel={closeModal}
            />
          )}
          {showInfoModal && (
            <InfoModal message="Loading Environment Simulation" />
          )}
        </div>
      </div>
    )
};

export default SavedSimulationsPage;
