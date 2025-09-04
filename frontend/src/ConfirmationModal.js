import React from 'react';

const ConfirmationModal = ({ message, onConfirm, onCancel }) => {
  return (
    <div className="modal-overlay">
      <div className="modal">
        <p>{message}</p>
        <div className="modal-buttons">
          <button onClick={onConfirm}>YES</button>
          <button onClick={onCancel}>NO</button>
        </div>
      </div>
    </div>
  );
};

export default ConfirmationModal;
