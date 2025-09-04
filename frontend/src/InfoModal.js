import React, { useState, useEffect } from 'react';

const InfoModal = ({ message }) => {
  const [dotCount, setDotCount] = useState(0);

  useEffect(() => {
    const interval = setInterval(() => {
      setDotCount((prev) => (prev + 1) % 4); // Cycle through 0, 1, 2, 3 to display dots
    }, 500);

    return () => clearInterval(interval); // Cleanup interval on component unmount
  }, []);

  return (
    <div className="modal-overlay">
      <div className="modal">
        <p>{message}{'.'.repeat(dotCount)}</p>
      </div>
    </div>
  );
};

export default InfoModal;
