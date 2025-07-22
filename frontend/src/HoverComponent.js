import React, { useState } from 'react';
import './HoverComponent.css'; 

const HoverComponent = ({ text }) => {
  const [isHovering, setIsHovering] = useState(false);

  return (
    <div
      className="hover-container"
      onMouseEnter={() => setIsHovering(true)}
      onMouseLeave={() => setIsHovering(false)}
    >
      <span className="hover-icon">❓</span>
      {isHovering && (
        <div className="hover-tooltip">
          {text}
        </div>
      )}
    </div>
  );
};

export default HoverComponent;