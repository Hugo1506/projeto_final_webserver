import React from 'react';
import './CollapseSection.css';

const CollapseSectionGroup = ({ sections, activeSection, setActiveSection }) => {
  return (
    <div className="collapse-section-group">
      <div className="collapse-section-buttons">
        {sections.map(({ buttonLabel }, idx) => (
          <button
            key={buttonLabel}
            className={`collapse-tab-button${activeSection === idx ? ' active' : ''}`}
            onClick={() => setActiveSection(idx)}
          >
            {buttonLabel}
          </button>
        ))}
      </div>
      <div className="collapse-section-content">
        {sections[activeSection]?.children}
      </div>
    </div>
  );
};

export default CollapseSectionGroup;