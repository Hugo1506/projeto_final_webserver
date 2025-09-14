import React, { useState } from 'react';
import './CollapseSection.css'
const CollapsibleSection = ({ buttonLabel, children }) => {
  const [collapsed, setCollapsed] = useState(false);

  return (
    <div style={{ width: '100%' }}>
        <button className="collapse-button" onClick={() => setCollapsed(prev => !prev)}>
            {collapsed ? `Show ${buttonLabel} ▼` : `Hide ${buttonLabel} ▲`}
        </button>
        {!collapsed && <div className="collapsible-content">{children}</div>}
    </div>
  );
};

export default CollapsibleSection;