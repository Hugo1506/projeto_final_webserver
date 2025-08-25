import React from 'react';
import './PagePath.css';

const PagePath = ({ path,onPathClick  }) => {
    if (!Array.isArray(path)) {
        return null;
    }

    return (
        <div className="page-path">
            {path.map((element, index) => (
                <span key={index} onClick={() => onPathClick(index)} className="path-element">
                    {element}
                    {index < path.length - 1 && <span className="separator"> &gt; </span>}
                </span>
            ))}
        </div>
    );
};

export default PagePath;