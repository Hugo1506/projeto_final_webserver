import { useState } from "react";
import "./Fields.css";

export default function AddLabelInput({ labels, values, onLabelChange, onValueChange, onAddLabel, onRemoveLabel }) {
  return (
    <div className="add-field-container">
      <button
        type="button"
        onClick={onAddLabel}
        className="add-field-button"
      >
        +
      </button>

      <div className="add-field-list">
        {labels.map((field, index) => (
          <div key={index} className="add-field-row">
            <div className="add-field-header">
              <input
                type="text"
                value={field.label}
                onChange={(e) => onLabelChange(index, e.target.value)}
                className="add-field-label-input"
              />
              <button
                type="button"
                className="delete-field-button"
                onClick={() => onRemoveLabel(index)}
              >
                🗑️
              </button>
            </div>
            <input
              type="text"
              placeholder="Enter text..."
              value={values[index]}
              onChange={(e) => onValueChange(index, e.target.value)}
              className="add-field-input"
            />
          </div>
        ))}
      </div>
    </div>
  );
}