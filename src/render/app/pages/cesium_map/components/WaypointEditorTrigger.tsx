import React from 'react';
import './WaypointEditorTrigger.css';

interface WaypointEditorTriggerProps {
  onOpen: () => void;
}

const WaypointEditorTrigger: React.FC<WaypointEditorTriggerProps> = ({ onOpen }) => {
  return (
    <div className="waypoint-editor-trigger-container">
      <button
        onClick={onOpen}
        className="waypoint-editor-trigger-button"
      >
        Waypoint Editor
      </button>
    </div>
  );
};

export default WaypointEditorTrigger;
