import React from 'react'
import './ExtraControls.css'

interface ExtraControlsProps {
  onRunMission2: () => void
  isMission2Running: boolean
  onSaveColorCode: () => void
  colorCode: string
  setColorCode: (value: string) => void
}

export const ExtraControls: React.FC<ExtraControlsProps> = ({
  onRunMission2,
  isMission2Running,
  onSaveColorCode,
  colorCode,
  setColorCode
}) => {
  return (
    <div className="extra-controls">
      <div className="extra-controls__form-group">
        <label className="extra-controls__label">
          Color Code:
        </label>
        <input
          type="text"
          value={colorCode}
          onChange={(e) => setColorCode(e.target.value)}
          placeholder="#RRGGBB or color name"
          className="extra-controls__color-input"
        />
        <button
          onClick={onSaveColorCode}
          className="extra-controls__save-button"
        >
          Save Color Code
        </button>
      </div>
      
      <div>
        <button
          onClick={onRunMission2}
          disabled={isMission2Running}
          className="extra-controls__mission-button"
        >
          {isMission2Running ? 'Mission 2 Running...' : 'Send Color Code'}
        </button>
      </div>
    </div>
  )
}
