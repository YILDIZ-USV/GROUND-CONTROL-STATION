import React from 'react'
import { MissionControlsProps } from '../types'
import './MissionControls.css'

export const MissionControls: React.FC<MissionControlsProps> = ({
  onRunMission,
  isMissionRunning
}) => {
  return (
    <div className="mission-controls">
      <button 
        onClick={onRunMission}
        disabled={isMissionRunning}
        className={`mission-controls__button ${isMissionRunning ? 'mission-controls__button--running' : ''}`}
      >
        {isMissionRunning ? 'Mission Running...' : 'Start Mission'}
      </button>
    </div>
  )
}
