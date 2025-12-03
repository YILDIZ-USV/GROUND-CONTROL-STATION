import React from 'react'
import './Nav2PlanControls.css'

interface Nav2PlanControlsProps {
  showPlan: boolean
  onTogglePlan: () => void
  isPlanLoading: boolean
  planError: string | null
  planData: any
}

const Nav2PlanControls: React.FC<Nav2PlanControlsProps> = ({
  showPlan,
  onTogglePlan,
  isPlanLoading,
  planError,
  planData
}) => {
  const getPlanStatus = () => {
    if (isPlanLoading) return { text: 'Loading...', className: 'loading' }
    if (planError) return { text: 'Error', className: 'invalid' }
    if (planData && planData.pose_count > 0) return { text: 'Active', className: 'valid' }
    return { text: 'No Data', className: 'invalid' }
  }

  const status = getPlanStatus()

  return (
    <div className="nav2-plan-controls">
      <h3>Nav2 Plan</h3>
      
      <div className="plan-toggle-container">
        <div 
          className={`plan-toggle ${showPlan ? 'active' : ''}`}
          onClick={onTogglePlan}
        >
          <div className="plan-toggle-slider"></div>
        </div>
        <span className="plan-toggle-label">
          {showPlan ? 'Hide' : 'Show'} Plan
        </span>
      </div>

      <div className={`plan-status ${status.className}`}>
        Status: {status.text}
      </div>

      {planData && planData.pose_count > 0 && (
        <div className="plan-info">
          Positions: <span>{planData.pose_count}</span>
        </div>
      )}

      {planError && (
        <div className="plan-info" style={{ color: '#ff4444', fontSize: '10px' }}>
          {planError}
        </div>
      )}
    </div>
  )
}

export default Nav2PlanControls
