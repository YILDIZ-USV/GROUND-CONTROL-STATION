import React from 'react'
import './WaypointStatusPanel.css'

interface WaypointStatusPanelProps {
  waypoints: any[]
  currentTargetWaypoint: number
  distanceToTarget: number
  reachedWaypoints: Set<any>
}

export const WaypointStatusPanel: React.FC<WaypointStatusPanelProps> = ({
  waypoints,
  currentTargetWaypoint,
  distanceToTarget,
  reachedWaypoints
}) => {
  const progressPercentage = waypoints.length > 0 ? (reachedWaypoints.size / waypoints.length) * 100 : 0

  return (
    <div className="waypoint-status">
      <h3 className="waypoint-status__header">🎯 Waypoint Tracking</h3>
      
      {waypoints.length > 0 ? (
        <>
          <div className="waypoint-status__target">
            <strong>Target:</strong> WP {currentTargetWaypoint + 1}
          </div>
          <div className="waypoint-status__distance">
            <strong>Distance:</strong> <span className={distanceToTarget < 50 ? 'waypoint-status__distance--close' : 'waypoint-status__distance--far'}>
              {distanceToTarget.toFixed(1)} m
            </span>
          </div>
          <div className="waypoint-status__progress">
            <strong>Progress:</strong> {reachedWaypoints.size} / {waypoints.length}
          </div>
          <div className="waypoint-status__progress-bar">
            <div 
              className="waypoint-status__progress-fill"
              style={{ width: `${progressPercentage}%` }}
            />
          </div>
        </>
      ) : (
        <div>Waiting for waypoints...</div>
      )}
    </div>
  )
}
