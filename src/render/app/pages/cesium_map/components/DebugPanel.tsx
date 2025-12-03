import React from 'react'
import './DebugPanel.css'

interface DebugPanelProps {
  isUpdating: boolean
  viewerReady: boolean
  gpsData: any
  odomData: any
  linearX: any
  angularZ: any
  reachedWaypoints: number
  totalWaypoints: number
  currentTargetWaypoint: number
  distanceToTarget: number
}

export const DebugPanel: React.FC<DebugPanelProps> = ({
  isUpdating,
  viewerReady,
  gpsData,
  odomData,
  linearX,
  angularZ,
  reachedWaypoints,
  totalWaypoints,
  currentTargetWaypoint,
  distanceToTarget
}) => {
  return (
    <div className="debug-panel">
      <div className="debug-panel__status">
        <strong>📡 GPS Status:</strong> {isUpdating ? '🟢 Active' : '🔴 Stopped'}
      </div>
      <div className="debug-panel__status">
        <strong>🎯 Viewer Ready:</strong> {viewerReady ? '✅' : '❌'}
      </div>
      <div className="debug-panel__info">
        <strong>- Latitude:</strong> {gpsData?.latitude?.toFixed(6)}
      </div>
      <div className="debug-panel__info">
        <strong>- Longitude:</strong> {gpsData?.longitude?.toFixed(6)}
      </div>
      <hr style={{ margin: '8px 0', borderColor: '#444' }}/>
      <div className="debug-panel__info">
        <strong>🏎️ Speed (X):</strong> {(odomData?.linear_velocity?.x || 0).toFixed(2)} m/s
      </div>
      <div className="debug-panel__info">
        <strong>➡️ Linear X Command:</strong> {(linearX?.linear_x || 0).toFixed(2)}
      </div>
      <div className="debug-panel__info">
        <strong>🔄 Angular Z Command:</strong> {(angularZ?.angular_z || 0).toFixed(2)}
      </div>
      <hr style={{ margin: '8px 0', borderColor: '#444' }}/>
      <div className="debug-panel__waypoint-info">
        <strong>📍 Reached Waypoints:</strong> {reachedWaypoints} / {totalWaypoints}
      </div>
      <div className="debug-panel__waypoint-info">
        <strong>🎯 Current Target:</strong> {totalWaypoints > 0 ? currentTargetWaypoint : 0} / {totalWaypoints}
      </div>
      <div className="debug-panel__waypoint-info">
        <strong>📏 Distance to Target:</strong> {distanceToTarget.toFixed(2)} m
      </div>
      <div className="debug-panel__info">
        <strong>🔄 Last Update:</strong> {new Date().toLocaleTimeString()}
      </div>
    </div>
  )
}
