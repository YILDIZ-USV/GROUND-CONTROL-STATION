import { useEffect, useState } from 'react'
import { useDispatch, useSelector } from 'react-redux'
import PlaneService from './service/plane_service'
import './CesiumMap.css'

import { useCesiumViewer } from './hooks/useCesiumViewer'
import { useUSVData } from './hooks/useUSVData'
import { useWaypointTracker } from './hooks/useWaypointTracker'
import { useUSVModelUpdater } from './hooks/useUSVModelUpdater'
import { useMissionHandlers } from './hooks/useMissionHandlers'
import { useNav2PlanData } from './hooks/useNav2PlanData'
import { useNav2PlanVisualizer } from './hooks/useNav2PlanVisualizer'
import { useWaypointVisualizer } from './hooks/useWaypointVisualizer'

import { 
  MissionControls, 
  ExtraControls, 
  DebugPanel, 
  WaypointStatusPanel, 
  VehicleStatusIndicator,
  FlightIndicator,
  Nav2PlanControls
} from './components'

// Constants and types
import { ARRIVAL_THRESHOLD_METERS } from './constants'
import { CesiumMapProps } from './types'
import type { RootState } from '../../store'
import { incrementByAmount } from '../../../hooks/HeadingIndicator_features'

// Main CesiumMap component for Yildizusv telemetry visualization 

const CesiumMap = (_props: CesiumMapProps) => {
  // Redux state - use applied waypoints for map visualization
  const appliedWaypoints = useSelector((state: RootState) => state.waypoints.appliedWaypoints)
  const dispatch = useDispatch()
  
  // Services
  const service = new PlaneService()

  // Nav2 plan state for toggle functionality
  const [showNav2Plan, setShowNav2Plan] = useState(false)

  // Hooks - each handles its own responsibility
  const { viewer, usvModelRef, pathPositionRef,  viewerReady } = useCesiumViewer('cesiumContainer')
  
  const { 
    gpsData, 
    imuData, 
    odomData, 
    linearX, 
    angularZ, 
    armedStatus, 
    modeStatus 
  } = useUSVData(service, viewerReady)
  
  const { reachedWaypoints, currentTargetWaypoint, distanceToTarget } = useWaypointTracker(
    appliedWaypoints,
    gpsData,
    ARRIVAL_THRESHOLD_METERS
  )

  // Waypoint visualizer hook for proper animation and display
  const waypointVisualizer = useWaypointVisualizer(viewer)
  

  useUSVModelUpdater({
    viewer,
    usvModelRef,
    pathPositionRef,
    viewerReady,
    gpsData,
    imuData,
    odomData,
    waypoints: appliedWaypoints,
    reachedWaypoints,
    currentTargetWaypoint,
    dispatch
  })

  
  const {
    handleRunMission,
    handleRunMission2,
    handleSaveColorCode,
    isMissionRunning,
    isMission2Running,
    colorCode,
    setColorCode
  } = useMissionHandlers(service)

  
  const { nav2PlanData, isPlanLoading, planError } = useNav2PlanData(service, viewerReady)

  useNav2PlanVisualizer({
    viewer,
    nav2PlanData,
    showPlan: showNav2Plan, // Dynamic showPlan state
    odomData,
    gpsData
  })

  // Nav2 plan toggle handler
  const handleNav2PlanToggle = () => {
    setShowNav2Plan(prev => !prev)
  }

  // Update waypoint visualization when waypoints or status changes
  useEffect(() => {
    if (waypointVisualizer && viewerReady) {
      waypointVisualizer.drawWaypoints(appliedWaypoints, reachedWaypoints, currentTargetWaypoint)
    }
  }, [appliedWaypoints, reachedWaypoints, currentTargetWaypoint, waypointVisualizer, viewerReady])

  // Orchestrator responsibility: coordinate data flow to Redux
  useEffect(() => {
    if (imuData && gpsData && linearX && angularZ) {
      const indicatorData = {
        heading: imuData.yaw || 0,
        pitch: imuData.pitch || 0,
        roll: imuData.roll || 0,
        airSpeed: linearX.linear_x || 0,
        vario: angularZ.angular_z || 0,
        turn: angularZ.angular_z || 0,
        altitude: gpsData.altitude || 0,
        pressure: 1013.25
      };
      dispatch(incrementByAmount(indicatorData));
    }
  }, [imuData, gpsData, linearX, angularZ, dispatch]);

  return (
    <div className="cesium-container">
      <div id="cesiumContainer" className="cesium-viewer" />
      
      <MissionControls
        onRunMission={handleRunMission}
        isMissionRunning={isMissionRunning}
      />
      
      <ExtraControls
        onRunMission2={handleRunMission2}
        isMission2Running={isMission2Running}
        onSaveColorCode={handleSaveColorCode}
        colorCode={colorCode}
        setColorCode={setColorCode}
      />
      
      <DebugPanel
        isUpdating={true} // Always true since useUSVData handles auto-updating
        viewerReady={viewerReady}
        gpsData={gpsData}
        odomData={odomData}
        linearX={linearX}
        angularZ={angularZ}
        reachedWaypoints={reachedWaypoints.size}
        totalWaypoints={appliedWaypoints.length}
        currentTargetWaypoint={currentTargetWaypoint + 1}
        distanceToTarget={distanceToTarget}
      />

      <WaypointStatusPanel
        waypoints={appliedWaypoints}
        currentTargetWaypoint={currentTargetWaypoint}
        distanceToTarget={distanceToTarget}
        reachedWaypoints={reachedWaypoints}
      />

      <VehicleStatusIndicator
        armedStatus={armedStatus}
        modeStatus={modeStatus}
      />

      <FlightIndicator />

      <Nav2PlanControls
        showPlan={showNav2Plan}
        onTogglePlan={handleNav2PlanToggle}
        isPlanLoading={isPlanLoading}
        planError={planError}
        planData={nav2PlanData}
      />
    </div>
  )
}

export default CesiumMap
