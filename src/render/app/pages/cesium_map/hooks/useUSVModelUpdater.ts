import * as Cesium from 'cesium'
import { useEffect } from 'react'
import { useWaypointVisualizer } from './useWaypointVisualizer'
import { 
  ModelUpdaterProps, 
} from '../types'
import { DEFAULT_LOCATION, CESIUM_CONFIG } from '../constants'
import { handleHookError } from '../utils/errorHandler'

// USV model update hook
export const useUSVModelUpdater = ({
  viewer,
  usvModelRef,
  pathPositionRef,
  viewerReady,
  gpsData,
  imuData,
  odomData,
  waypoints,
  reachedWaypoints,
  currentTargetWaypoint,
  dispatch
}: ModelUpdaterProps) => {

  const hpRoll = new Cesium.HeadingPitchRoll()
  const fixedFrameTransform = Cesium.Transforms.localFrameToFixedFrameGenerator('north', 'west')

 // Waypoint visualization hook
  const { drawWaypoints } = useWaypointVisualizer(viewer)

  // Waypoint visual update
  useEffect(() => {
    if (viewerReady && viewer) {
      console.log('🎨 Updating waypoints:', waypoints.length, 'count, Reached:', reachedWaypoints.size, 'Target:', currentTargetWaypoint)
      try {
        drawWaypoints(waypoints, reachedWaypoints, currentTargetWaypoint)
      } catch (error) {
        handleHookError(error, 'useUSVModelUpdater - WaypointVisualizer')
      }
    }
  }, [waypoints, reachedWaypoints, currentTargetWaypoint, viewerReady, viewer, drawWaypoints])

  // Model and position update
  useEffect(() => {
    if (!viewerReady || !viewer || !gpsData || !imuData || !usvModelRef || !pathPositionRef) {
      return
    }

    // Only pure GPS data is used 
    let longitude = gpsData.longitude || DEFAULT_LOCATION.longitude
    let latitude = gpsData.latitude || DEFAULT_LOCATION.latitude
    const altitude = DEFAULT_LOCATION.altitude


    const newPosition = Cesium.Cartesian3.fromDegrees(longitude, latitude, altitude)

    // Set model rotation
    hpRoll.heading = -(imuData.yaw + CESIUM_CONFIG.HEADING_OFFSET || 0)
    hpRoll.pitch = -(imuData.pitch || 0)
    hpRoll.roll = (imuData.roll || 0)

    const modelMatrix = Cesium.Transforms.headingPitchRollToFixedFrame(
      newPosition, 
      hpRoll, 
      Cesium.Ellipsoid.WGS84, 
      fixedFrameTransform
    )
    
    // Update model position
    if (usvModelRef.current) {
      Cesium.Matrix4.clone(modelMatrix, usvModelRef.current.modelMatrix)
    }

    // Update path position
    pathPositionRef.current.addSample(Cesium.JulianDate.now(), newPosition)

 
    if (dispatch) {
      dispatch({
        type: 'incrementByAmount',
        payload: {
          heading: Cesium.Math.toDegrees(imuData.yaw || 0),
          pitch: Cesium.Math.toDegrees(imuData.pitch || 0),
          roll: Cesium.Math.toDegrees(imuData.roll || 0),
          airSpeed: odomData?.linear_velocity?.x || 0
        }
      })
    }
  }, [gpsData, imuData, odomData, viewerReady, viewer, usvModelRef, pathPositionRef, dispatch])

  return {
    hpRoll,
    fixedFrameTransform
  }
}
