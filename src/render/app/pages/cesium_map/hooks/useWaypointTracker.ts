import * as Cesium from 'cesium'
import { useEffect, useState } from 'react'
import { 
  Waypoint, 
  GPSData, 
  WaypointTrackerReturn 
} from '../types'
import { DEFAULT_LOCATION, ARRIVAL_THRESHOLD_METERS } from '../constants'

export const useWaypointTracker = (
  waypoints: Waypoint[],
  gpsData: GPSData,
  arrivalThreshold: number = ARRIVAL_THRESHOLD_METERS
): WaypointTrackerReturn => {
  const [reachedWaypoints, setReachedWaypoints] = useState<Set<string | number>>(new Set())
  const [currentTargetWaypoint, setCurrentTargetWaypoint] = useState(0)
  const [distanceToTarget, setDistanceToTarget] = useState(0)

  useEffect(() => {
    if (waypoints.length === 0) {
      setDistanceToTarget(0)
      return
    }

    if (currentTargetWaypoint >= waypoints.length) {
      return
    }

    const targetWaypoint = waypoints[currentTargetWaypoint]
    const targetCartesian = Cesium.Cartesian3.fromDegrees(
      targetWaypoint.longitude, 
      targetWaypoint.latitude, 
      0
    )
    
    const currentCartesian = Cesium.Cartesian3.fromDegrees(
      gpsData.longitude || DEFAULT_LOCATION.longitude,
      gpsData.latitude || DEFAULT_LOCATION.latitude,
      DEFAULT_LOCATION.altitude
    )
    
    const distance = Cesium.Cartesian3.distance(currentCartesian, targetCartesian)
    setDistanceToTarget(distance)
    
    console.log(` Target WP ${currentTargetWaypoint + 1}: ${distance.toFixed(2)}m`)
    
    if (distance < arrivalThreshold) {
      const newReached = new Set(reachedWaypoints)
      newReached.add(targetWaypoint.id)
      setReachedWaypoints(newReached)
      
      if (currentTargetWaypoint < waypoints.length - 1) {
        setCurrentTargetWaypoint(currentTargetWaypoint + 1)
        console.log(`✅ WP ${currentTargetWaypoint + 1} reached! New target: ${currentTargetWaypoint + 2}`)
      } else {
        console.log(`🎉 FINAL WAYPOINT REACHED!`)
      }
    }
  }, [waypoints, gpsData, currentTargetWaypoint, reachedWaypoints, arrivalThreshold])

  const resetTracker = () => {
    setReachedWaypoints(new Set())
    setCurrentTargetWaypoint(0)
    setDistanceToTarget(0)
  }

  return {
    reachedWaypoints,
    currentTargetWaypoint,
    distanceToTarget,
    resetTracker
  }
}
