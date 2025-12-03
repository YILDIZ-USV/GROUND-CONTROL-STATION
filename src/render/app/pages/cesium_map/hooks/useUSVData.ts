import { useState, useEffect, useCallback } from 'react'
import PlaneService from '../service/plane_service'
import { 
  GPSData, 
  IMUData, 
  OdomData, 
  LinearX, 
  AngularZ, 
  ArmedStatus, 
  ModeStatus,
  USVDataReturn 
} from '../types'
import { DEFAULT_LOCATION } from '../constants'
import { handleHookError } from '../utils/errorHandler'
import { CESIUM_CONFIG } from '../constants'

export const useUSVData = (service: PlaneService, viewerReady: boolean): USVDataReturn => {
  
  const [gpsData, setGpsData] = useState<GPSData>({ 
    altitude: DEFAULT_LOCATION.altitude, 
    latitude: DEFAULT_LOCATION.latitude, 
    longitude: DEFAULT_LOCATION.longitude, 
    timestamp: 0 
  })
  
  const [imuData, setImuData] = useState<IMUData>({ 
    yaw: 0, 
    pitch: 0, 
    roll: 0 
  })
  
  const [odomData, setOdomData] = useState<OdomData>({ 
    position: { x: 0, y: 0, z: 0 },
    linear_velocity: { x: 0, y: 0, z: 0 } 
  })
  
  const [linearX, setLinearX] = useState<LinearX>({ linear_x: 0 })
  const [angularZ, setAngularZ] = useState<AngularZ>({ angular_z: 0 })
  const [armedStatus, setArmedStatus] = useState<ArmedStatus>({ armed: false })
  const [modeStatus, setModeStatus] = useState<ModeStatus>({ mode: "UNKNOWN" })

  // Memoize fetchUSVData to prevent unnecessary re-renders
  const fetchUSVData = useCallback(async () => {
    if (!viewerReady) return

    try {
      const [
        gpsResponse, 
        imuResponse, 
        odomResponse,
        linearXResponse, 
        angularZResponse,
        armedResponse,
        modeResponse
      ] = await Promise.all([
        service.getGPSVerileri(), 
        service.getIMUVerileri(),
        service.getOdometryVerileri(),
        service.getLinearX(),
        service.getAngularZ(),
        service.getArmedStatus(),
        service.getModeStatus()
      ])


      if (!gpsResponse || !imuResponse) {
        return
      }

      setGpsData(gpsResponse)
      setImuData(imuResponse)
      if (odomResponse) setOdomData(odomResponse)
      if (linearXResponse) setLinearX(linearXResponse)
      if (angularZResponse) setAngularZ(angularZResponse)
      if (armedResponse) setArmedStatus(armedResponse)
      if (modeResponse) setModeStatus(modeResponse)
      
    } catch (error: any) { 
    
      const msg = error?.message || '';
      
      if (msg.includes('Failed to fetch') || msg.includes('Network Error')) {
        return; 
      }
  
      handleHookError(error, 'useUSVData - fetchUSVData')
    
    }
  }, [service, viewerReady])

  // Auto-update data with setInterval
  useEffect(() => {
    if (!viewerReady) return

    const updateInterval = setInterval(() => {
      fetchUSVData()
    }, CESIUM_CONFIG.UPDATE_INTERVAL_MS)

    return () => {
      clearInterval(updateInterval)
    }
  }, [viewerReady, fetchUSVData])

  return {
    gpsData,
    imuData,
    odomData,
    linearX,
    angularZ,
    armedStatus,
    modeStatus,
    fetchUSVData
  }
}