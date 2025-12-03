import { useEffect, useRef } from 'react'
import * as Cesium from 'cesium'

interface Nav2PlanPose {
  position: {
    x: number
    y: number
    z: number
  }
  orientation: {
    x: number
    y: number
    z: number
    w: number
  }
  header: {
    frame_id: string
    stamp: number
  }
}

interface Nav2PlanData {
  header: {
    frame_id: string
    stamp: number
  }
  poses: Nav2PlanPose[]
  pose_count: number
}

interface UseNav2PlanVisualizerProps {
  viewer: any
  nav2PlanData: Nav2PlanData | null
  showPlan: boolean
  odomData?: {
    position: {
      x: number
      y: number
      z: number
    }
  }
  gpsData?: {
    latitude: number
    longitude: number
    altitude: number
  }
}

export const useNav2PlanVisualizer = ({ 
  viewer, 
  nav2PlanData, 
  showPlan,
  odomData,
  gpsData
}: UseNav2PlanVisualizerProps) => {
  const planEntityRef = useRef<any>(null)

  useEffect(() => {
    if (!viewer || !nav2PlanData || !showPlan) {
      if (planEntityRef.current && viewer) {
        viewer.entities.remove(planEntityRef.current)
        planEntityRef.current = null
      }
      return
    }

    const createOrUpdatePlanEntity = () => {
      
      const positions = nav2PlanData.poses.map((pose, index) => {

        // Using GPS reference and the robot's current odometry
        // This way, the Nav2 plan will be correctly displayed relative to the robot's current position
        const baseLat = gpsData?.latitude || 41.0211
        const baseLon = gpsData?.longitude || 29.0046
        
        // Robot's current odometry position
        const currentOdomX = odomData?.position?.x || 0
        const currentOdomY = odomData?.position?.y || 0
        
        // 1 degree = 111.32 km (equator)
        const metersPerDegreeLat = 111320
        const metersPerDegreeLon = 111320 * Math.cos(baseLat * Math.PI / 180)
        
        // Convert Nav2 plan positions relative to the robot's current position
        const relativeX = pose.position.x - currentOdomX
        const relativeY = pose.position.y - currentOdomY

        const latitude = baseLat + (relativeY / metersPerDegreeLat)
        const longitude = baseLon + (relativeX / metersPerDegreeLon)
        
        const isDev = process.env.NODE_ENV === 'development'
        if (isDev && (index === 0 || index > nav2PlanData.poses.length - 5)) {
          console.log(`Plan convert [${new Date().toLocaleTimeString()}]: Base GPS(${baseLat}, ${baseLon}) + Odom(${currentOdomX}, ${currentOdomY}) + Plan(${pose.position.x}, ${pose.position.y}) -> Relative(${relativeX}, ${relativeY}) -> GPS(${latitude.toFixed(6)}, ${longitude.toFixed(6)})`)
        }
        
        return Cesium.Cartesian3.fromDegrees(
          longitude,
          latitude,
          pose.position.z || 0
        )
      }).filter(pos => pos)

      if (positions.length === 0) {
        console.warn('Gösterilecek plan pozisyonu bulunamadı')
        return
      }

    
      if (planEntityRef.current) {
       
        planEntityRef.current.polyline.positions = positions
        const isDev = process.env.NODE_ENV === 'development'
        if (isDev) {
          console.log(`Nav2 plan güncellendi: ${positions.length} pozisyon`)
        }
      } else {
    
        planEntityRef.current = viewer.entities.add({
          name: 'Nav2 Plan',
          polyline: {
            positions: positions,
            width: 3,
            material: Cesium.Color.fromCssColorString('#00FF00'), 
            clampToGround: true,
            classificationType: Cesium.ClassificationType.TERRAIN
          }
        })
        const isDev = process.env.NODE_ENV === 'development'
        if (isDev) {
          console.log(`Nav2 plan oluşturuldu: ${positions.length} pozisyon`)
        }
      }
    }

    createOrUpdatePlanEntity()

    
    return () => {
      if (planEntityRef.current && viewer) {
        viewer.entities.remove(planEntityRef.current)
        planEntityRef.current = null
      }
    }
  }, [viewer, nav2PlanData, showPlan]) 

  return {
    planEntity: planEntityRef.current
  }
}
