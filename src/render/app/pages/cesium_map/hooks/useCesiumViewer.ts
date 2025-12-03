import * as Cesium from 'cesium'
import { useEffect, useRef, useState } from 'react'
import yildizusv from '../../../../assets/models/yildizusv.glb'
import { DEFAULT_LOCATION, CESIUM_CONFIG } from '../constants'

export const useCesiumViewer = (containerId: string = 'cesiumContainer') => {
  const mapviewer = useRef<Cesium.Viewer | null>(null)
  const usvModelRef = useRef<Cesium.Model | null>(null)
  const pathPositionRef = useRef<Cesium.SampledPositionProperty | null>(null)
  const intervalRef = useRef<NodeJS.Timeout | null>(null)
  
  const [viewerReady, setViewerReady] = useState(false)

  const initialPosition = Cesium.Cartesian3.fromDegrees(
    DEFAULT_LOCATION.longitude, 
    DEFAULT_LOCATION.latitude, 
    DEFAULT_LOCATION.altitude
  )
  const fixedFrameTransform = Cesium.Transforms.localFrameToFixedFrameGenerator('north', 'west')

  useEffect(() => {
    let viewer: Cesium.Viewer | null = null
    
    const initializeCesium = async () => {
      try {
        console.log('🚀 Initializing Cesium...')
        
        // Get token from environment variable
        const cesiumToken = import.meta.env.VITE_CESIUM_ION_TOKEN
        
        if (!cesiumToken) {
          console.error('Cesium Ion token is missing in environment variables.')
          console.error(cesiumToken)
          return
        } 
        
        Cesium.Ion.defaultAccessToken = cesiumToken

        viewer = new Cesium.Viewer(containerId, {
          shouldAnimate: true, 
          sceneMode: Cesium.SceneMode.SCENE3D, 
          animation: false, 
          timeline: false,
          fullscreenButton: false, 
          geocoder: false, 
          homeButton: false, 
          infoBox: false, 
          sceneModePicker: false,
          selectionIndicator: false, 
          navigationHelpButton: false, 
          navigationInstructionsInitiallyVisible: false,
          baseLayerPicker: false
        })
        
        mapviewer.current = viewer
        console.log('✅ Cesium viewer created')

        pathPositionRef.current = new Cesium.SampledPositionProperty()
        
        const pathEntity = viewer.entities.add({ 
          position: pathPositionRef.current, 
          name: 'Path', 
          path: { 
            show: true, 
            trailTime: CESIUM_CONFIG.PATH_TRAIL_TIME, 
            width: CESIUM_CONFIG.PATH_WIDTH, 
            material: new Cesium.PolylineGlowMaterialProperty({ 
              glowPower: CESIUM_CONFIG.PATH_GLOW_POWER, 
              color: Cesium.Color[CESIUM_CONFIG.PATH_COLOR] 
            }) 
          } 
        })
        
        console.log('📍 Path entity created:', pathEntity)

        try {
          usvModelRef.current = await Cesium.Model.fromGltfAsync({
            url: yildizusv,
            modelMatrix: Cesium.Transforms.headingPitchRollToFixedFrame(initialPosition, new Cesium.HeadingPitchRoll()),
            scale: CESIUM_CONFIG.MODEL_SCALE, 
            minimumPixelSize: CESIUM_CONFIG.MINIMUM_PIXEL_SIZE,
          })
          
          viewer.scene.primitives.add(usvModelRef.current)
          console.log('🚤 USV Model loaded and added')
        } catch (modelError) {
          console.error('USV Model loading error:', modelError)
        }

        setViewerReady(true)
        console.log('🎯 Cesium fully ready!')
        
      } catch (error) {
        console.error('❌ Cesium startup error:', error)
      }
    }

    initializeCesium()

    return () => {
      console.log('🧹 Cesium cleanup...')
      if (intervalRef.current) clearInterval(intervalRef.current)
      if (viewer && !viewer.isDestroyed()) {
        viewer.destroy()
      }
    }
  }, [containerId])

  return {
    viewer: mapviewer.current,
    usvModelRef,
    pathPositionRef,
    intervalRef,
    viewerReady,
    initialPosition,
    fixedFrameTransform
  }
}
