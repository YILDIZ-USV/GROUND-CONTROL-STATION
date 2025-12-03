import { useState } from 'react'
import PlaneService from '../service/plane_service'
import { MissionHandlersReturn, Waypoint } from '../types'
import { MESSAGES } from '../constants'
import { handleHookError } from '../utils/errorHandler'
import { useNotification } from '../services/notification.service'

// Hook for managing mission execution handlers and UI state 
export const useMissionHandlers = (service: PlaneService): MissionHandlersReturn => {
  const { showSuccess, showError, showWarning } = useNotification()
  const [isMissionRunning, setIsMissionRunning] = useState(false)
  const [isMission2Running, setIsMission2Running] = useState(false)
  const [colorCode, setColorCode] = useState('')

  // Handle mission execution - runs waypoint publisher mission
  const handleRunMission = async () => {
    setIsMissionRunning(true)
    try {
      const result = await service.runMission()
      if (result && result.status === "success") {
        showSuccess(MESSAGES.MISSION_SUCCESS)
      } else {
        showError(MESSAGES.MISSION_FAILED)
      }
    } catch (error) {
      handleHookError(error, 'useMissionHandlers - handleRunMission')
      showError(MESSAGES.MISSION_ERROR)
    } finally {
      setIsMissionRunning(false)
    }
  }

  // Handle second mission execution - runs color code publisher mission 
  const handleRunMission2 = async () => {
    setIsMission2Running(true)
    try {
      const result = await service.runMission2()
      if (result && result.status === "success") {
        showSuccess("Second mission started successfully!")
      } else {
        showError("Second mission could not be started!")
      }
    } catch (error) {
      handleHookError(error, 'useMissionHandlers - handleRunMission2')
      showError("Error occurred while running second mission!")
    } finally {
      setIsMission2Running(false)
    }
  }

  // Handle color code saving via storage service
  const handleSaveColorCode = async () => {
    if (!colorCode.trim()) {
      showWarning(MESSAGES.COLOR_CODE_EMPTY)
      return
    }
    try {
      const result = await service.saveColorCode(colorCode)
      if (result && result.status === "success") {
        showSuccess(MESSAGES.COLOR_CODE_SUCCESS(result.file_path || ''))
        setColorCode('')
      } else {
        showError(MESSAGES.COLOR_CODE_FAILED)
      }
    } catch (error) {
      handleHookError(error, 'useMissionHandlers - handleSaveColorCode')
      showError("Error occurred while saving color code!")
    }
  }

  // Handle waypoint saving via storage service
  const handleSaveWaypoints = async (waypoints: Waypoint[]) => {
    if (waypoints.length === 0) {
      throw new Error('No waypoints to save found.')
    }
    
    try {
      const result = await service.saveWaypoints(waypoints)
      if (result && result.status === "success") {
        return result
      } else {
        throw new Error('Waypoints could not be saved.')
      }
    } catch (error) {
      handleHookError(error, 'useMissionHandlers - handleSaveWaypoints')
      throw error
    }
  }

  return {
    isMissionRunning,
    isMission2Running,
    colorCode,
    setColorCode,
    handleRunMission,
    handleRunMission2,
    handleSaveColorCode,
    handleSaveWaypoints
  }
}
