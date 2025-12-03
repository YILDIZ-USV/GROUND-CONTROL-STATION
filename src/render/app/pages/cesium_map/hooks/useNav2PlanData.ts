import { useState, useEffect, useCallback } from 'react'
import { NAV2_UPDATE_INTERVAL_MS } from '../constants'
import { Nav2PlanData } from '../types'

export const useNav2PlanData = (service: any, viewerReady: boolean) => {
  const [nav2PlanData, setNav2PlanData] = useState<Nav2PlanData | null>(null)
  const [isPlanLoading, setIsPlanLoading] = useState(false)
  const [planError, setPlanError] = useState<string | null>(null)

  const fetchNav2PlanData = useCallback(async () => {
    if (!viewerReady || !service) return

    try {
      
      setPlanError(null)
      
      const response = await service.getNav2Plan()
      
      if (response) {
        setNav2PlanData(response)
        setIsPlanLoading(false) 
      }
    } catch (error: any) {
      
      const msg = error?.message || '';
      
      
      if (msg.includes('Failed to fetch') || msg.includes('Network Error')) {
        setPlanError('Connection lost') 
        setNav2PlanData(null)
        setIsPlanLoading(false)
        return; 
      }
     

      console.error('Error while fetching Nav2 plan data:', error)
      setPlanError('Plan data could not be retrieved')
      setNav2PlanData(null)
      setIsPlanLoading(false)
    } 
  
  }, [viewerReady, service])

  useEffect(() => {
    if (!viewerReady) return

    const interval = setInterval(() => {
      fetchNav2PlanData()
    }, NAV2_UPDATE_INTERVAL_MS) 

   
    fetchNav2PlanData()

    return () => {
      clearInterval(interval)
    }
  }, [viewerReady]) 

  return {
    nav2PlanData,
    isPlanLoading,
    planError,
    refetchPlan: fetchNav2PlanData
  }
}