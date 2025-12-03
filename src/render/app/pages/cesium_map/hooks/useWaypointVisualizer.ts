import { useEffect, useMemo, useRef } from 'react';
import * as Cesium from 'cesium';
import { Waypoint, WaypointVisualizerConfig } from '../types/waypoint.types';
import { WaypointVisualizerService } from '../services/waypoint-visualizer.service';

// Waypoint visualization hook

export const useWaypointVisualizer = (
  viewer: Cesium.Viewer | null,
  config?: Partial<WaypointVisualizerConfig>
) => {
  const visualizerRef = useRef<WaypointVisualizerService | null>(null);

  // Create the service only once
  const visualizer = useMemo(() => {
    if (!visualizerRef.current) {
      visualizerRef.current = new WaypointVisualizerService(config);
    }
    return visualizerRef.current;
  }, [config]);

  // Update configuration
  useEffect(() => {
    if (config && visualizer) {
      visualizer.updateConfig(config);
    }
  }, [config, visualizer]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (viewer && visualizer) {
        visualizer.clear(viewer);
      }
    };
  }, [viewer, visualizer]);

  // Draw waypoints

  const drawWaypoints = (
    waypoints: Waypoint[],
    reachedWaypoints: Set<string | number>,
    currentTargetIndex: number
  ): void => {
    if (!viewer || !visualizer) return;

    console.log('🎨 Drawing waypoints:', {
      count: waypoints.length,
      reached: reachedWaypoints.size,
      targetIndex: currentTargetIndex
    });

    visualizer.draw(viewer, waypoints, reachedWaypoints, currentTargetIndex);
  };

  // Clear all waypoints
  const clearWaypoints = (): void => {
    if (!viewer || !visualizer) return;
    visualizer.clear(viewer);
  };

  // Update configuration
  const updateConfig = (newConfig: Partial<WaypointVisualizerConfig>): void => {
    if (!visualizer) return;
    visualizer.updateConfig(newConfig);
  };

  // Gets the current configuration
   
  const getConfig = (): WaypointVisualizerConfig | undefined => {
    if (!visualizer) return undefined;
    return visualizer.getConfig();
  };

  // Access to the entity manager
  const getEntityManager = () => {
    if (!visualizer) return null;
    return visualizer.getEntityManager();
  };

  // Access to the animation service

  const getAnimationService = () => {
    if (!visualizer) return null;
    return visualizer.getAnimationService();
  };

  return {
    drawWaypoints,
    clearWaypoints,
    updateConfig,
    getConfig,
    getEntityManager,
    getAnimationService,
  };
};


export type UseWaypointVisualizerReturn = ReturnType<typeof useWaypointVisualizer>;
