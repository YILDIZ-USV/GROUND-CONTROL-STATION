  import * as Cesium from 'cesium';
  import { Waypoint, WaypointStatus, WaypointVisualizerConfig } from '../types/waypoint.types';
  import { DEFAULT_WAYPOINT_CONFIG } from '../constants/waypoint.constants';
  import { WaypointEntityManagerService } from './waypoint-entity-manager.service';
  import { WaypointAnimationService } from './waypoint-animation.service';
  import { handleHookError } from '../utils/errorHandler';

  // Main waypoint visualization service
  export class WaypointVisualizerService {
    private config: WaypointVisualizerConfig;
    private entityManager: WaypointEntityManagerService;
    private animationService: WaypointAnimationService;

    constructor(config?: Partial<WaypointVisualizerConfig>) {
      this.config = { ...DEFAULT_WAYPOINT_CONFIG, ...config };
      this.entityManager = new WaypointEntityManagerService();
      this.animationService = new WaypointAnimationService(this.entityManager);
    }

    // Draws waypoints
    draw(
      viewer: Cesium.Viewer,
      waypoints: Waypoint[],
      reachedWaypoints: Set<string | number>,
      currentTargetIndex: number
    ): void {
      if (!viewer || !viewer.entities) {
        console.warn('WaypointVisualizer: Viewer veya entities mevcut değil');
        return;
      }

      try {
        // First, clean up unnecessary animations
        this.animationService.cleanupAnimations(
          waypoints,
          reachedWaypoints,
          currentTargetIndex,
          this.config
        );

        // Process waypoints
        waypoints.forEach((waypoint, index) => {
          if (!waypoint) return;

          const status = this.determineWaypointStatus(
            waypoint,
            index,
            reachedWaypoints,
            currentTargetIndex
          );

          this.processWaypoint(viewer, waypoint, status, index);
        });


        this.cleanupRemovedWaypoints(viewer, waypoints);

      } catch (error) {
        handleHookError(error, 'WaypointVisualizerService.draw');
      }
    }

    // Clears all waypoints
    clear(viewer: Cesium.Viewer): void {
      if (!viewer) return;

      try {
        // First, stop all animations
        this.animationService.stopAllAnimations();

        // Clear all entities
        this.entityManager.clearAll(viewer);

        console.log('🧹 WaypointVisualizer: All waypoints cleared');
      } catch (error) {
        handleHookError(error, 'WaypointVisualizerService.clear');
      }
    }
  r
    
    updateConfig(newConfig: Partial<WaypointVisualizerConfig>): void {
      this.config = { ...this.config, ...newConfig };
    }

  
    
    getConfig(): WaypointVisualizerConfig {
      return { ...this.config };
    }

    
    
    getEntityManager(): WaypointEntityManagerService {
      return this.entityManager;
    }

    
    getAnimationService(): WaypointAnimationService {
      return this.animationService;
    }

    
    
    private determineWaypointStatus(
      waypoint: Waypoint,
      index: number,
      reachedWaypoints: Set<string | number>,
      currentTargetIndex: number
    ): WaypointStatus {
      if (reachedWaypoints.has(waypoint.id)) {
        return WaypointStatus.REACHED;
      }
      
      if (index === currentTargetIndex) {
        return WaypointStatus.CURRENT;
      }
      
      return WaypointStatus.PENDING;
    }

    // Processes a single waypoint
    
    private processWaypoint(
      viewer: Cesium.Viewer,
      waypoint: Waypoint,
      status: WaypointStatus,
      index: number
    ): void {
      const existingEntity = this.entityManager.getEntity(waypoint.id);

      if (existingEntity) {
        // Update existing entity
        this.entityManager.updateEntity(waypoint.id, status, this.config, index);
      } else {
        // Create new entity
        this.entityManager.createEntity(viewer, waypoint, status, this.config);
      }

      // Start animation for the target waypoint
      if (status === WaypointStatus.CURRENT) {
        this.animationService.startBlinkAnimation(waypoint.id, this.config);
      }
    }

    // Cleans up removed waypoints
    
    private cleanupRemovedWaypoints(viewer: Cesium.Viewer, currentWaypoints: Waypoint[]): void {
      const currentWaypointIds = new Set(currentWaypoints.map(wp => wp.id));
      const existingIds = this.entityManager.getAllIds();

      existingIds.forEach(waypointId => {
        if (!currentWaypointIds.has(waypointId)) {
          this.entityManager.removeEntity(viewer, waypointId);
          this.animationService.stopAnimation(waypointId);
        }
      });
    }
  }
