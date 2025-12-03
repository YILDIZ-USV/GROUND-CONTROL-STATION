
import { WaypointEntityManagerService } from './waypoint-entity-manager.service';

// Waypoint animation management service
 
export class WaypointAnimationService {
  private animationIntervals = new Map<string | number, NodeJS.Timeout>();

  constructor(private entityManager: WaypointEntityManagerService) {}

  // Starts blink animation for the target waypoint

  startBlinkAnimation(
    waypointId: string | number,
    config: any
  ): void {
    // If animation is already running, do not restart
    if (this.animationIntervals.has(waypointId)) {
      return;
    }

    const entity = this.entityManager.getEntity(waypointId);
    if (!entity || !entity.point) return;

    let isGreen = true;
    const intervalId = setInterval(() => {
      if (entity.point) {
        // Change color: Green -> Red -> Green ...
        entity.point.color = isGreen ? config.colors.pending : config.colors.current;
        isGreen = !isGreen;
      }
    }, config.animation.blinkInterval);

    this.animationIntervals.set(waypointId, intervalId);
  }

  // Stops animation for a specific waypoint
   
  stopAnimation(waypointId: string | number): void {
    const intervalId = this.animationIntervals.get(waypointId);
    if (intervalId) {
      clearInterval(intervalId);
      this.animationIntervals.delete(waypointId);
    }
  }

  // Stops all animations
   
  stopAllAnimations(): void {
    this.animationIntervals.forEach((intervalId) => {
      clearInterval(intervalId);
    });
    this.animationIntervals.clear();
  }

  // Cleans up unnecessary animations. Only animations for the target waypoint and unreached waypoints continue
   
  cleanupAnimations(
    waypoints: any[],
    reachedWaypoints: Set<string | number>,
    currentTargetIndex: number,
    config: any
  ): void {
    const currentTargetWaypoint = waypoints[currentTargetIndex];
    const currentTargetId = currentTargetWaypoint ? currentTargetWaypoint.id : null;

    // Check all animations
    this.animationIntervals.forEach((_intervalId, waypointId) => {
      const isReached = reachedWaypoints.has(waypointId);
      const isCurrentTarget = waypointId === currentTargetId;

      // If this waypoint is no longer the target or has been reached, stop the animation
      if (!isCurrentTarget || isReached) {
        this.stopAnimation(waypointId);

        // Reset the waypoint's color to normal
        const entity = this.entityManager.getEntity(waypointId);
        if (entity && entity.point) {
          if (isReached) {
            entity.point.color = config.colors.reached;
          } else {
            entity.point.color = config.colors.pending;
          }
        }
      }
    });
  }

  // Checks if animation is running
   
  isAnimating(waypointId: string | number): boolean {
    return this.animationIntervals.has(waypointId);
  }

  // Returns the count of active animations
 
  getActiveAnimationCount(): number {
    return this.animationIntervals.size;
  }

  // Returns all animation IDs

  getAnimatingWaypointIds(): (string | number)[] {
    return Array.from(this.animationIntervals.keys());
  }
}
