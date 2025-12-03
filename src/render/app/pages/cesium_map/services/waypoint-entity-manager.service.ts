import * as Cesium from 'cesium';
import { Waypoint, WaypointEntity, WaypointStatus } from '../types/waypoint.types';
import { WAYPOINT_ENTITY_PREFIX } from '../constants/waypoint.constants';

// Cesium entity management service
 
 
export class WaypointEntityManagerService {
  private entities = new Map<string | number, WaypointEntity>();

  // Creates a new waypoint entity
  
  createEntity(
    viewer: Cesium.Viewer,
    waypoint: Waypoint,
    status: WaypointStatus,
    config: any
  ): Cesium.Entity {
    const entityId = `${WAYPOINT_ENTITY_PREFIX}${waypoint.id}`;
    
    // Check if entity already exists and remove it to prevent duplicate errors
    const existingEntity = viewer.entities.getById(entityId);
    if (existingEntity) {
      viewer.entities.remove(existingEntity);
      this.entities.delete(waypoint.id);
      console.log(`Removed existing entity with ID: ${entityId}`);
    }
    
    const entity = viewer.entities.add({
      id: entityId,
      position: Cesium.Cartesian3.fromDegrees(waypoint.longitude, waypoint.latitude, 0),
      point: {
        pixelSize: this.getPixelSize(status, config),
        color: this.getColor(status, config),
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        heightReference: Cesium.HeightReference.CLAMP_TO_GROUND
      },
      label: {
        text: this.getLabelText(waypoint),
        font: config.labels.font,
        style: config.labels.style,
        outlineWidth: config.labels.outlineWidth,
        verticalOrigin: config.labels.verticalOrigin,
        pixelOffset: config.labels.pixelOffset,
      },
    });

    const waypointEntity: WaypointEntity = {
      id: waypoint.id,
      entity,
      status
    };

    this.entities.set(waypoint.id, waypointEntity);
    console.log(`✅ Created new entity with ID: ${entityId}`);
    return entity;
  }

  // Updates the existing entity
   
  updateEntity(
    waypointId: string | number,
    status: WaypointStatus,
    config: any,
    index?: number
  ): void {
    const waypointEntity = this.entities.get(waypointId);
    if (!waypointEntity) return;

    const { entity } = waypointEntity;
    
    if (entity.point) {
      (entity.point as any).color = this.getColor(status, config);
      (entity.point as any).pixelSize = this.getPixelSize(status, config);
    }

    if (entity.label) {
      (entity.label as any).text = this.getLabelText(
        { id: waypointId, latitude: 0, longitude: 0 }, 
        index
      );
    }

    waypointEntity.status = status;
  }

  // Removes the entity
  removeEntity(viewer: Cesium.Viewer, waypointId: string | number): void {
    const waypointEntity = this.entities.get(waypointId);
    if (!waypointEntity) return;

    viewer.entities.remove(waypointEntity.entity);
    this.entities.delete(waypointId);
  }

  // Clears all entities
  
  clearAll(viewer: Cesium.Viewer): void {
    this.entities.forEach((waypointEntity) => {
      viewer.entities.remove(waypointEntity.entity);
    });
    this.entities.clear();
  }

  // Gets the entity
   
  getEntity(waypointId: string | number): Cesium.Entity | undefined {
    const waypointEntity = this.entities.get(waypointId);
    return waypointEntity?.entity;
  }

  // Gets the waypoint entity
   
  getWaypointEntity(waypointId: string | number): WaypointEntity | undefined {
    return this.entities.get(waypointId);
  }

  // Gets all entity IDs
   
  getAllIds(): (string | number)[] {
    return Array.from(this.entities.keys());
  }

  // Returns color based on status
   
  private getColor(status: WaypointStatus, config: any): Cesium.Color {
    switch (status) {
      case WaypointStatus.REACHED:
        return config.colors.reached;
      case WaypointStatus.CURRENT:
        return config.colors.current;
      case WaypointStatus.PENDING:
        return config.colors.pending;
      default:
        return config.colors.pending;
    }
  }

  // Returns pixel size based on status
  private getPixelSize(status: WaypointStatus, config: any): number {
    return status === WaypointStatus.CURRENT 
      ? config.animation.pixelSize.current 
      : config.animation.pixelSize.default;
  }

  // Returns label text
   
  private getLabelText(waypoint: Waypoint, index?: number): string {
    const indexText = index !== undefined ? `WP ${index + 1}` : `WP ${waypoint.id}`;
    return `${indexText}`;
  }
}
