import * as Cesium from 'cesium';

export interface Waypoint {
  id: string | number;
  latitude: number;
  longitude: number;
}

export interface WaypointVisualizerConfig {
  colors: {
    reached: Cesium.Color;
    current: Cesium.Color;
    pending: Cesium.Color;
  };
  animation: {
    blinkInterval: number;
    pixelSize: {
      current: number;
      default: number;
    };
  };
  labels: {
    font: string;
    style: Cesium.LabelStyle;
    outlineWidth: number;
    verticalOrigin: Cesium.VerticalOrigin;
    pixelOffset: Cesium.Cartesian2;
  };
}

export enum WaypointStatus {
  REACHED = 'reached',
  CURRENT = 'current',
  PENDING = 'pending'
}

export interface WaypointEntity {
  id: string | number;
  entity: Cesium.Entity;
  status: WaypointStatus;
  animationId?: NodeJS.Timeout;
}
