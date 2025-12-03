import * as Cesium from 'cesium';
import { WaypointVisualizerConfig } from '../types/waypoint.types';

export const DEFAULT_WAYPOINT_CONFIG: WaypointVisualizerConfig = {
  colors: {
    reached: Cesium.Color.GREEN,
    current: Cesium.Color.GREEN,
    pending: Cesium.Color.RED,
  },
  animation: {
    blinkInterval: 200,
    pixelSize: {
      current: 20,
      default: 12,
    },
  },
  labels: {
    font: '14pt monospace',
    style: Cesium.LabelStyle.FILL_AND_OUTLINE,
    outlineWidth: 2,
    verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
    pixelOffset: new Cesium.Cartesian2(0, -25),
  },
};

export const WAYPOINT_ENTITY_PREFIX = 'waypoint-';

