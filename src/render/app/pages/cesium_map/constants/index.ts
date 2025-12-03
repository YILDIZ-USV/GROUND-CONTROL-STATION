export const DEFAULT_LOCATION = {
  longitude: 29.0046,
  latitude: 41.0211,
  altitude: 0
} as const;

export const ARRIVAL_THRESHOLD_METERS = 3.0;


const API_PORT = import.meta.env.VITE_API_PORT || 5002;
export const API_BASE_URL = `http://${window.location.hostname}:${API_PORT}`;

export const CESIUM_CONFIG = {
  HEADING_OFFSET: -1.58,
  DEFAULT_ALTITUDE: 0,
  UPDATE_INTERVAL_MS: 100,
  MODEL_SCALE: 0.01,
  MINIMUM_PIXEL_SIZE: 128,
  PATH_TRAIL_TIME: 3,
  PATH_WIDTH: 8,
  PATH_GLOW_POWER: 0.4,
  PATH_COLOR: 'CYAN',
} as const;

export const NAV2_UPDATE_INTERVAL_MS = 2000;

export const UI_POSITIONS = {
  TOP_LEFT: { top: '10px', left: '10px' },
  TOP_RIGHT: { top: '10px', right: '10px' },
  BOTTOM_LEFT: { bottom: '10px', left: '10px' },
  BOTTOM_RIGHT: { bottom: '10px', right: '10px' }
} as const;

export const COLORS = {
  PRIMARY: '#007bff',
  DISABLED: '#ccc',
  SUCCESS: '#28a745',
  WARNING: '#ffc107',
  DANGER: '#dc3545'
} as const;

export const MESSAGES = {
  MISSION_SUCCESS: "Mission started successfully!",
  MISSION_FAILED: "Mission could not be started!",
  MISSION_ERROR: "Error occurred while running mission!",
  COLOR_CODE_SUCCESS: (filePath: string) => `Color code saved successfully!\n${filePath}`,
  COLOR_CODE_FAILED: "Color code could not be saved!",
  COLOR_CODE_EMPTY: "Please enter a color code!"
} as const;
