import { Waypoint } from '../types/waypoint.types';

export interface ParseResult {
  waypoints: Waypoint[];
  errors: string[];
}

export const parseWaypointTxt = (content: string): ParseResult => {
  const lines = content.trim().split('\n').filter(line => line.trim());
  const waypoints: Waypoint[] = [];
  const errors: string[] = [];

  lines.forEach((line, index) => {
    const parts = line.split(',').map(s => s.trim());
    if (parts.length !== 2) {
      errors.push(`Line ${index + 1}: Invalid format (expected lat,lon): ${line}`);
      return;
    }

    const lat = parseFloat(parts[0]);
    const lon = parseFloat(parts[1]);
    if (Number.isNaN(lat) || Number.isNaN(lon)) {
      errors.push(`Line ${index + 1}: Invalid numbers: ${line}`);
      return;
    }

    waypoints.push({
      id: `wp_${index}`,
      latitude: lat,
      longitude: lon
    });
  });

  return { waypoints, errors };
};
