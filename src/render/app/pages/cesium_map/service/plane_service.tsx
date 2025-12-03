import { BaseService, APIEndpoints } from './baseService';
import { 
  GPSData, 
  IMUData, 
  OdomData,
  LinearX, 
  AngularZ, 
  ArmedStatus, 
  ModeStatus, 
  APIResponse,
  Waypoint,
  WaypointsRequest,
  Nav2PlanData
} from '../types';

// USV (Unmanned Surface Vehicle) API service 
class PlaneService extends BaseService {
  
  // Get GPS data from telemetry store
  async getGPSVerileri(): Promise<GPSData | null> {
    return this.get<GPSData>(APIEndpoints.GPS_MESSAGE);
  }

  // Get IMU data from telemetry store
  async getIMUVerileri(): Promise<IMUData | null> {
    return this.get<IMUData>(APIEndpoints.IMU_MESSAGE);
  }

  // Get odometry data from telemetry store
  async getOdometryVerileri(): Promise<OdomData | null> {
    return this.get<OdomData>(APIEndpoints.ODOMETRY);
  }

  // Get linear X velocity data from telemetry store
  async getLinearX(): Promise<LinearX | null> {
    return this.get<LinearX>(APIEndpoints.LINEAR_X);
  }

  // Get angular Z velocity data from telemetry store
  async getAngularZ(): Promise<AngularZ | null> {
    return this.get<AngularZ>(APIEndpoints.ANGULAR_Z);
  }

  // Get armed status from telemetry store
  async getArmedStatus(): Promise<ArmedStatus | null> {
    return this.get<ArmedStatus>(APIEndpoints.ARMED_STATUS);
  }

  // Get mode status from telemetry store
  async getModeStatus(): Promise<ModeStatus | null> {
    return this.get<ModeStatus>(APIEndpoints.MODE_STATUS);
  }

  // Get Nav2 plan data from telemetry store
  async getNav2Plan(): Promise<Nav2PlanData | null> {
    return this.get<Nav2PlanData>(APIEndpoints.NAV2_PLAN);
  }

  // Run waypoint mission via mission service
  async runMission(): Promise<APIResponse | null> {
    return this.post(APIEndpoints.RUN_MISSION);
  }

  // Run color code mission via mission service
  async runMission2(): Promise<APIResponse | null> {
    return this.post(APIEndpoints.RUN_MISSION2);
  }

  // Save color code via storage service
  async saveColorCode(colorCode: string): Promise<APIResponse | null> {
    const formData = new FormData();
    formData.append('color_code', colorCode);
    
    return this.postFormData(APIEndpoints.SAVE_COLOR_CODE, formData);
  }

  // Save waypoints via storage service
  async saveWaypoints(waypoints: Waypoint[], missionName: string = "yildizusv_mission"): Promise<APIResponse | null> {
    const waypointsRequest: WaypointsRequest = {
      waypoints: waypoints.map(wp => ({
        latitude: wp.latitude,
        longitude: wp.longitude
      })),
      mission_name: missionName
    };
    
    return this.post(APIEndpoints.SAVE_WAYPOINTS, waypointsRequest);
  }
}

export default PlaneService;
