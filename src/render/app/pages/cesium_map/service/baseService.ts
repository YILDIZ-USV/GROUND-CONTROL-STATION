import { API_BASE_URL } from '../constants';
import { APIResponse, ServiceEndpoint } from '../types';
import { handleAPIError } from '../utils/errorHandler';

export class BaseService {
  protected baseUrl: string;

  constructor(baseUrl: string = API_BASE_URL) {
    this.baseUrl = baseUrl;
  }

  protected async request<T>(
    endpoint: ServiceEndpoint,
    data?: any,
    options?: RequestInit
  ): Promise<APIResponse<T> | null> {
    try {
      const url = `${this.baseUrl}${endpoint.url}`;
      const config: RequestInit = {
        method: endpoint.method,
        headers: {
          'Content-Type': 'application/json',
          ...options?.headers,
        },
        ...options,
      };

      if (data && endpoint.method !== 'GET') {
        config.body = JSON.stringify(data);
      }

      const response = await fetch(url, config);

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`HTTP ${response.status}: ${errorText}`);
      }

      const result = await response.json();
      return {
        status: 'success',
        data: result,
      };
    } catch (error) {
      handleAPIError(error, endpoint.url);
      throw error;
    }
  }

  protected async get<T>(endpoint: string): Promise<T | null> {
    const response = await this.request<T>({
      url: endpoint,
      method: 'GET',
    });

    return response?.data || null;
  }

  protected async post<T>(
    endpoint: string,
    data?: any,
    options?: RequestInit
  ): Promise<APIResponse<T> | null> {
    return this.request<T>(
      {
        url: endpoint,
        method: 'POST',
      },
      data,
      options
    );
  }

  protected async postFormData<T>(
    endpoint: string,
    formData: FormData
  ): Promise<APIResponse<T> | null> {
    try {
      const url = `${this.baseUrl}${endpoint}`;
      const response = await fetch(url, {
        method: 'POST',
        body: formData,
      });

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(`HTTP ${response.status}: ${errorText}`);
      }

      const result = await response.json();
      return {
        status: 'success',
        data: result,
      };
    } catch (error) {
      handleAPIError(error, endpoint);
      throw error;
    }
  }
}

export enum APIEndpoints {
  GPS_MESSAGE = '/api/gps_message',
  IMU_MESSAGE = '/api/imu_message',
  ODOMETRY = '/api/odometry',
  LINEAR_X = '/api/linear_x',
  ANGULAR_Z = '/api/angular_z',
  ARMED_STATUS = '/api/armed_status',
  MODE_STATUS = '/api/mode_status',
  NAV2_PLAN = '/api/nav2_plan',
  RUN_MISSION = '/api/run_mission',
  RUN_MISSION2 = '/api/run_mission2',
  SAVE_COLOR_CODE = '/api/save_color_code',
  SAVE_WAYPOINTS = '/api/save_waypoints',
}
