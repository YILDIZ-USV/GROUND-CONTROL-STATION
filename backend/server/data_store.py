import threading
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Bool as BoolMsg

class TelemetryDataStore:
    """Telemetry data store for handling ROS message processing"""
    
    def __init__(self):
        """Initialize telemetry data store with thread-safe data structures"""
        self.core_lock = threading.Lock()
        self.nav2_lock = threading.Lock()
        self.status_lock = threading.Lock()
        
        self.imu_data = {}
        self.odom_data = {}
        self.linear_x_data = {}
        self.angular_z_data = {}
        self.gps_data = {}
        self.armed_data = {}
        self.mode_data = {}
        self.nav2_plan_data = {}

    def update_imu(self, msg: Imu):
        """Update IMU data with quaternion to Euler angle conversion"""
        with self.core_lock:
            try:
                x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
                
                roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                sin_pitch = 2*(w*y - z*x)
                pitch = math.copysign(math.pi/2, sin_pitch) if abs(sin_pitch) >= 1 else math.asin(sin_pitch)
                yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                
                self.imu_data = {"imu": {"yaw": yaw, "pitch": pitch, "roll": roll}}
            except Exception as e:
                print(f"Quaternion conversion error: {e}")
                self.imu_data = {"imu": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}}

    def update_odom(self, msg: Odometry):
        """Update odometry data with velocity and position information"""
        with self.core_lock:
            self.odom_data = {
                "odom": {
                    "linear_velocity": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y, 
                        "z": msg.twist.twist.linear.z
                    },
                    "angular_velocity": {
                        "x": msg.twist.twist.angular.x,
                        "y": msg.twist.twist.angular.y,
                        "z": msg.twist.twist.angular.z
                    },
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z
                    }
                }
            }

    def update_cmd_vel(self, msg: Twist):
        """Update command velocity data"""
        with self.core_lock:
            self.linear_x_data = {"linear_x": msg.linear.x}
            self.angular_z_data = {"angular_z": msg.angular.z}

    def update_gps(self, msg: NavSatFix):
        """Update GPS data with location coordinates"""
        with self.core_lock:
            self.gps_data = {
                "gps": {
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "altitude": msg.altitude
                }
            }

    def update_armed(self, msg: BoolMsg):
        """Update armed status data"""
        with self.status_lock:
            self.armed_data = {"armed": msg.data}

    def update_mode(self, msg: StringMsg):
        """Update mode status data"""
        with self.status_lock:
            self.mode_data = {"mode": msg.data}

    def update_nav2_plan(self, msg: Path):
        """Update Nav2 plan data with waypoint poses"""
        with self.nav2_lock:
            try:
                plan_poses = []
                for pose_stamped in msg.poses:
                    pose_data = {
                        "position": {
                            "x": pose_stamped.pose.position.x,
                            "y": pose_stamped.pose.position.y,
                            "z": pose_stamped.pose.position.z
                        },
                        "orientation": {
                            "x": pose_stamped.pose.orientation.x,
                            "y": pose_stamped.pose.orientation.y,
                            "z": pose_stamped.pose.orientation.z,
                            "w": pose_stamped.pose.orientation.w
                        },
                        "header": {
                            "frame_id": pose_stamped.header.frame_id,
                            "stamp": pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
                        }
                    }
                    plan_poses.append(pose_data)
                
                self.nav2_plan_data = {
                    "nav2_plan": {
                        "header": {
                            "frame_id": msg.header.frame_id,
                            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        },
                        "poses": plan_poses,
                        "pose_count": len(plan_poses)
                    }
                }
            except Exception as e:
                print(f"Nav2 plan parse error: {e}")
                self.nav2_plan_data = {"nav2_plan": {"poses": [], "pose_count": 0}}

    def get_core_data(self):
        """Get core telemetry data (IMU, GPS, velocity, odometry)"""
        with self.core_lock:
            return {
                **self.imu_data,
                **self.odom_data,
                **self.linear_x_data,
                **self.angular_z_data,
                **self.gps_data
            }
    
    def get_status_data(self):
        """Get status data (armed status, mode)"""
        with self.status_lock:
            return {
                **self.armed_data,
                **self.mode_data
            }
    
    def get_nav2_data(self):
        """Get Nav2 plan data"""
        with self.nav2_lock:
            return self.nav2_plan_data
    
    def get_data(self):
        """Get all telemetry data"""
        with self.core_lock:
            core_data = {
                **self.imu_data,
                **self.odom_data,
                **self.linear_x_data,
                **self.angular_z_data,
                **self.gps_data
            }
        
        with self.status_lock:
            status_data = {
                **self.armed_data,
                **self.mode_data
            }
        
        with self.nav2_lock:
            nav2_data = self.nav2_plan_data
        
        return {**core_data, **status_data, **nav2_data}
