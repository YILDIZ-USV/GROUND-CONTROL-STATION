import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class WaypointPublisher(Node):
    """ROS2 node for publishing waypoints to telemetry system"""
    
    def __init__(self):
        """Initialize waypoint publisher with timer and data loading"""
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(String, 'waypoint', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.waypoints = self.read_waypoints_json()
        
        self.shutdown_timer = self.create_timer(3000.0, self.shutdown_callback)
        
    def read_waypoints_json(self):
        """Read waypoints from JSON file and return waypoint list"""
        try:
            file_path = os.path.join(os.path.dirname(__file__), "..", "data", "waypoints.json")
            with open(file_path, 'r') as file:
                data = json.load(file)
                waypoints = data.get("waypoints", [])
                mission_name = data.get("mission_name", "unknown_mission")
                
                self.get_logger().info(f'Mission name: {mission_name}')
                self.get_logger().info(f'{len(waypoints)} waypoints found')
                
                return waypoints
        except FileNotFoundError:
            self.get_logger().warn('waypoints.json file not found, using empty waypoint list')
            return []
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse error: {e}, using empty waypoint list')
            return []
        except Exception as e:
            self.get_logger().error(f'File read error: {e}, using empty waypoint list')
            return []

    def timer_callback(self):
        """Timer callback for publishing waypoint data"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish')
            return
            
        waypoint_data = {
            "waypoints": self.waypoints,
            "timestamp": json.dumps(self.waypoints)
        }
        
        msg = String()
        msg.data = json.dumps(waypoint_data)
        self.publisher_.publish(msg)
        
        waypoint_str = "Waypoint'ler: [\n"
        for wp in self.waypoints:
            waypoint_str += f"  ({wp['latitude']}, {wp['longitude']})\n"
        waypoint_str += "]"
        self.get_logger().info(f'Publishing waypoints: {waypoint_str}')

    def shutdown_callback(self):
        """Timer callback for node shutdown"""
        self.get_logger().info('30 seconds elapsed, shutting down node...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """Main function for waypoint publisher node"""
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
