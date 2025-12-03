import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os

class ColorCodePublisher(Node):
    """ROS2 node for publishing color codes to telemetry system"""
    
    def __init__(self):
        """Initialize color code publisher with timer and data loading"""
        super().__init__('color_code_publisher')
        self.publisher_ = self.create_publisher(String, 'color_code', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.color_code = self.read_color_code_json()
        
        self.shutdown_timer = self.create_timer(3000.0, self.shutdown_callback)
        
    def read_color_code_json(self):
        """Read color code from JSON file and return color string"""
        try:
            file_path = os.path.join(os.path.dirname(__file__), "..", "data", "color_code.json")
            with open(file_path, 'r') as file:
                data = json.load(file)
                return data.get("color_code", "#FF0000")
        except FileNotFoundError:
            self.get_logger().warn('color_code.json file not found, using default color')
            return "#FF0000"
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse error: {e}, using default color')
            return "#FF0000"
        except Exception as e:
            self.get_logger().error(f'File read error: {e}, using default color')
            return "#FF0000"

    def timer_callback(self):
        """Timer callback for publishing color code data"""
        msg = String()
        msg.data = self.color_code
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing color code: %s' % msg.data)

    def shutdown_callback(self):
        """Timer callback for node shutdown"""
        self.get_logger().info('30 seconds elapsed, shutting down node...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """Main function for color code publisher node"""
    rclpy.init(args=args)
    color_code_publisher = ColorCodePublisher()
    rclpy.spin(color_code_publisher)
    color_code_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
