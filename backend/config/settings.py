import os
from pathlib import Path

class Settings:
    """Application configuration settings"""
    
    # Base directories
    BASE_DIR = Path(__file__).parent.parent
    DATA_DIR = BASE_DIR / "data"
    ROS_NODES_DIR = BASE_DIR / "ros_nodes"
    
    # Data files
    WAYPOINTS_FILE = "waypoints.json"
    COLOR_CODE_FILE = "color_code.json"
    
    # ROS scripts
    WAYPOINT_SCRIPT = "waypoint_publisher.py"
    COLOR_CODE_SCRIPT = "color_code_publisher.py"
    
    # Server settings - 
    HOST = os.environ.get('HOST', '0.0.0.0')
    PORT = int(os.environ.get('PORT', 5002))
    
    # Mission settings
    SCRIPT_TIMEOUT = 300  # seconds
    
    @classmethod
    def get_data_file_path(cls, filename: str) -> str:
        """Get full path to data file"""
        return str(cls.DATA_DIR / filename)
    
    @classmethod
    def get_script_path(cls, script_name: str) -> str:
        """Get full path to ROS script"""
        return str(cls.ROS_NODES_DIR / script_name)
    
    @classmethod
    def ensure_data_dir_exists(cls):
        """Ensure data directory exists"""
        cls.DATA_DIR.mkdir(exist_ok=True)

