from config.settings import Settings

class ConfigService:
    """Service for managing application configuration and paths"""
    
    def __init__(self):
        self.settings = Settings
        
        self.settings.ensure_data_dir_exists()
    
    def get_data_file_path(self, filename: str) -> str:
        """Get full path to data file"""
        return self.settings.get_data_file_path(filename)
    
    def get_script_path(self, script_name: str) -> str:
        """Get full path to ROS script"""
        return self.settings.get_script_path(script_name)
    
    def get_waypoints_file_path(self) -> str:
        """Get waypoints file path"""
        return self.get_data_file_path(self.settings.WAYPOINTS_FILE)
    
    def get_color_code_file_path(self) -> str:
        """Get color code file path"""
        return self.get_data_file_path(self.settings.COLOR_CODE_FILE)
    
    def get_waypoint_script_path(self) -> str:
        """Get waypoint publisher script path"""
        return self.get_script_path(self.settings.WAYPOINT_SCRIPT)
    
    def get_color_code_script_path(self) -> str:
        """Get color code publisher script path"""
        return self.get_script_path(self.settings.COLOR_CODE_SCRIPT)
    
    def get_script_timeout(self) -> int:
        """Get script execution timeout"""
        return self.settings.SCRIPT_TIMEOUT
