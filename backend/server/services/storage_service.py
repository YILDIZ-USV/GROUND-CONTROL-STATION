import json
import os
from datetime import datetime
from typing import Dict, Any, List
from .config_service import ConfigService

class StorageService:
    """Service for handling file storage operations"""
    
    def __init__(self, config_service: ConfigService):
        self.config_service = config_service
    
    def save_waypoints(self, waypoints: List[Dict[str, float]], mission_name: str = "yildizusv_mission") -> Dict[str, Any]:
        """Save waypoints to JSON file"""
        try:
            waypoints_data = {
                "waypoints": waypoints,
                "timestamp": datetime.now().isoformat() + "Z",
                "mission_name": mission_name
            }
            
            file_path = self.config_service.get_waypoints_file_path()
            with open(file_path, "w") as file:
                json.dump(waypoints_data, file, indent=2)
            
            return {
                "status": "success", 
                "message": "Waypoints saved in JSON format", 
                "data": waypoints_data
            }
        except Exception as e:
            raise Exception(f"Waypoints could not be saved: {str(e)}")
    
    def save_color_code(self, color_code: str) -> Dict[str, Any]:
        """Save color code to JSON file"""
        try:
            color_data = {
                "color_code": color_code,
                "timestamp": datetime.now().isoformat() + "Z",
                "status": "active"
            }
            
            file_path = self.config_service.get_color_code_file_path()
            with open(file_path, "w") as file:
                json.dump(color_data, file, indent=2)
            
            return {
                "status": "success", 
                "message": "Color code saved in JSON format", 
                "data": color_data
            }
        except Exception as e:
            raise Exception(f"Color code could not be saved: {str(e)}")
    
    def load_waypoints(self) -> Dict[str, Any]:
        """Load waypoints from JSON file"""
        try:
            file_path = self.config_service.get_waypoints_file_path()
            if not os.path.exists(file_path):
                return {"waypoints": [], "mission_name": "", "timestamp": ""}
            
            with open(file_path, "r") as file:
                return json.load(file)
        except FileNotFoundError:
            return {"waypoints": [], "mission_name": "", "timestamp": ""}
        except json.JSONDecodeError:
            return {"waypoints": [], "mission_name": "", "timestamp": ""}
        except Exception as e:
            raise Exception(f"Could not load waypoints: {str(e)}")
    
    def load_color_code(self) -> Dict[str, Any]:
        """Load color code from JSON file"""
        try:
            file_path = self.config_service.get_color_code_file_path()
            if not os.path.exists(file_path):
                return {"color_code": "#FF0000", "timestamp": "", "status": "inactive"}
            
            with open(file_path, "r") as file:
                return json.load(file)
        except FileNotFoundError:
            return {"color_code": "#FF0000", "timestamp": "", "status": "inactive"}
        except json.JSONDecodeError:
            return {"color_code": "#FF0000", "timestamp": "", "status": "inactive"}
        except Exception as e:
            raise Exception(f"Could not load color code: {str(e)}")
