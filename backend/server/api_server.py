from fastapi import FastAPI, HTTPException, Form
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from data_store import TelemetryDataStore
from services.config_service import ConfigService
from services.storage_service import StorageService
from services.mission_service import MissionService

class ColorCode(BaseModel):
    """Color code data model"""
    color_code: str

class Waypoint(BaseModel):
    """Waypoint data model with coordinates"""
    latitude: float
    longitude: float


class WaypointsRequest(BaseModel):
    """Waypoints request data model"""
    waypoints: list[Waypoint]
    mission_name: str = "yildizusv_mission"

class APIServer:
    """FastAPI server for Yildizusv telemetry system"""
    
    def __init__(
        self, 
        data_store: TelemetryDataStore,
        config_service: ConfigService,      
        storage_service: StorageService,   
        mission_service: MissionService     
    ):
        """Initialize API server with service dependencies"""
        self.data_store = data_store
        
       
        self.config_service = config_service
        self.storage_service = storage_service
        self.mission_service = mission_service
        
        # Setup FastAPI app
        self.app = FastAPI(title="Yildizusv Gazebo Telemetry Server")
        self._setup_middleware()
        self._setup_routes()
    
    def _setup_middleware(self):
        """Setup CORS and other middleware"""
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
    
    def _setup_routes(self):
        """Setup all API routes - delegates to services for business logic"""
        
        @self.app.get("/")
        async def root():
            """Root endpoint health check"""
            return {"message": "Yildizusv Gazebo Telemetry Server Active"}
        
        @self.app.get("/api/gps_message")
        async def get_gps_message():
            """Get GPS message data from telemetry store"""
            return self.data_store.get_core_data().get("gps", {})
        
        @self.app.get("/api/imu_message")
        async def get_imu_message():
            """Get IMU message data from telemetry store"""
            return self.data_store.get_core_data().get("imu", {})
        
        @self.app.get("/api/linear_x")
        async def get_linear_x():
            """Get linear X velocity data from telemetry store"""
            return {"linear_x": self.data_store.get_core_data().get("linear_x", 0.0)}
        
        @self.app.get("/api/angular_z")
        async def get_angular_z():
            """Get angular Z velocity data from telemetry store"""
            return {"angular_z": self.data_store.get_core_data().get("angular_z", 0.0)}
        
        @self.app.get("/api/odometry")
        async def get_odometry():
            """Get odometry data from telemetry store"""
            return self.data_store.get_core_data().get("odom", {})
        
        @self.app.get("/api/armed_status")
        async def get_armed_status():
            """Get armed status from telemetry store"""
            return {"armed": self.data_store.get_status_data().get("armed", False)}
        
        @self.app.get("/api/mode_status")
        async def get_mode_status():
            """Get mode status from telemetry store"""
            return {"mode": self.data_store.get_status_data().get("mode", "UNKNOWN")}
        
        @self.app.get("/api/nav2_plan")
        async def get_nav2_plan():
            """Get Nav2 plan data from telemetry store"""
            nav2_data = self.data_store.get_nav2_data().get("nav2_plan", {"poses": [], "pose_count": 0})
            
            if not nav2_data.get("poses"):
                raise HTTPException(
                    status_code=204,
                    detail="No Nav2 plan data available from ROS2"
                )
            
            return nav2_data
        
        @self.app.get("/api/all_telemetry")
        async def get_all_telemetry():
            """Get all telemetry data from telemetry store"""
            return self.data_store.get_data()
        
        @self.app.post("/api/save_color_code")
        async def save_color_code(color_code: str = Form(...)):
            """Save color code via storage service"""
            try:
                return self.storage_service.save_color_code(color_code)
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/api/save_waypoints")
        async def save_waypoints(request: WaypointsRequest):
            """Save waypoints via storage service"""
            try:
                waypoints_data = [{"latitude": wp.latitude, "longitude": wp.longitude} for wp in request.waypoints]
                return self.storage_service.save_waypoints(waypoints_data, request.mission_name)
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/api/run_mission")
        async def run_mission():
            """Run waypoint mission via mission service"""
            try:
                return self.mission_service.run_waypoint_mission()
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/api/run_mission2")
        async def run_mission2():
            """Run color code mission via mission service"""
            try:
                return self.mission_service.run_color_code_mission()
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
    
    def get_app(self):
        """Get the FastAPI app instance"""
        return self.app
