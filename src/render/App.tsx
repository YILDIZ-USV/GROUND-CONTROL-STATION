import { useDispatch } from 'react-redux';
import CesiumMap from './app/pages/cesium_map/CesiumMap';
import FlightIndicator from './app/pages/cesium_map/components/flight-indicator/flight_indicator';
import WaypointEditor from './app/pages/cesium_map/WaypointEditor';
import WaypointEditorTrigger from './app/pages/cesium_map/components/WaypointEditorTrigger';
import { NotificationProvider } from './app/pages/cesium_map/services/notification.service';
import { openEditor, closeEditor } from './app/store/editorSlice';
import { saveWaypointsThunk } from './app/store/waypointsSlice';
import type { Waypoint } from './app/pages/cesium_map/types/waypoint.types';
import './App.css';

function App() {
  const dispatch = useDispatch();
  


  // Editor management - Redux actions
  const handleOpenEditor = () => {
    dispatch(openEditor());
  };

  const handleCloseEditor = () => {
    dispatch(closeEditor());
  };

  // Real backend save using Redux Thunk
  const handleSaveWaypoints = async (waypoints: Waypoint[]) => {
    // Dispatch the actual save thunk that uses PlaneService
    return dispatch(saveWaypointsThunk(waypoints) as any);
  };

  return (
    <NotificationProvider>
      <div className="stack-container">

        <WaypointEditorTrigger onOpen={handleOpenEditor} />

        <div className="widget1">
          
          <CesiumMap />
        </div>
        <div className="widget2">
          <FlightIndicator />
        </div>

       
        <WaypointEditor
          onClose={handleCloseEditor}
          onSaveWaypoints={handleSaveWaypoints}
        />
      </div>
    </NotificationProvider>
  );
}

export default App;
