import React, { useState, useEffect, useRef } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import './WaypointEditor.css';
import { useNotification } from './services/notification.service';
import { Waypoint } from './types/waypoint.types';
import { 
  addEditorWaypoint, 
  removeEditorWaypoint, 
  applyEditorWaypoints,
  setWaypointsLoading, 
  setWaypointsError,
  clearEditorWaypoints,
  setEditorWaypoints
} from '../../store/waypointsSlice';
import type { RootState } from '../../store';
import { parseWaypointTxt } from './utils/waypointParser';

interface WaypointEditorProps { 
  onClose: () => void; 
  onSaveWaypoints?: (waypoints: Waypoint[]) => Promise<void>;
}

const WaypointEditor: React.FC<WaypointEditorProps> = ({ onClose, onSaveWaypoints }) => {
  const dispatch = useDispatch();
  const { showSuccess, showError, showWarning } = useNotification();
  
  const fileInputRef = useRef<HTMLInputElement>(null);
  
  // Get state from Redux store - single source of truth
  const isOpen = useSelector((state: RootState) => state.editor.isOpen);
  const editorWaypoints = useSelector((state: RootState) => state.waypoints.editorWaypoints);
  const isLoading = useSelector((state: RootState) => state.waypoints.isLoading);
  const error = useSelector((state: RootState) => state.waypoints.error);
  
  const [lat, setLat] = useState('');
  const [lon, setLon] = useState('');

  // Reset form when editor opens
  useEffect(() => { if (isOpen) { setLat(''); setLon(''); } }, [isOpen]);
  
  if (!isOpen) return null;

  const handleAddWaypoint = () => {
    const latitude = parseFloat(lat); const longitude = parseFloat(lon);
    if (isNaN(latitude) || isNaN(longitude)) { 
      showWarning('Please enter valid latitude and longitude values.'); 
      return; 
    }
    const newWaypoint: Waypoint = { 
      id: Date.now(), 
      latitude, 
      longitude 
    };
    // Use editor waypoint action - doesn't affect map immediately
    dispatch(addEditorWaypoint(newWaypoint)); 
    setLat(''); setLon('');
  };

  const handleRemoveWaypoint = (id: string | number) => { 
    // Use editor waypoint action
    dispatch(removeEditorWaypoint(id)); 
  };

  const handleSaveToBackend = async () => {
    if (!onSaveWaypoints) {
      showError('waypoint save function not found.');
      return;
    }
    
    if (editorWaypoints.length === 0) {
      showWarning('No waypoints to save.');
      return;
    }

    // Use Redux for loading state management
    dispatch(setWaypointsLoading(true));
    dispatch(setWaypointsError(null));
    
    try {
      await onSaveWaypoints(editorWaypoints);
      showSuccess('Waypoints successfully saved');
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      dispatch(setWaypointsError(errorMessage));
      showError(`Save error: ${errorMessage}`);
    } finally {
      dispatch(setWaypointsLoading(false));
    }
  };

  const handleLoadFile = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    try {
      const text = await file.text();
      const { waypoints, errors } = parseWaypointTxt(text);

      dispatch(clearEditorWaypoints());
      
      if (waypoints.length > 0) {
        dispatch(setEditorWaypoints(waypoints));
        showSuccess(`${waypoints.length} waypoints loaded successfully.`);
      } else {
        showWarning('No valid waypoints found.');
      }

      if (errors.length > 0) {
        showWarning(`Some errors: ${errors.slice(0, 3).join('; ')}${errors.length > 3 ? '...' : ''}`);
      }
    } catch (error) {
      showError('File read error.');
    } finally {
      e.target.value = '';
    }
  };

  // Apply to map - move editor waypoints to applied waypoints
  const handleApply = () => { 
    dispatch(applyEditorWaypoints());
    onClose(); 
  };

  return (
    <div className="waypoint-editor-container">
      <div className="waypoint-editor-header">
        <h2>Waypoint Editor</h2>
        <button onClick={onClose} className="waypoint-editor-close-button">✕</button>
      </div>
      
      {error && (
        <div className="waypoint-editor-error">
          Error: {error}
        </div>
      )}
      
      <div className="waypoint-editor-list-container">
        {editorWaypoints.length === 0 ? <p className="waypoint-editor-no-waypoints">No waypoints added yet.</p> :
          editorWaypoints.map((wp, index) => (
            <div key={wp.id} className="waypoint-editor-waypoint-item">
              <span>{index + 1}: Lat: {wp.latitude.toFixed(6)}, Lon: {wp.longitude.toFixed(6)}</span>
              <button onClick={() => handleRemoveWaypoint(wp.id)} className="waypoint-editor-remove-button">Delete</button>
            </div>
          ))
        }
      </div>
      
      <div className="waypoint-editor-load-container">
        <button 
          onClick={() => fileInputRef.current?.click()} 
          className="waypoint-editor-load-button"
          disabled={isLoading}
        >
          Load TXT File
        </button>
        <input
          ref={fileInputRef}
          type="file"
          accept=".txt"
          onChange={handleLoadFile}
          style={{ display: 'none' }}
          className="waypoint-editor-file-input"
        />
      </div>
      
      <div className="waypoint-editor-form-container">
        <input 
          type="number" 
          placeholder="Latitude" 
          value={lat} 
          onChange={e => setLat(e.target.value)} 
          className="waypoint-editor-input"
        />
        <input 
          type="number" 
          placeholder="Longitude" 
          value={lon} 
          onChange={e => setLon(e.target.value)} 
          className="waypoint-editor-input"
        />
        <button onClick={handleAddWaypoint} className="waypoint-editor-add-button">Add</button>
      </div>
      
      <div className="waypoint-editor-button-group">
        <button 
          onClick={handleSaveToBackend} 
          disabled={isLoading}
          className="waypoint-editor-file-button"
        >
          {isLoading ? 'Saving...' : 'Save'}
        </button>
      </div>
      
      <div className="waypoint-editor-control-group">
        <button onClick={handleApply} className="waypoint-editor-apply-button">Apply to Map</button>
      </div>
    </div>
  );
};

export default WaypointEditor;
