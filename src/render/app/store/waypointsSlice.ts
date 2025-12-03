import { createSlice, createAsyncThunk, PayloadAction } from '@reduxjs/toolkit';
import { Waypoint } from '../pages/cesium_map/types/waypoint.types';
import PlaneService from '../pages/cesium_map/service/plane_service';

// Async thunk for saving waypoints to backend
export const saveWaypointsThunk = createAsyncThunk(
  'waypoints/saveWaypoints',
  async (waypoints: Waypoint[], { rejectWithValue }) => {
    try {
      const service = new PlaneService();
      const result = await service.saveWaypoints(waypoints);
      
      if (result && result.status === 'success') {
        return { message: 'Waypoints successfully saved' };
      } else {
        return rejectWithValue('Backend save failed');
      }
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error occurred';
      return rejectWithValue(errorMessage);
    }
  }
);

interface WaypointsState {
  // Editor state - waypoints being edited but not applied to map
  editorWaypoints: Waypoint[];
  // Applied state - waypoints currently shown on map
  appliedWaypoints: Waypoint[];
  isLoading: boolean;
  error: string | null;
  saveStatus: 'idle' | 'loading' | 'success' | 'error';
}

const initialState: WaypointsState = {
  editorWaypoints: [],
  appliedWaypoints: [],
  isLoading: false,
  error: null,
  saveStatus: 'idle',
};

const waypointsSlice = createSlice({
  name: 'waypoints',
  initialState,
  reducers: {
    // Editor actions - for waypoints being edited
    setEditorWaypoints: (state, action: PayloadAction<Waypoint[]>) => {
      state.editorWaypoints = action.payload;
      state.error = null;
      state.saveStatus = 'idle';
    },
    addEditorWaypoint: (state, action: PayloadAction<Waypoint>) => {
      state.editorWaypoints.push(action.payload);
      state.error = null;
      state.saveStatus = 'idle';
    },
    removeEditorWaypoint: (state, action: PayloadAction<string | number>) => {
      state.editorWaypoints = state.editorWaypoints.filter(wp => wp.id !== action.payload);
      state.error = null;
      state.saveStatus = 'idle';
    },
    clearEditorWaypoints: (state) => {
      state.editorWaypoints = [];
      state.error = null;
      state.saveStatus = 'idle';
    },
    
    // Applied actions - for waypoints shown on map
    setAppliedWaypoints: (state, action: PayloadAction<Waypoint[]>) => {
      state.appliedWaypoints = action.payload;
      state.error = null;
      state.saveStatus = 'idle';
    },
    applyEditorWaypoints: (state) => {
      // Move editor waypoints to applied waypoints
      state.appliedWaypoints = [...state.editorWaypoints];
      state.error = null;
      state.saveStatus = 'idle';
    },
    clearAppliedWaypoints: (state) => {
      state.appliedWaypoints = [];
      state.error = null;
      state.saveStatus = 'idle';
    },
    
    // Utility actions
    setWaypointsLoading: (state, action: PayloadAction<boolean>) => {
      state.isLoading = action.payload;
      if (action.payload) {
        state.error = null;
        state.saveStatus = 'loading';
      }
    },
    setWaypointsError: (state, action: PayloadAction<string>) => {
      state.error = action.payload;
      state.isLoading = false;
      state.saveStatus = 'error';
    },
  },
  extraReducers: (builder) => {
    builder
      .addCase(saveWaypointsThunk.pending, (state) => {
        state.isLoading = true;
        state.error = null;
        state.saveStatus = 'loading';
      })
      .addCase(saveWaypointsThunk.fulfilled, (state) => {
        state.isLoading = false;
        state.error = null;
        state.saveStatus = 'success';
      })
      .addCase(saveWaypointsThunk.rejected, (state, action) => {
        state.isLoading = false;
        state.error = action.payload as string;
        state.saveStatus = 'error';
      });
  },
});

export const {
  // Editor actions
  setEditorWaypoints,
  addEditorWaypoint,
  removeEditorWaypoint,
  clearEditorWaypoints,
  // Applied actions
  setAppliedWaypoints,
  applyEditorWaypoints,
  clearAppliedWaypoints,
  // Utility actions
  setWaypointsLoading,
  setWaypointsError,
} = waypointsSlice.actions;

export default waypointsSlice.reducer;
