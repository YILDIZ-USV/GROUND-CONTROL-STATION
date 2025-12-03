import { createSlice, PayloadAction } from '@reduxjs/toolkit';

interface EditorState {
  isOpen: boolean;
  isDirty: boolean;
  selectedWaypointId: string | number | null;
  mode: 'view' | 'edit' | 'create';
}

const initialState: EditorState = {
  isOpen: false,
  isDirty: false,
  selectedWaypointId: null,
  mode: 'view',
};

const editorSlice = createSlice({
  name: 'editor',
  initialState,
  reducers: {
    openEditor: (state) => {
      state.isOpen = true;
      state.isDirty = false;
    },
    closeEditor: (state) => {
      state.isOpen = false;
      state.isDirty = false;
      state.selectedWaypointId = null;
      state.mode = 'view';
    },
    setEditorMode: (state, action: PayloadAction<'view' | 'edit' | 'create'>) => {
      state.mode = action.payload;
    },
    setSelectedWaypoint: (state, action: PayloadAction<string | number | null>) => {
      state.selectedWaypointId = action.payload;
    },
    setEditorDirty: (state, action: PayloadAction<boolean>) => {
      state.isDirty = action.payload;
    },
    resetEditor: (state) => {
      state.isDirty = false;
      state.selectedWaypointId = null;
      state.mode = 'view';
    },
  },
});

export const {
  openEditor,
  closeEditor,
  setEditorMode,
  setSelectedWaypoint,
  setEditorDirty,
  resetEditor,
} = editorSlice.actions;

export default editorSlice.reducer;
