import { configureStore } from '@reduxjs/toolkit';
import IndicatorData from '../hooks/HeadingIndicator_features';
import waypointsReducer from './store/waypointsSlice';
import editorReducer from './store/editorSlice';

const store = configureStore({
  reducer: {
    indicators: IndicatorData,
    waypoints: waypointsReducer,
    editor: editorReducer,
  },
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;
export default store;
