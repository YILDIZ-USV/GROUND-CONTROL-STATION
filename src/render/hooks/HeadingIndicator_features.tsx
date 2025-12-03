import { createSlice } from '@reduxjs/toolkit'

export const IndicatorData = createSlice({
    name: 'indicators',
    initialState: {
        roll: 0,
        pitch: 0,
        heading: 0,
        vario: 0,
        turn: 0,
        airSpeed: 0,
        altitude: 0,
        pressure: 0
    },
    reducers: {

        incrementByAmount: (state, action) => {
            state.heading = action.payload.heading;
            state.pitch = action.payload.pitch;
            state.roll = action.payload.roll;
            state.airSpeed = action.payload.airSpeed;
            state.vario = action.payload.vario;
            state.turn = action.payload.turn;
            state.altitude = action.payload.altitude;
            state.pressure = action.payload.pressure;
        },
    },
})

export const {

    incrementByAmount } = IndicatorData.actions

export default IndicatorData.reducer
