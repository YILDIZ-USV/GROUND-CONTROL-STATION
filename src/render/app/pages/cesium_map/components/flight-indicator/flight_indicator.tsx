import type { RootState } from '../../../../store'
import { useSelector } from 'react-redux'
import {
  AttitudeIndicator,
  HeadingIndicator,
  TurnCoordinator,
} from 'react-typescript-flight-indicators'
import './styles.css'

export default function FlightIndicator() {
  const { heading, pitch, roll} = useSelector((state: RootState) => state.indicators)
  
// Convert radians to degrees (flight indicators expect degrees)
  const headingDeg = (heading * 180 / Math.PI) % 360
  const pitchDeg = pitch * 180 / Math.PI
  const rollDeg = roll * 180 / Math.PI
  
  
  return (
    <div className="indicators">
      <TurnCoordinator size="12em" showBox={false} turn={rollDeg} />
      <HeadingIndicator size="12em" showBox={false} heading={headingDeg} />
      <AttitudeIndicator size="12em" showBox={false} pitch={pitchDeg} roll={rollDeg} />
    </div>
  )
}
