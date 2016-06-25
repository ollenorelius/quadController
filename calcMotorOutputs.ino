void calcMotorOutputs()
{
  if(unThrottleIn > 1100)
  {
    motorSignals[0] = -pitchSignal + rollSignal + yawSignal + 2*throttleSignal;
    motorSignals[1] = -pitchSignal - rollSignal - yawSignal + 2*throttleSignal;
    motorSignals[2] = pitchSignal + rollSignal - yawSignal + 2*throttleSignal;
    motorSignals[3] = pitchSignal - rollSignal + yawSignal + 2*throttleSignal;
  }
  else
  {
    motorSignals[0] = 0;
    motorSignals[1] = 0;
    motorSignals[2] = 0;
    motorSignals[3] = 0;
  }
}

