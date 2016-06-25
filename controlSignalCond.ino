void controlSignalCond()
{
  const int maxRollAng = 30;
  const int rollInpZero = 1482;
  rollAngTarget = (((int)unRollIn - rollInpZero) * maxRollAng ) / 512; 
  //division by 512 instead of 500 for speed 
  //This will quantize inputs to rough single degree intervals,
  //  but that should be alright.
  
  const int maxPitchAng = 30;
  const int pitchInpZero = 1480;
  pitchAngTarget = (((int)unPitchIn - pitchInpZero) * maxPitchAng) / 512;

  const int maxYawRate = 5;
  const int yawInpZero = 1500;
  yawRateTarget = (((int)unYawIn - yawInpZero) * maxYawRate / 512);
  //Quantization might be an issue here, but we'll see

  const int maxThrottle = 1; //Use throttle values between 0 and 500
  const int throttleInpZero = 1000;
  throttleSignal = ((int)unThrottleIn - throttleInpZero) / 2;
}

void controlSignalFilter()
{
  static int oldThrottle = 0;
  static int oldPitch = 0;
  static int oldRoll = 0;
  static int oldYaw = 0;

  if(abs((int)unThrottleIn - oldThrottle) > 1600)
  {
    unThrottleIn = oldThrottle;
  }

  if(abs((int)unPitchIn - oldPitch) > 1600)
  {
    unPitchIn = oldPitch;
  }

    if(abs((int)unRollIn - oldRoll) > 1600)
  {
    unRollIn = oldRoll;
  }

    if(abs((int)unYawIn - oldYaw) > 1600)
  {
    unYawIn = oldYaw;
  }

  oldThrottle = unThrottleIn;
  oldPitch = unPitchIn;
  oldRoll = unRollIn;
  oldYaw = unYawIn;

  
}

