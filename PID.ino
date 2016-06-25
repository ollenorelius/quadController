const float P = 10;
const float I = 0;
const float D = 2;

void PID()
{
  dt = micros() - lastMicros;
  lastMicros = micros(); 
  
  static float rollError = 0;
  static float pitchError = 0;

  float dRoll = 0;
  float dPitch = 0;

  static float iRoll = 0;
  static float iPitch = 0;

//  rollAngTarget = 0;
//  pitchAngTarget = 0;
//  yawRateTarget = 0;
//  throttleValue = 0;

  float oldRollError = rollError;
  float oldPitchError = pitchError;
  
  rollError = rollAngTarget - roll_F;
  pitchError = pitchAngTarget - pitch_F;  

  dRoll = -(roll_F - last_roll_F) / 0.001;
  dPitch = -(pitch_F - last_pitch_F) / 0.001;

  //iRoll += rollError * dt / 1000000;
  //iPitch += pitchError * dt / 1000000;
  //Serial.println(dRoll);
  rollSignal = rollError * P + dRoll * D + iRoll * I;
  pitchSignal = pitchError * P + dPitch * D + iPitch * I;
  
  yawSignal = yawRateTarget; //Don't know what to do with this
}

