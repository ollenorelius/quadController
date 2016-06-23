void serialPrint()
{
  Serial.print("D ");
  Serial.print(micros());
  Serial.print(" ");
  Serial.print(rollAngAcc);
  Serial.print(" ");
  Serial.print(pitchAngAcc);
  Serial.print(" ");
  Serial.print(roll_F);
  Serial.print(" ");
  Serial.print(pitch_F);
  Serial.print(" ");
 // Serial.print(zAcc);
//  Serial.print(xGyro0);
//  Serial.print("\t");
//  Serial.print(xGyroF);
//  Serial.print("\t");
//  Serial.print(xGyroInt);
//  Serial.print("\tGp: ");
//  Serial.print("x: ");
//  Serial.print(roll_F);
//  Serial.print("\t");
//  Serial.println(pitch_F);
//  Serial.print("\tz: ");
//  Serial.println(abs(zGyroF));
//  Serial.print("\tFp: ");
//  Serial.println(pitch_F);

//
//  Serial.print(RC_CHANNEL_OUT_COUNT);
//  Serial.print("\t");
//  Serial.print(unYawIn);
//  Serial.print("\t");
//  Serial.print(unPitchIn);
//  Serial.print("\t");
//  Serial.println(unRollIn);

  Serial.println();
}

void dataScopeInit()
{
    //!, number of channels, name1, name2, ...
  Serial.println("N roll pitch roll_Filtered pitch_Filtered");
  // y limits
  Serial.println("L -10 10");
  Serial.println("!");
}

