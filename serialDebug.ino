void serialPrint()
{
  Serial.print("D ");
  Serial.print(micros());
  Serial.print(" ");

  
  Serial.print(rollSignal);
  Serial.print(" ");
  Serial.print(pitchSignal);
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
//  Serial.print(unThrottleIn);
//  Serial.print(" ");
//  Serial.print(unYawIn);
//  Serial.print(" ");
//  Serial.print(unPitchIn);
//  Serial.print(" ");
//  Serial.print(unRollIn);

//  Serial.print(motorSignals[0]);
//  Serial.print(" ");
//  Serial.print(motorSignals[1]);
//  Serial.print(" ");
//  Serial.print(motorSignals[2]);
//  Serial.print(" ");
//  Serial.print(motorSignals[3]);
//  Serial.print(" ");


  Serial.println();

}

void dataScopeInit()
{
    //!, number of channels, name1, name2, ...
  Serial.println("N roll pitch roll_Filtered pitch_Filtered");
  // y limits
  Serial.println("L -500 500");
  Serial.println("!");
}

