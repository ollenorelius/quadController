void getI2CData()
{
  Wire.beginTransmission(ADR_IMU);
  Wire.write(byte(59));
  Wire.endTransmission();
  
  Wire.requestFrom(ADR_IMU,14);
  
  if (12 <= Wire.available())   // if two bytes were received
  {
    LPIndex = ((LPIndex + 1) % LPLength);
    xAcc_LP[LPIndex] = Wire.read();  // receive high byte (overwrites previous reading)
    xAcc_LP[LPIndex] = xAcc_LP[LPIndex] << 8;    // shift high byte to be high 8 bits
    xAcc_LP[LPIndex] |= Wire.read(); // receive low byte as lower 8 bits
    
    yAcc_LP[LPIndex] = Wire.read();  // receive high byte (overwrites previous reading)
    yAcc_LP[LPIndex] = yAcc_LP[LPIndex] << 8;    // shift high byte to be high 8 bits
    yAcc_LP[LPIndex] |= Wire.read(); // receive low byte as lower 8 bits
    
    zAcc_LP[LPIndex] = Wire.read();  // receive high byte (overwrites previous reading)
    zAcc_LP[LPIndex] = zAcc_LP[LPIndex] << 8;    // shift high byte to be high 8 bits
    zAcc_LP[LPIndex] |= Wire.read(); // receive low byte as lower 8 bits

    xAcc = arrayMean(xAcc_LP,LPLength);
    yAcc = arrayMean(yAcc_LP,LPLength);
    zAcc = arrayMean(zAcc_LP,LPLength);

    Wire.read(); //Dump temperature data, DOH!
    Wire.read();    
    
    xGyroRaw = Wire.read();  // receive high byte (overwrites previous reading)
    xGyroRaw = xGyroRaw << 8;    // shift high byte to be high 8 bits
    xGyroRaw |= Wire.read(); // receive low byte as lower 8 bits
    xGyro_LP[LPIndex] = xGyroRaw - xGyro0;
    
    yGyroRaw = Wire.read();  // receive high byte (overwrites previous reading)
    yGyroRaw = yGyroRaw << 8;    // shift high byte to be high 8 bits
    yGyroRaw |= Wire.read(); // receive low byte as lower 8 bits
    yGyro_LP[LPIndex] = yGyroRaw - yGyro0;
    
    zGyroRaw = Wire.read();  // receive high byte (overwrites previous reading)
    zGyroRaw = zGyroRaw << 8;    // shift high byte to be high 8 bits
    zGyroRaw |= Wire.read(); // receive low byte as lower 8 bits
    zGyro_LP[LPIndex] = zGyroRaw - zGyro0;
    
    xGyro = arrayMean(xGyro_LP,LPLength);
    yGyro = arrayMean(yGyro_LP,LPLength);
    zGyro = arrayMean(zGyro_LP,LPLength);
    
  }
}

void calcAngles()
{
  rollAngAcc = atan(xAcc/(float)zAcc)*180/PI + rollOffset;
  pitchAngAcc = atan(yAcc/(float)zAcc)*180/PI + pitchOffset;
  
  xGyroF = xGyro * (250 / 32767.0);
  yGyroF = yGyro * (250 / 32767.0);
  zGyroF = zGyro * (250 / 32767.0);
  
  complementary();  
}



void complementary()
{
  dt = micros() - comp_lastMicros;
  roll_F = ACC_WEIGHT * rollAngAcc + GYRO_WEIGHT * (roll_F - yGyroF * 0.001); 
  pitch_F = ACC_WEIGHT * pitchAngAcc + GYRO_WEIGHT * (pitch_F + xGyroF * 0.001);
  comp_lastMicros = micros(); 
}

void calibGyro()
{
  xGyro0 += xGyroRaw / (float)gyroCalibLength;
  yGyro0 += yGyroRaw / (float)gyroCalibLength;
  zGyro0 += zGyroRaw / (float)gyroCalibLength;
}

void IntegrateGyro()
{
  xGyroInt = xGyroInt + xGyroF *dt/1000000.0;
}
void SetGyroCalibration()
{
  byte xGyroHB = xGyro0/255;
  byte xGyroLB = (int)xGyro0 % 0xFF00;
  Serial.print(xGyroLB);
  Serial.print("\t");
  Serial.println(xGyroHB);
  //Wire.beginTransmission(ADR_IMU);
  //Wire.write(byte(19));
  //Wire.write(xGyroHB);
  //Wire.write(xGyroLB);
  //Wire.endTransmission();
}

int arrayMean(int inpArray[], int len)
{
  long arSum = 0;
  for(int i = 0; i < len; i++)
  {
    arSum += inpArray[i];
  }
  return (arSum / len);
}

