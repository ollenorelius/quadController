void setMotorOutputs()
{
  motor1.writeMicroseconds(1000+ motorSignals[0]);
  motor2.writeMicroseconds(1000+motorSignals[1]);
  motor3.writeMicroseconds(1000+motorSignals[2]);
  motor4.writeMicroseconds(1000+motorSignals[3]);
}

void stopMotors()
{
  motor1.writeMicroseconds(0);
  motor2.writeMicroseconds(0);
  motor3.writeMicroseconds(0);
  motor4.writeMicroseconds(0);
}

