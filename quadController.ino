
#include <RCArduinoFastLib.h>

 // MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
//

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>
#include <Wire.h>

// Assign your channel in pins
#define THROTTLE_IN_PIN 3
#define YAW_IN_PIN 4
#define PITCH_IN_PIN 5
#define ROLL_IN_PIN 2

// Assign your channel out pins
#define THROTTLE_OUT_PIN 8
#define YAW_OUT_PIN 9
#define PITCH_OUT_PIN 10
#define ROLL_OUT_PIN 11

// Assign servo indexes
#define SERVO_THROTTLE 4
#define SERVO_YAW 1
#define SERVO_PITCH 2
#define SERVO_ROLL 3

#define SERVO_FRAME_SPACE 4                                                            

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define YAW_FLAG 2
#define PITCH_FLAG 4
#define ROLL_FLAG 8

#define ADR_IMU 104

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unYawInShared;
volatile uint16_t unPitchInShared;
volatile uint16_t unRollInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint16_t unThrottleInStart;
uint16_t unYawInStart;
uint16_t unPitchInStart;
uint16_t unRollInStart;

uint16_t unLastPitchIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;

int xAcc = 0;
int yAcc = 0;
int zAcc = 0;

int xGyroRaw = 0;
int yGyroRaw = 0;
int zGyroRaw = 0;

long xGyro = 0;
long yGyro = 0;
long zGyro = 0;

float xGyro0 = 0;
float yGyro0 = 0;
float zGyro0 = 0;

float xGyroF = 0;
float yGyroF = 0;
float zGyroF = 0;

float xGyroInt = 0;

const int gyroCalibLength = 500;
int calibCount = 0;

unsigned long lastMicros = 0;
unsigned int dt = 0;

float rollAngAcc = 0;
float pitchAngAcc = 0;

float roll_F = 0;
float pitch_F = 0;

const float ACC_WEIGHT = 0.1;
const float GYRO_WEIGHT = 0.9;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  CRCArduinoFastServos::attach(SERVO_THROTTLE,THROTTLE_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_YAW,YAW_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_PITCH,PITCH_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_ROLL,ROLL_OUT_PIN);

  
 
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,6*2000);

  CRCArduinoFastServos::begin();
 
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);
  getI2CData();
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unYawIn;
  static uint16_t unPitchIn;
  static uint16_t unRollIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
  
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
  
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
  
    if(bUpdateFlags & YAW_FLAG)
    {
      unYawIn = unYawInShared;
    }
  
    if(bUpdateFlags & PITCH_FLAG)
    {
      unPitchIn = unPitchInShared;
    }
    if(bUpdateFlags & ROLL_FLAG)
    {
      unRollIn = unRollInShared;
    }
   
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
  
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unPitchIn, unThrottleIn and unYawIn, the shared
  // variables unPitchInShared, unThrottleInShared, unYawInShared are always owned by
  // the interrupt routines and should not be used in loop

  // we are checking to see if the channel value has changed, this is indicated
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  
  
  
  
  //Start Olle-code
  
  if(calibCount < gyroCalibLength)
  {
    calibGyro();
    calibCount++; 
    Serial.println(calibCount);
    xGyroInt = 0;
  }

  IntegrateGyro();
  getI2CData();
  calcAngles();
//  Serial.print(dt);
//  Serial.print("\tAp: ");
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


  Serial.print(RC_CHANNEL_OUT_COUNT);
  Serial.print("\t");
  Serial.print(unYawIn);
  Serial.print("\t");
  Serial.print(unPitchIn);
  Serial.print("\t");
  Serial.println(unRollIn);


  CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,unThrottleIn);
  CRCArduinoFastServos::writeMicroseconds(SERVO_YAW,unThrottleIn);
  CRCArduinoFastServos::writeMicroseconds(SERVO_PITCH,unThrottleIn);
  CRCArduinoFastServos::writeMicroseconds(SERVO_ROLL,unThrottleIn);
  
  
//
//  if(bUpdateFlags & THROTTLE_FLAG)
//  {
//    CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,unThrottleIn);
//    //Serial.print("Throttle: ");
//    //Serial.print(unThrottleIn);
//  }
//
//  if(bUpdateFlags & YAW_FLAG)
//  {
//    CRCArduinoFastServos::writeMicroseconds(SERVO_YAW,unYawIn);
//    //Serial.print("  Yaw: ");
//    //Serial.print(unYawIn);
//  }
//
//  if(bUpdateFlags & PITCH_FLAG)
//  {
//   CRCArduinoFastServos::writeMicroseconds(SERVO_PITCH,unPitchIn);
//   //Serial.print("  Pitch: ");
//   //Serial.print(unPitchIn);
//  }
//  if(bUpdateFlags & ROLL_FLAG)
//  {
//   CRCArduinoFastServos::writeMicroseconds(SERVO_ROLL,unRollIn);
//   //Serial.print("  Roll: ");
//   //Serial.println(unRollIn);
//  }
  

  bUpdateFlags = 0;
  
}


// simple interrupt service routine
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcYaw()
{
  if(PCintPort::pinState)
  {
    unYawInStart = TCNT1;
  }
  else
  {
    unYawInShared = (TCNT1 - unYawInStart)>>1;
    bUpdateFlagsShared |= YAW_FLAG;
  }
}

void calcPitch()
{
  if(PCintPort::pinState)
  {
    unPitchInStart = TCNT1;
  }
  else
  {
    unPitchInShared = (TCNT1 - unPitchInStart)>>1;
    bUpdateFlagsShared |= PITCH_FLAG;  }
}
void calcRoll()
{
  if(PCintPort::pinState)
  {
    unRollInStart = TCNT1;
  }
  else
  {
    unRollInShared = (TCNT1 - unRollInStart)>>1;
    bUpdateFlagsShared |= ROLL_FLAG;  }
}

void getI2CData()
{
  Wire.beginTransmission(ADR_IMU);
  Wire.write(byte(59));
  Wire.endTransmission();
  
  Wire.requestFrom(ADR_IMU,14);
  
  if (12 <= Wire.available())   // if two bytes were received
  {
    xAcc = Wire.read();  // receive high byte (overwrites previous reading)
    xAcc = xAcc << 8;    // shift high byte to be high 8 bits
    xAcc |= Wire.read(); // receive low byte as lower 8 bits
    
    yAcc = Wire.read();  // receive high byte (overwrites previous reading)
    yAcc = yAcc << 8;    // shift high byte to be high 8 bits
    yAcc |= Wire.read(); // receive low byte as lower 8 bits
    
    zAcc = Wire.read();  // receive high byte (overwrites previous reading)
    zAcc = zAcc << 8;    // shift high byte to be high 8 bits
    zAcc |= Wire.read(); // receive low byte as lower 8 bits

    Wire.read(); //Dump temperature data, DOH!
    Wire.read();    
    
    xGyroRaw = Wire.read();  // receive high byte (overwrites previous reading)
    xGyroRaw = xGyroRaw << 8;    // shift high byte to be high 8 bits
    xGyroRaw |= Wire.read(); // receive low byte as lower 8 bits
    xGyro = xGyroRaw - xGyro0;
    
    yGyroRaw = Wire.read();  // receive high byte (overwrites previous reading)
    yGyroRaw = yGyroRaw << 8;    // shift high byte to be high 8 bits
    yGyroRaw |= Wire.read(); // receive low byte as lower 8 bits
    yGyro = yGyroRaw - yGyro0;
    
    zGyroRaw = Wire.read();  // receive high byte (overwrites previous reading)
    zGyroRaw = zGyroRaw << 8;    // shift high byte to be high 8 bits
    zGyroRaw |= Wire.read(); // receive low byte as lower 8 bits
    zGyro = zGyroRaw - zGyro0;
  }
}

void calcAngles()
{
  rollAngAcc = atan(xAcc/(float)zAcc)*180/PI;
  pitchAngAcc = atan(yAcc/(float)zAcc)*180/PI;
  
  xGyroF = xGyro * (250 / 32767.0);
  yGyroF = yGyro * (250 / 32767.0);
  zGyroF = zGyro * (250 / 32767.0);
  
  complementary();  
}



void complementary()
{
  dt = micros() - lastMicros;
  roll_F = ACC_WEIGHT * rollAngAcc + GYRO_WEIGHT * (roll_F + yGyroF * dt/1000000.0); 
  pitch_F = ACC_WEIGHT * pitchAngAcc + GYRO_WEIGHT * (pitch_F + xGyroF * dt/1000000.0);
  lastMicros += dt; 
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

