#include <PinChangeInt.h>
#include <Wire.h>
#include <Servo.h>

// Assign your channel in pins
#define THROTTLE_IN_PIN 3
#define YAW_IN_PIN 4
#define PITCH_IN_PIN 5
#define ROLL_IN_PIN 2

#define ARM_PIN 12

// Assign your channel out pins
#define SERVO_1 6
#define SERVO_2 9
#define SERVO_3 10
#define SERVO_4 11

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

const int LPLength = 64;
int LPIndex = 0;

int xAcc_LP[LPLength];
int yAcc_LP[LPLength];
int zAcc_LP[LPLength];

int xGyroRaw = 0;
int yGyroRaw = 0;
int zGyroRaw = 0;

int xGyro = 0;
int yGyro = 0;
int zGyro = 0;

int xGyro_LP[LPLength];
int yGyro_LP[LPLength];
int zGyro_LP[LPLength];

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
unsigned long dt = 0;
unsigned long comp_lastMicros = 0;

float rollAngAcc = 0;
float pitchAngAcc = 0;

float roll_F = 0;
float pitch_F = 0;



float last_roll_F = 0;
float last_pitch_F = 0;

const float rollOffset = 3.42;
const float pitchOffset = -1.52;

float rollAngTarget = 0;
float pitchAngTarget = 0;
float yawRateTarget = 0;
float throttleValue = 0;

int rollSignal = 0;
int pitchSignal = 0;
int yawSignal = 0;
int throttleSignal = 0;

const float ACC_WEIGHT = 0.01;
const float GYRO_WEIGHT = 0.99;

bool armed = false;

int motorSignals[] = {0,0,0,0};

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  motor1.attach(SERVO_1);
  motor2.attach(SERVO_2);
  motor3.attach(SERVO_3);
  motor4.attach(SERVO_4);

  pinMode(ARM_PIN, OUTPUT);

  dataScopeInit();

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);
  
  getI2CData();
}

uint16_t unThrottleIn;
uint16_t unYawIn;
uint16_t unPitchIn;
uint16_t unRollIn;

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.

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
  static long lastI2CPoll = 0;
  const int I2CPollInterval = 1000; //500Hz


  if(micros() - lastI2CPoll > I2CPollInterval)
  {
    getI2CData();
    lastI2CPoll = micros();

    last_roll_F = roll_F;
    last_pitch_F = pitch_F;
    calcAngles();
  }
  controlSignalFilter();

  
  if(calibCount < gyroCalibLength)
  {
    calibGyro();
    calibCount++; 
    //Serial.println(calibCount);
    xGyroInt = 0;
  }

  //IntegrateGyro();
  
  
  
  armingStateM();
  controlSignalCond();
  PID();
  calcMotorOutputs();
  

  if(armed)
  {
    digitalWrite(ARM_PIN,HIGH);
    setMotorOutputs();
  }
  else
  {
    digitalWrite(ARM_PIN,LOW);
    stopMotors();
  }

  if(micros() % 512 == 0)
  {
    serialPrint();
  }


  bUpdateFlags = 0;
  
}





