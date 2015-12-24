/*

  ARDUINO based hexbug Scarab v 1.8.0
  
  Arduino based hexbug spider Scarab is a robotic mechanical toy.
  
  Scarab is equipped with:
  1. Ultrasonic distance sensor
  2. IR obstacle sensor
  3. 3 axis Gyro
  4. 3 axis Accelerometer
  5. Laser pointer
  
  Scarab can:
  1. Move forward and maintain direction (which is set when turned on)
  2. Detect obstacles
  3. Attempt to go around obstacles, ultimately in the direction set at #1
  
  When scarab detects an obstacle, it performs a "dance".
  A dance is a number of movements and actions scarab performs to help him decide how to go around the obstacle. 
  Some of the dance moves are just for fun and showoff.
  Currently scarab performs 3 separate dances, which he chooses randomly in front of every obstacle:
  1. Randomly turn right or left by 90 degrees and walk in that direction for 5 seconds. Then attempt to move towards original direction
  2. Stop and measure distance to the obstacle a little to the right and a little to the left. Try go around the obstacle in the direction of further measurement.
  3. Back off a little, then execute dance #1 (random) - this is just a little more than #1 dance. 
  4. Back off a litle, then #2 but measured at 90 degrees to the original direction, preferring one direction. 

  When scarab is turned on, it needs to calibrate the gyro. It has to be absolutely still and on a horizontal surface. 
  When scarab is calibrating gyro, a yellow light is blinking. If scarab is moved or shaken durinnng calibration, a red light starts blinking, 
  and the calibration process restarts. 
  If scarab is turned upside down, it stops and re-starts the calibration process. This also resets the direction scarab is trying to maintain. 
  If Scarab cannot move, either because of some obstacle or low batteries, all three lights will blink with a slight relative delay (running lights). 

Change log:
  1. 2015-03-03 - added average filters for accelerometer values
  2. 2015-03-25 - added another dance and a "directional preoclivity" to make sure scarab goes in one direction. 
  3. 2015-05-05 - v1.5.0 switched to DiretIO library
  4. 2015-05-19 - v1.6.0 compiled against latest TaskScheduler library with IDLE sleep support. Laser indicated ultrasonic ping activity
  5. 2015-05-22 - v1.7.0 use of micros for gyro with possibly more accurate tracking
  6. 2015-12-04 - v1.7.1 updated gyro calibration routine
  7. 2015-12-23 - v1.8.0 gyro and accel tasks are running with elevated priority (TaskScheduler 2.0.0)
*/

#include <DirectIO.h>
#include <AvgFilter.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_TIMECRITICAL
#define _TASK_PRIORITY
#include <TaskScheduler.h>

#include <avr/sleep.h>
#include <avr/power.h>

#include <Statistic.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>

/*
 * HMC5883L Connection.
 * Arduino UNO or compatible
 *  Arduino GND -> GY271/HMC5883L GND
 *  Arduino 3.3V -> GY271/HMC5883L VCC
 *  Arduino A4 (SDA) -> GY271/HMC5883L SDA
 *  Arduino A5 (SCL) -> GY271/HMC5883L SCL
 */

//#define _DEBUG_

#define TRIGGERPIN 11
#define ECHOPIN 10
#define IRPIN 12
#define LASERPIN A3

#define GREENLEDPIN 7
#define YELLOWLEDPIN 4
#define REDLEDPIN 8

#define M1PIN1 6
#define M1PIN2 9
#define M2PIN1 5
#define M2PIN2 3

// PINs for DirectIO

Output<TRIGGERPIN>    pTrigger;
Input<ECHOPIN>        pEcho; 
Input<IRPIN>          pIrPin;
Output<GREENLEDPIN>   pGreenLedPin;
Output<YELLOWLEDPIN>  pYellowLedPin;
Output<REDLEDPIN>     pRedLedPin;
Output<LASERPIN>      pLaserPin;

AnalogOutput<M1PIN1>  pM1Pin1;
AnalogOutput<M1PIN2>  pM1Pin2;
AnalogOutput<M2PIN1>  pM2Pin1;
AnalogOutput<M2PIN2>  pM2Pin2;

Output<13>            pLED;
  
//const char CToken[10] = "SCARAB14"; // Eeprom token: Spider

int state;
boolean error;

#define PING_OBSTACLE	  1000 //  ~20 cm; distance = delay/58.2
#define PING_PERIOD	  200  // 5 times per second
#define GYRO_PERIOD       10  // potentially 1 kHz if the board can handle it
#define ACCEL_PERIOD      100  // 5 times per second
#define MOVE_PERIOD       50 // 20 times per second
#define MOVECHK_PERIOD    500 // 2 times per second
#define SLOW_BLINK        500  // half second
#define FAST_BLINK        250  // 4 times per second
#define SUPERFAST_BLINK   100  // 10 times per second
#define LASER_STROBE      50  // was 100
#define LASER_STROBES     10

#define CALIBRATE_CYCLES  (5000/GYRO_PERIOD)
#define AVG_PRIMRE_CYCLES 100
#define MOVE_MAXSPEED     255 //255
//#define MOVE_MINSPEED     140 //180
//#define MOVE_HALFSPEED    195  // (250-140)/2
#define MOVE_START        100

#define GYRO_MODE        MPU6050_GYRO_FS_250
//#define GYRO_MODE        MPU6050_GYRO_FS_500
//#define GYRO_MODE        MPU6050_GYRO_FS_1000
//#define GYRO_MODE        MPU6050_GYRO_FS_2000

#define  AC_LSB_10        1310
//#define  AC_LSB_10        655
//#define  AC_LSB_10        328
//#define  AC_LSB_10        164

// average R (ax^2+ay^2+az^2) = 310,000,000, std deviation = ~28,000,000
#define G2_R_AVERAGE  287584352L
#define G2_STD_DEV     2579226L
#define G2_2STD_DEV    (G2_STD_DEV+G2_STD_DEV)
#define G2_3STD_DEV    (G2_STD_DEV*3)
#define G2_THRESHOLD_HIGH  G2_R_AVERAGE+G2_2STD_DEV  // g^2 in a very stable state (need to calculate) representing "stable" state (no movements)
#define G2_THRESHOLD_LOW   G2_R_AVERAGE-G2_2STD_DEV  // g^2 in a very stable state (need to calculate) representing "stable" state (no movements)

// Callback methods prototypes 
void mPingCallback();
void gyroCalibrate();
void accelCallback();
void moveCallback();
void moveCheckCallback();
void blinkRedOn(); void blinkYellowOn(); void blinkGreenOn();
void errorCallback();
void moveDanceScientificInit();

// Tasks
// gyroManager is a higher priority scheduler
// taskManager is a base scheduler
Scheduler taskManager, gyroManager;

Task tDistance (PING_PERIOD, TASK_FOREVER, &mPingCallback );
Task tGyro (GYRO_PERIOD, CALIBRATE_CYCLES, &gyroCalibrate );
Task tAccel (ACCEL_PERIOD, TASK_FOREVER, &accelCallback);
Task tMove (MOVE_PERIOD, TASK_FOREVER, &moveCallback );
Task tMoveCheck (MOVECHK_PERIOD, TASK_FOREVER, &moveCheckCallback);
Task tBlinkRed (FAST_BLINK, TASK_FOREVER, &blinkRedOn );
Task tBlinkYellow (SLOW_BLINK, TASK_FOREVER, &blinkYellowOn );
Task tBlinkGreen (SLOW_BLINK, TASK_FOREVER, &blinkGreenOn );
Task tError (TASK_SECOND, TASK_FOREVER, &errorCallback);

#ifdef _DEBUG_
void displayCallback();
Task tDisplay (SECOND, TASK_FOREVER, &displayCallback );
#endif

MPU6050 accelgyro;

//#define MOVE_NUMOFDANCES 3
//void  (*moveDances[MOVE_NUMOFDANCES])() = { &moveDance2MeasuresInit, &moveDanceRandom, &moveDanceSwirlInit };

#define MOVE_NUMOFDANCES 1
void  (*moveDances[MOVE_NUMOFDANCES])() = { &moveDanceScientificInit };


enum Motion_States {
  MOVE_NORMAL,
  MOVE_CHECKSTILL,
  MOVE_ROTATE,
  MOVE_OBSTACLE,
};

Motion_States moveNow = MOVE_NORMAL;

boolean upsideDown = false;
boolean movingShaking = false;
int  shakeSettleSeconds = 2;
int  moveAdjustSeconds = 4;

byte minSpeed, halfSpeed;

//
// CODE
//

// ----------------------- Movements --------------------------------

int rightLegSpeed, leftLegSpeed;

void moveStop() {
  rightLegSpeed = leftLegSpeed =0;
  pM1Pin1 = 0;
  pM1Pin2 = 0;
  pM2Pin1 = 0;
  pM2Pin2 = 0;
}

void moveRightLegsStop() {
  rightLegSpeed = 0;
  pM2Pin1 = 0;
  pM2Pin2 = 0;
}

void moveRightLegsForward(int aSpeed) {
  rightLegSpeed = aSpeed > MOVE_MAXSPEED ? MOVE_MAXSPEED : aSpeed;
  pM2Pin1 = rightLegSpeed;
  pM2Pin2 = 0;
}

void moveRightLegsBack(int aSpeed) {
  rightLegSpeed = aSpeed > MOVE_MAXSPEED ? -MOVE_MAXSPEED : -aSpeed;
  pM2Pin1 = 0;
  pM2Pin2 = -rightLegSpeed;
}

void moveLeftLegsStop() {
  leftLegSpeed = 0;
  pM1Pin1 = 0;
  pM1Pin2 = 0;
}

void moveLeftLegsForward(int aSpeed) {
  leftLegSpeed = aSpeed > MOVE_MAXSPEED ? MOVE_MAXSPEED : aSpeed;
  pM1Pin1 = leftLegSpeed;
  pM1Pin2 = 0;
}

void moveLeftLegsBack(int aSpeed) {
  leftLegSpeed = aSpeed > MOVE_MAXSPEED ? -MOVE_MAXSPEED : -aSpeed;
  pM1Pin1 = 0;
  pM1Pin2 = -leftLegSpeed;
}

void moveForward(int aS1, int aS2) {
  moveLeftLegsForward(aS1);
  moveRightLegsForward(aS2);
}

void moveBack(int aS1, int aS2) {
  moveLeftLegsBack(aS1);
  moveRightLegsBack(aS2);
}

void moveRotateRight(int aS1, int aS2) {
  moveLeftLegsForward(aS1);
  moveRightLegsBack(aS2);
}

void moveRotateLeft(int aS1, int aS2) {
  moveLeftLegsBack(aS2);
  moveRightLegsForward(aS1);
}



//
// ----------------- Distance measurement routines ------------------
//
long pingDistance = 39999;
long mPingFilterd[3];
avgFilter mPingFilter(3, mPingFilterd);
boolean  obstacle;


void mPingCallback() {
  unsigned long d; 
  
  laserOn();

  pTrigger = LOW;
  delayMicroseconds(4);
  pTrigger = HIGH;
  delayMicroseconds(10);
  pTrigger = LOW;
  d = pulseIn(ECHOPIN, HIGH, PING_OBSTACLE<<1);
  
  laserOff();

  pingDistance = mPingFilter.value((d ? d : PING_OBSTACLE<<1));
  obstacle = (pingDistance <= PING_OBSTACLE) || (!pIrPin);
  if (obstacle) tDistance.delay(10);
}


long longRangePing(unsigned long aTimeout) {
  unsigned long d; 
  
  laserOn();

  pTrigger = LOW;
  delayMicroseconds(4);
  pTrigger = HIGH;
  delayMicroseconds(10);
  pTrigger = LOW;
  d = pulseIn(ECHOPIN, HIGH, aTimeout);
  
  laserOff();

  return (d ? d : aTimeout);
}

// ---------------------------------------------------------------


// Calculate rotational angle based on gyro

long gz, gz_bias, gz_zero_zone, prev_gz;
unsigned long curr_t, prev_t;
long  currentAngle;
long  mGyroFilterd[3];
avgFilter mGyroFilter(3, mGyroFilterd);
long currentHeading, ultimateHeading;
unsigned long currentHeadingStart, ultimateHeadingStart;
Statistic stat;

void gyroCalibrate() {

  // Only calibrate if spider is standing still and not upside down.
  if (movingShaking || upsideDown) {
    if (!tBlinkRed.isEnabled()) tBlinkRed.enable();
    tGyro.setIterations(CALIBRATE_CYCLES);
    tGyro.enable();
    return;
  }
  else {
    if (tBlinkRed.isEnabled()) {
      tBlinkRed.disable();
      ledRedOff();
    }
  }

  if (tGyro.isFirstIteration()) {
    //  first run
#ifdef _DEBUG_
    Serial.println("Calibrating gyro");
#endif
    tBlinkYellow.enable();
    accelgyro.initialize();
    if (!accelgyro.testConnection()) {
      errorCondition();
      return;
    }
    
    accelgyro.setFullScaleGyroRange(GYRO_MODE);
    accelgyro.setDMPEnabled(false);
    accelgyro.setStandbyYGyroEnabled(true);
    accelgyro.setStandbyXGyroEnabled(true);
    accelgyro.setTempSensorEnabled(false);
    
    
    curr_t = micros(); 
    gz = 0;
    currentAngle = 0;
    mGyroFilter.initialize();
    stat.clear();
  }

  if (!tGyro.isLastIteration()) {
    stat.add((float) accelgyro.getRotationZ());
#ifdef _DEBUG_
//Serial.print("tGyro iteration = "); Serial.println(tGyro.getIterations());
#endif
  }
  else {
#ifdef _DEBUG_
//    Serial.print("tGyro iteration = "); Serial.println(tGyro.getIterations());
#endif
    gz_bias = (long) stat.average();
    gz_zero_zone = (long) stat.pop_stdev() * 2;  // should we use 3 or 2 std dev?

    tGyro.set(GYRO_PERIOD, AVG_PRIMRE_CYCLES, &gyroCallbackPrime);
    tGyro.enable();
    tDistance.enable();
    curr_t = micros(); //millis();
#ifdef _DEBUG_
    Serial.println("Calibration done:");
    Serial.print("Bias = "); Serial.println(gz_bias);
    Serial.print("Zero = "); Serial.println(gz_zero_zone);
#endif
  }
}


void gyroCallbackPrime() {
  gyroCallback();
  if (tGyro.isLastIteration()) {
    // Last iteration
    tGyro.setCallback(&gyroCallback);
    tGyro.setIterations(TASK_FOREVER);
    ultimateHeading = currentHeading = currentAngle;
    ultimateHeadingStart = currentHeadingStart = millis();    
    minSpeed = MOVE_START;
    tMove.set (MOVECHK_PERIOD, TASK_FOREVER, &moveCalibrate );
    tMove.enable();
    tBlinkGreen.enable();

#ifdef _DEBUG_
    tDisplay.enable();
    Serial.println("CurrentAngle primed");
#endif
  }
}

void gyroCallback() {
#ifdef _TASK_TIMECRITICAL
  checkOverrun();
#endif

  prev_t = curr_t;
  prev_gz = gz;
  gz = mGyroFilter.value(accelgyro.getRotationZ() - gz_bias);
  curr_t = micros(); //millis();
  if (abs(gz) < gz_zero_zone) {
    gz = 0;
  }
#ifdef _DEBUG_
  //  Serial.print("gz="); Serial.print(gz); Serial.print(", prev_gz="); Serial.print(prev_gz); Serial.print(", t="); Serial.print(curr_t); Serial.print(", pt="); Serial.println(prev_t);
  //  Serial.print("gz+prev_gz=");Serial.print(gz+prev_gz); Serial.print(", t-pt="); Serial.print(curr_t-prev_t); Serial.print(", delta="); Serial.println(((long) (gz+prev_gz)*((long) (curr_t-prev_t))));
//  if (tGyro.getOverrun() < 0) ledRedOn();
//  else ledRedOff();
#endif
  long delta_g = (gz + prev_gz) / 2L;  // rates are 250 deg/s, 500 deg/s, 1000 deg/s and 2000 deg/s
  long delta_t = curr_t - prev_t;
  long delta_a = (delta_g * delta_t) / AC_LSB_10 / 1000;  //LSBs are 131, 65.5, 32.8 and 16.4 respectively
  currentAngle += delta_a;
}




int accX, accY, accZ;
long laccX, laccY, laccZ;
long faccXd[5], faccYd[5], faccZd[5];
avgFilter faccX(5, faccXd), faccY(5, faccYd), faccZ(5, faccZd);
int  shakeCounter = 0;

void accelCallback() {
  long g2;

#ifdef _TASK_TIMECRITICAL
  checkOverrun();
#endif

  accelgyro.getAcceleration(&accX, &accY, &accZ);
  laccX = faccX.value(accX);
  laccY = faccY.value(accY);
  laccZ = faccZ.value(accZ);
  upsideDown = false;
  if (laccZ >= 0) upsideDown = true;

  g2 = (laccX * laccX + laccY * laccY + laccZ * laccZ);
  if (g2 < G2_THRESHOLD_LOW || g2 > G2_THRESHOLD_HIGH) {
    // Based on acceleration - we are moving
    movingShaking = true;
    shakeCounter = shakeSettleSeconds * 1000 / ACCEL_PERIOD; // i.e. 2 seconds = 2000/200 = 10 cycles
  }
  else {
    if (shakeCounter == 0) movingShaking = false;
    else shakeCounter--;
  }
#ifdef _DEBUG_
    Serial.println("Accelerometer:");
//    Serial.print("accZ = "); Serial.println(accZ);
    Serial.print("g2 = "); Serial.println(g2);
//    Serial.print("UpsideDown = "); Serial.println(upsideDown);
Serial.print("movingShaking = "); Serial.println(movingShaking);
//    Serial.print("shakeCounter = "); Serial.println(shakeCounter);
#endif
}


void blinkRedOn () {
  ledRedOn();
  tBlinkRed.setCallback(&blinkRedOff);
}

void blinkRedOff () {
  ledRedOff();
  tBlinkRed.setCallback(&blinkRedOn);
}

void blinkYellowOn () {
  ledYellowOn();
  tBlinkYellow.setCallback(&blinkYellowOff);
}

void blinkYellowOff () {
  ledYellowOff();
  tBlinkYellow.setCallback(&blinkYellowOn);
}

void blinkGreenOn () {
  ledGreenOn();
  tBlinkGreen.setCallback(&blinkGreenOff);
}

void blinkGreenOff () {
  ledGreenOff();
  tBlinkGreen.setCallback(&blinkGreenOn);
}


void errorCallback() {

}


#ifdef _DEBUG_
unsigned long counter = 0;
void displayCallback() {
//  if (counter++ % 40 == 0) {
//    Serial.println("obstacle, distance, angle");
//  }
//  Serial.print(obstacle); Serial.print(",");
//  Serial.print(pingDistance); Serial.print(",");
//  Serial.print(currentAngle); Serial.print(",");
//  Serial.println("");
}
#endif

#define  MOVE_MINANGLEMOVEFWD  4500
#define  MOVE_HEADING90DEG     9000
#define  MOVE_HEADING30DEG     3000
#define  MOVE_HEADING45DEG     4500
#define  MOVE_HEADING60DEG     6000
#define  MOVE_HEADING360DEG    36000
#define  MOVE_HEADING180DEG    18000
#define  MOVE_ROTATEANGLETOLERANCE  500
#define  MOVE_ANGLETOSPEEDFACTOR    1

long  moveAdjustCounter = 0;

void moveCallback() {
  long deltaAngle = currentAngle - currentHeading;  // Angle is accumulated in 100's of degrees
  int ls = MOVE_MAXSPEED, rs = MOVE_MAXSPEED;

#ifdef _DEBUG_
//  Serial.print("currentAngle = "); Serial.println(currentAngle);
//  Serial.print("currentHeading = "); Serial.println(currentHeading);
//  Serial.print("ultimateHeading = "); Serial.println(ultimateHeading);
//  Serial.print("deltaAngle = "); Serial.println(deltaAngle);
#endif

#ifdef _TASK_TIMECRITICAL
  checkOverrun();
#endif

  if (moveCheckUpsideDown()) return;
  if (currentHeading != ultimateHeading) ultimateHeadingStart = millis();

  switch (moveNow) {

    case MOVE_NORMAL:
      if (moveCheckObstacle()) break;

      if (abs (deltaAngle) > MOVE_MINANGLEMOVEFWD) {
        moveStop();
        tBlinkGreen.enable();
        shakeSettleSeconds = 1;
        moveNow = MOVE_CHECKSTILL;
        return;
      }
      else {
        if (deltaAngle >= MOVE_ROTATEANGLETOLERANCE) ls = halfSpeed;
        if (deltaAngle <= -MOVE_ROTATEANGLETOLERANCE) rs = halfSpeed;
        if (deltaAngle >   MOVE_MINANGLEMOVEFWD>>1 ) ls = minSpeed;
        if (deltaAngle < -(MOVE_MINANGLEMOVEFWD>>1)) rs = minSpeed;
        moveForward(ls, rs);
        tBlinkGreen.disable();
        ledGreenOn();
        ledRedOff();
      }
      if (currentHeading != ultimateHeading) {
        if (moveAdjustCounter == 0) {
          if (currentHeading > ultimateHeading) addToCurrentHeading(-MOVE_HEADING90DEG);
          else addToCurrentHeading(MOVE_HEADING90DEG);
          moveAdjustCounter = moveAdjustSeconds * 1000 / MOVE_PERIOD;
          moveNow = MOVE_ROTATE;
        }
        else moveAdjustCounter--;
      }
      break;

    case MOVE_CHECKSTILL:
      moveStop();
      if (!movingShaking) moveNow = MOVE_ROTATE;
      break;

    case MOVE_ROTATE:
      if (abs (deltaAngle) <= MOVE_ROTATEANGLETOLERANCE) {
        moveNow = MOVE_NORMAL;
        moveAdjustCounter = moveAdjustSeconds * 1000 / MOVE_PERIOD;
        if (currentHeading == ultimateHeading) ultimateHeadingStart = millis();
        currentHeadingStart = millis();
        return;
      }
      ls = abs(deltaAngle) > MOVE_HEADING45DEG ? halfSpeed : minSpeed;
      if (deltaAngle > 0) moveRotateLeft(ls, ls);
      else moveRotateRight(ls, ls);
      break;

    case MOVE_OBSTACLE:
      if (obstacle)  {
        moveStop();
        ls = random(0, MOVE_NUMOFDANCES);
        tMove.set(MOVE_PERIOD, TASK_FOREVER, moveDances[ls]);
      }
      break;
  }
}


void moveCalibrate() {

  if (moveCheckUpsideDown()) return;
  
  if (movingShaking) {
    ledYellowOff();
    ledGreenOn();
    tBlinkYellow.disable();
    tBlinkGreen.disable();
    tDistance.enable();
    moveNow = MOVE_NORMAL;
    halfSpeed = (byte) ((255 + (int) minSpeed)>>1);
    tMove.set (MOVE_PERIOD, TASK_FOREVER, &moveCallback );
    tMoveCheck.enable();
    shakeSettleSeconds = 1;
  }
  
  if (minSpeed <= 250) minSpeed += 5;
  else errorCondition();

  moveForward(minSpeed, minSpeed);
}


boolean moveCheckUpsideDown() {
  if (upsideDown) {
    moveStop();
    ledGreenOff();
    laserOff();
    taskManager.disableAll();
    tGyro.set(GYRO_PERIOD, CALIBRATE_CYCLES, &gyroCalibrate );
    tAccel.set(ACCEL_PERIOD, TASK_FOREVER, &accelCallback);
    shakeSettleSeconds = 3;  // Wait for at least three seconds for the robot to be motionless
    tAccel.enable();
    tGyro.enable();
  }
  return upsideDown;
}

boolean moveCheckObstacle () {
      if (obstacle) {
        moveNow = MOVE_OBSTACLE;
        tBlinkYellow.enable();
        laserOn();
      }
      else {
        tBlinkYellow.disable();
        ledYellowOff();
          laserOff();
      }
      return obstacle;
}

#define SC_RESETPREFHEADINGAFTER  10000 // 10 seconds
#define SC_MOVEBACKTIME  1
#define SC_ANGLETOLERANCE  500

int  mdScCounter, preferredHeading = 0, attemptsCounter, attemptsMax = 4, measureCounter;
long mdScDist[2];
#define NUM_ANGLES 2 // 6
int  angles[NUM_ANGLES+1] = {9000, -18000, 9000};

enum {
  SC_STOP,
  SC_MOVE_BACK,
  SC_MEASURE,
  SC_DECIDE,
} mdScState;

void moveDanceScientificInit() {
#ifdef _DEBUG_
//Serial.println("In moveDanceSwirlInit");
#endif
  mdScState = SC_STOP;
  tMove.setCallback(&moveDanceScientific);
  shakeSettleSeconds = 1;
  measureCounter = 0;
  mdScDist[0] = mdScDist[1] = 0;
  if ( currentHeading == ultimateHeading && (millis() - ultimateHeadingStart) > SC_RESETPREFHEADINGAFTER ) {
    preferredHeading = 0;
    attemptsCounter = 0;
    attemptsMax = 4;
  }
  tMoveCheck.disable();
}


void moveDanceScientific() {
  /* This is a "dance" to determine direction of the obstacle avoidance
   In this dance spider will:
   1. Stop
   2. If preferred direction of obstable avoidance is already set, turn 90 deg in that direction and continue
       however, if enough attempts to move into that direction was made and it did not result into reaching ultimate direction and maintaining it for over 5 seconds, 
       change the preferred direction to opposite, and increase number of attempts by 4, skip to step 8
   3. To determine preferred direction: Turn right 90 deg
   4. Measure distance to obstacle 3 times: straight ahead, + and - 5 degrees
   5. Turn left 180 deg
   6. Measure distance to obstacle 3 times: straight ahead, + and - 5 degrees
   7. Set preferred direction to the one with higher avarage distance to obstacle, turn into that direction
   8. Go back to normal move
  */
  
#ifdef _DEBUG_
//Serial.println("In moveDanceScientific");
#endif

  long deltaAngle = currentAngle - currentHeading;  // Angle is accumulated in 100's of degrees
  if (moveCheckUpsideDown()) return;

  switch (mdScState) {
    case SC_STOP:
      moveStop();
      if (movingShaking) return;
      moveBack(minSpeed, minSpeed);
      mdScCounter = SC_MOVEBACKTIME * 1000 / MOVE_PERIOD; // move back for 1 second
      mdScState = SC_MOVE_BACK;
      break;
    
    case SC_MOVE_BACK:
      if (--mdScCounter) return;
      moveStop();
      
// If the preferred heading was not established yet, we should select it based on the extended ping      
      if (preferredHeading == 0) {
        addToCurrentHeading( angles[measureCounter] ); //turn right 90 deg
        mdScState = SC_MEASURE;  
      }
      else {
        if (attemptsCounter++ > attemptsMax) {
          attemptsMax += 4;
          attemptsCounter = 0;
          preferredHeading = -preferredHeading;
          ledRedOn();
        }
        addToCurrentHeading(preferredHeading);
        moveNow = MOVE_CHECKSTILL;
        moveAdjustCounter = moveAdjustSeconds * 1000 / MOVE_PERIOD;
        tMove.setCallback(&moveCallback);
        tMoveCheck.enableDelayed();
      }
      break;
  
    case SC_MEASURE:
      if (abs(deltaAngle) > SC_ANGLETOLERANCE) {
        int ls = (abs(deltaAngle) > MOVE_HEADING45DEG) ? halfSpeed : minSpeed;
        if (deltaAngle > 0) moveRotateLeft(ls, ls);
        else moveRotateRight(ls, ls);
        return;
      }
      else {
        moveStop();
      }
      if (movingShaking) {
        return;
      }
      mdScDist[(measureCounter<<1)/NUM_ANGLES] += longRangePing(250000); 
      addToCurrentHeading(angles[++measureCounter]); 
      if (measureCounter < NUM_ANGLES)  break;
      mdScState = SC_DECIDE;
      break;
    
    case SC_DECIDE:
      if (mdScDist[0] == mdScDist[1]) {
        preferredHeading = (random(0, 1000) < 500) ? (-MOVE_HEADING90DEG) : MOVE_HEADING90DEG;
      }
      if (mdScDist[0] > mdScDist[1]) {
       preferredHeading = MOVE_HEADING90DEG;
      }
      else {
       preferredHeading = -MOVE_HEADING90DEG;
      }
      addToCurrentHeading(preferredHeading);
      attemptsCounter++;
      moveNow = MOVE_CHECKSTILL;
      moveAdjustCounter = moveAdjustSeconds * 1000 / MOVE_PERIOD;
      tMove.setCallback(&moveCallback);
      tMoveCheck.enableDelayed();
      break;
  }

}

void addToCurrentHeading(long aDelta) {
   currentHeading += aDelta;
   currentHeading %= MOVE_HEADING360DEG;
}

#define MCRETRIES  8
int mcRetries = 0;

void moveCheckCallback() {
  if (rightLegSpeed==0 && leftLegSpeed==0) return;
  if (movingShaking) {
    mcRetries = 0;
    return;
  }
  if (++mcRetries >= MCRETRIES) {
// We are not moving when we should - probably batteries - error    
    moveStop();
    laserOff();
    while (1) {
      ledRedOn(); delay(50);
      ledYellowOn(); delay(50);
      ledGreenOn(); delay(200);
      ledRedOff(); delay(50);
      ledYellowOff(); delay(50);
      ledGreenOff(); delay(200);
    }
  }
}


// ----------------------- end of Callback functions ------------------

void errorCondition() {
  // to-do: distinguish between errors vie enum status

  ledYellowOff();
  ledGreenOff();
  taskManager.disableAll();
  tBlinkRed.enable();
  tError.enable();
}



//------------------------- LED CONTROL functions ----------------------
//void ledOn(int aPin) {
//  digitalWrite(aPin, HIGH);
//}
//
//void ledOff(int aPin) {
//  digitalWrite(aPin, LOW);
//}

void ledRedOn () {
  pRedLedPin = HIGH;
}
void ledRedOff () {
  pRedLedPin = LOW;
}
void ledYellowOn () {
  pYellowLedPin = HIGH;
}
void ledYellowOff () {
  pYellowLedPin = LOW;
}
void ledGreenOn () {
  pGreenLedPin = HIGH;
}
void ledGreenOff () {
  pGreenLedPin = LOW;
}
void laserOn () {
  pLaserPin = HIGH;
}
void laserOff () {
  pLaserPin = LOW;
}


#ifdef _TASK_TIMECRITICAL
void checkOverrun() {
  pLED = Scheduler::currentScheduler().isOverrun()  ? HIGH: LOW;
}
#endif

// -------------- Main -----------------

void initPins() {
  pinMode(ECHOPIN, INPUT);
  pLED = LOW;
}

void setup () {

  initPins();
  
  Wire.begin();       //Initiate the Wire library and join the I2C bus as a master
  randomSeed(analogRead(0));
  
#ifdef _DEBUG_
  Serial.begin(115200);
  Serial.println(CToken);
#endif

  moveStop();

  ledRedOn();
  ledYellowOn();
  ledGreenOn();
  laserOn();
  
#ifdef _DEBUG_
//  Serial.println("Init move functions");
#endif

  power_adc_disable();
  power_adc_disable();

#ifndef _DEBUG_
  power_usart0_disable();
#endif

//  taskManager.init();

  gyroManager.addTask(tGyro);
  gyroManager.addTask(tAccel);
  
  taskManager.addTask(tDistance);
  taskManager.addTask(tMove);
  taskManager.addTask(tMoveCheck);
  taskManager.addTask(tBlinkRed);
  taskManager.addTask(tBlinkYellow);
  taskManager.addTask(tBlinkGreen);
  taskManager.addTask(tError);
  
  taskManager.setHighPriorityScheduler(&gyroManager);
  
//  taskManager.allowSleep(false);
#ifdef _DEBUG_  
  taskManager.addTask(tDisplay);
#endif

  shakeSettleSeconds = 3;  // Wait for at least three seconds for the robot to be motionless

  delay(2000);

  ledRedOff();
  ledYellowOff();
  ledGreenOff();
  laserOff();

  tAccel.enable();
  tGyro.enable();

  obstacle = false;
}


void loop ()
{
  taskManager.execute();
}



