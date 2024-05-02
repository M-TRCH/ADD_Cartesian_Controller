 
/*
 *  Cartesian.h
 *  
 *  Created on: 03 Apr 2024
 *  Author: M.Teerachot
 *  Tested with STM32G431CB
 */

/*
 *  Note:
 *  (*) -> the most important functions
 */
 
#ifndef CARTESIAN_H
#define CARTESIAN_H
#include "AccelStepper.h"

/* (1) pin cofig
 * refer by STM32G431CB pinout
*/

#define ALMV_PIN    PC4     // driver alarm pin
#define ALMH_PIN    PB0
#define BKA_PIN     PC13    // relay pin
#define BKB_PIN     PC14     
#define ALMV        !digitalRead(ALMV_PIN)
#define ALMH        !digitalRead(ALMH_PIN)
#define LIMITH_PIN  PB12
#define LIMITV_PIN  PB13
#define LIMITH      !digitalRead(LIMITH_PIN)
#define LIMITV      !digitalRead(LIMITV_PIN)

/* (2) constant variables */
// 2.1 printing 
//#define MOTOR_OPERATE_PRINT
//#define RAIL_HOME_PRINT
//#define INDEP_TRAVEL_PRINT  
//#define CART_TRAVEL_PRINT
//#define GRAPH_TUNNING
// 2.2 general purpose 
#define X_AXIS        0
#define Y_AXIS        1
#define VERTICAL      0  
#define HORIZONTAL    1
#define CARTESIAN     2
#define OFF           0
#define ON            1
// 2.3 error identify
#define REACHED       1
#define ERR_TIMEOUT   -1   
#define ERR_ALARM     -2
#define ERR_ZERO_MOVE -3
// 2.4 constant value 
// constant value between millimeter and pulse **depends on system
//#define CONST_PMM     55.5172414  // @4000 pulse
#define CONST_PMM     11.1034483  // @800 pulse
// 2.5 macro function
#define mmToPul(mm)       (float)mm * CONST_PMM  
#define pulTomm(pul)      (float)pul / CONST_PMM
#define pulToMeter(pul)   (float)pul / CONST_PMM / 1000.0

/* (3) create object  
 * AccelStep constructors
 * pin:     (enable pin, direction pin, pulse pin) 
 * boolean: (enable invert, direction invert)
 */

AccelStepper motv(PA4, PA5, PA6, false, true);
AccelStepper moth(PB1, PB2, PA7, false, false);

/* (4) interrupt event */
int callback_v()  { motv.callback();  return 0; }
int callback_h()  { moth.callback();  return 0; }

/* (5) cartesian variables */
// 5.1 constant value
const float modulePosRow[] = {0, 0, 192, 383, 575, 762, 952, 1140}; // module position (mm unit)
const float modulePosCol[] = {0, 1450, 1300, 1150, 1001, 849, 697, 547, 397};
const float offsetPosRow = 0;
const float offsetPosCol = 25;
// 5.2 dynamic value
float relativeMem[2];   // relative value (x, y) *use with relateToAbs() function

void brake(int state, int delayTime=0)
{
  digitalWrite(BKA_PIN, !state);
  digitalWrite(BKB_PIN, !state);
  delay(delayTime);
}

void cartesianErrPrint(int err)
{
  switch (err)
  {
    case ERR_TIMEOUT:
      Serial.println("Cartesian: Timeout");
      break;
    case ERR_ALARM:
      Serial.println("Cartesian: Driver is alarm");
      break;
    case ERR_ZERO_MOVE:
      Serial.println("Cartesian: Zero movement");
      break;
    case REACHED:
      Serial.println("Cartesian: Goal reached");
      break;
  }
}

float relateToAbs(int refVal, int axis)
{
  float absVal = refVal - relativeMem[axis];
  relativeMem[axis] = refVal;
  return absVal;
}

void zeroRalative()
{
  relativeMem[0] = 0;
  relativeMem[1] = 0;
}

void enable(int axis, int t=1) // enable delay should be more than 5 us.
{
  switch (axis)
  {
    case VERTICAL:
      brake(OFF, 0);
      motv.operate(motv.ENABLE, t);
      #ifdef MOTOR_OPERATE_PRINT
        Serial.println(F("Motor: vertical enable")); 
      #endif
      break;
    case HORIZONTAL:    
      moth.operate(moth.ENABLE, t);
      #ifdef MOTOR_OPERATE_PRINT 
        Serial.println(F("Motor: horizontal enable")); 
      #endif
      break;
    case CARTESIAN:
      brake(OFF, 0);
      motv.operate(motv.ENABLE);
      moth.operate(moth.ENABLE, t);      
      #ifdef MOTOR_OPERATE_PRINT  
        Serial.println(F("Motor: cartesian enable")); 
      #endif
      break;
  }
}

void disable(int axis, int t=0)
{
  switch (axis)
  {
    case VERTICAL:
      brake(ON, 0);
      motv.operate(motv.DISABLE, t);
      #ifdef MOTOR_OPERATE_PRINT 
        Serial.println(F("Motor: vertical disable")); 
      #endif
      break;
    case HORIZONTAL:    
      moth.operate(moth.DISABLE, t);
      #ifdef MOTOR_OPERATE_PRINT 
        Serial.println(F("Motor: horizontal disable")); 
      #endif
      break;
    case CARTESIAN:
      brake(ON, 0);
      motv.operate(motv.DISABLE);
      moth.operate(moth.DISABLE, t);      
      #ifdef MOTOR_OPERATE_PRINT  
        Serial.println(F("Motor: cartesian disable")); 
      #endif
      break;
  }
}

/* base functions of other movement functions */
int independentTravel(int32_t pulv, float freqv, float accelv, float jerkv, bool lockv, 
                      int32_t pulh, float freqh, float accelh, float jerkh, bool lockh, int timeout=200)
{
  // 0. data preparing
       if (pulv == 0 && pulh == 0)  return ERR_ZERO_MOVE;
  else if (pulv != 0 && pulh == 0)  
  {
    motv.moveTo(pulv, freqv, accelv, jerkv); 
    motv.simulate();
    motv.moveTo(pulv, freqv, accelv, jerkv); 
  }
  else if (pulv == 0 && pulh != 0)  
  {
    moth.moveTo(pulh, freqh, accelh, jerkh); 
    moth.simulate();
    moth.moveTo(pulh, freqh, accelh, jerkh); 
  }
  else if (pulv != 0 && pulh != 0) 
  {
    motv.moveTo(pulv, freqv, accelv, jerkv);
    moth.moveTo(pulh, freqh, accelh, jerkh);
    motv.simulate();
    moth.simulate();
    motv.moveTo(pulv, freqv, accelv, jerkv);
    moth.moveTo(pulh, freqh, accelh, jerkh);
  } 

  // 1. show parameter
  #ifdef INDEP_TRAVEL_PRINT
  if (pulv != 0)
  {
    Serial.println(F("Cartesian: info<Vertical>"));
    Serial.print(F("  Operating time:\t"));       Serial.println(motv.timeout());
    Serial.print(F("  Average Velocity:\t"));     Serial.print(abs(pulv)/motv.timeout());                         Serial.println(F("\tpul/ms"));
    Serial.print(F("\t\t\t"));                    Serial.print(pulToMeter(abs(pulv))/(motv.timeout()/1000.0));    Serial.println(F("\tm/s"));
  }
  if (pulh != 0)
  {
    Serial.println(F("Cartesian: info<Horizontal>"));
    Serial.print(F("  Operating time:\t"));       Serial.println(moth.timeout());
    Serial.print(F("  Average Velocity:\t"));     Serial.print(abs(pulh)/moth.timeout());    Serial.println(F("\tpul/ms"));
    Serial.print(F("\t\t\t"));                    Serial.print(pulToMeter(abs(pulh))/(moth.timeout()/1000.0));    Serial.println(F("\tm/s"));
  }
  #endif

  // 2. motor preparing
  #ifdef INDEP_TRAVEL_PRINT
    Serial.println("Cartesian: moving");
  #endif
  
       if (pulv != 0 && pulh == 0)  motv.start();
  else if (pulv == 0 && pulh != 0)  moth.start();
  else if (pulv != 0 && pulh != 0) 
  {
    motv.start();
    moth.start();
  } 
  
  // 3. motor started
  if (abs(pulv) > abs(pulh))  timeout += motv.timeout();
  else                        timeout += moth.timeout();
  unsigned long startTime = millis();
  
  while((!motv.finished() && pulv != 0) || (!moth.finished() && pulh != 0))
  {
    if (ALMV || ALMH)               
    {
      brake(1);
      return ERR_ALARM; 
    }
    if (millis()-startTime >= timeout)  return ERR_TIMEOUT;

    #ifdef GRAPH_TUNNING
           if (pulv != 0 && pulh == 0)  motv.trajectoryGraphPrint();
      else if (pulh != 0 && pulv == 0)  moth.trajectoryGraphPrint();
    #endif
  }
  
  // 4. motor stop (if you need)
  if (!lockv)   motv.operate(motv.DISABLE);
  if (!lockh)   moth.operate(moth.DISABLE);

  return REACHED;
}

int verticalTravel(int32_t pulv, float freqv, float accelv, float jerkv, bool lockv, int timeout=1000)
{
  return independentTravel(pulv, freqv, accelv, jerkv, lockv, 0, 0, 0, 0, false, timeout);
}

int horizontalTravel(int32_t pulh, float freqh, float accelh, float jerkh, bool lockh, int timeout=1000)
{
  return independentTravel(0, 0, 0, 0, false, pulh, freqh, accelh, jerkh, lockh, timeout);
}

bool homeHonrizontal(Stream *serial, String headPacket, String endPacket, float freq, long timeout)
{
  // 0. data preparing
  unsigned long timer;
  freq = fabs(freq);
  timeout = abs(timeout);
  
  // 1. choose a direction
  // exit function 
  if (LIMITH) 
  {
#ifdef RAIL_HOME_PRINT
    Serial.println(F("Honrizontal: still at home"));
#endif    
    return 1;
  }

  // 2. start process
#ifdef RAIL_HOME_PRINT
    Serial.println(F("Honrizontal: going home"));
#endif

  serial->println(headPacket + 'T' + endPacket);
  enable(HORIZONTAL);
  moth.setDir(moth.DIR_CCW);  
  moth.start(freq);  
  timer = millis();
     
  while(1)
  {
    // 2.1 timeout
    if (millis()-timer >= timeout)
    {
      moth.stop();
      disable(HORIZONTAL);
#ifdef RAIL_HOME_PRINT
    Serial.println(F("Honrizontal: timeout"));
#endif
      break;
    } 
    // 2.2 reached
    if (LIMITH)
    {
      moth.stop();
      disable(HORIZONTAL);
#ifdef RAIL_HOME_PRINT
      Serial.println(F("Honrizontal: got home"));
#endif
      break;
    }
  }
  return 0;
}

bool homeVertical(Stream *serial, String headPacket, String endPacket, float freq, long timeout)
{
  // 0. data preparing
  unsigned long timer;
  freq = fabs(freq);
  timeout = abs(timeout);
  
  // 1. choose a direction
  // exit function 
  if (LIMITV) 
  {
#ifdef RAIL_HOME_PRINT
    Serial.println(F("Vertical: still at home"));
#endif    
    return 1;
  }

  // 2. start process
#ifdef RAIL_HOME_PRINT
    Serial.println(F("Vertical: going home"));
#endif

  serial->println(headPacket + 'T' + endPacket);
  enable(VERTICAL);
  motv.setDir(motv.DIR_CCW);  
  motv.start(freq);  
  timer = millis();
     
  while(1)
  {
    // 2.1 timeout
    if (millis()-timer >= timeout)
    {
      motv.stop();
      disable(VERTICAL);
#ifdef RAIL_HOME_PRINT
    Serial.println(F("Vertical: timeout"));
#endif
      break;
    } 
    // 2.2 reached
    if (LIMITV)
    {
      motv.stop();
      disable(VERTICAL);
#ifdef RAIL_HOME_PRINT
      Serial.println(F("Vertical: got home"));
#endif
      break;
    }
  }
  return 0;
}

void togHorizontal(int32_t pos, uint32_t vel, uint32_t accel, uint32_t jerk, int stopTime=0, int cnt=1)
{
  // 1. enable
  enable(HORIZONTAL);
  // 2. run
  for(int i=0; i<cnt; i++)
  {
    int8_t err = horizontalTravel(pos, vel, accel, jerk, true);   // forward
    cartesianErrPrint(err);
    delay(stopTime);
    err = horizontalTravel(-pos, vel, accel, jerk, false);        // backward
    cartesianErrPrint(err);
  }
  // 3. disable
  disable(HORIZONTAL);
}

void togVertical(int32_t pos, uint32_t vel, uint32_t accel, uint32_t jerk, int stopTime=0, int cnt=1)
{
  // 1. enable
  brake(OFF, 0);
  enable(VERTICAL);
  // 2. run 
  for(int i=0; i<cnt; i++)
  {
    int8_t err = verticalTravel(pos, vel, accel, jerk, true);   // forward
    cartesianErrPrint(err);
    delay(stopTime);
    err = verticalTravel(-pos, vel, accel, jerk, false);        // backward
    cartesianErrPrint(err);
  }
  // 3. disable
  brake(ON, 0);
  disable(VERTICAL);
}

void togSynchronize(int32_t pos, uint32_t vel, uint32_t accel, uint32_t jerk, int stopTime=0, int cnt=1)
{
  // 1. enable
  brake(OFF, 0);
  enable(CARTESIAN);
  // 2. run 
  for(int i=0; i<cnt; i++)
  {
    int8_t err = independentTravel(pos, vel, accel, jerk, true, pos, vel, accel, jerk, true);   // forward
    cartesianErrPrint(err);
    delay(stopTime);
    err = independentTravel(-pos, vel, accel, jerk, false, -pos, vel, accel, jerk, false);         // backward
    cartesianErrPrint(err);
  }
  // 3. disable
  brake(ON, 0);
  disable(CARTESIAN);
}

/* base cartesian drive function */
int cartesianTravel(float x_target, float y_target, bool preEnable, bool postDisable, int timeout=1000)
{
  // 1.convert to usable variables
  x_target = mmToPul(x_target);
  y_target = mmToPul(y_target);

  // 2.parameter calculate 
  float vectResult = sqrt(pow(x_target, 2) + pow(y_target, 2)); 
  float x_unit = fabs(x_target) / vectResult;
  float y_unit = fabs(y_target) / vectResult;

  // old speed
//  float calVel   = vectResult * 0.240; 
//  float calAccel = calVel     * 0.800;
//  float calJerk  = calAccel   * 1.800;

  float calVel   = vectResult *  0.32; 
  float calAccel = calVel     *  1.00;
  float calJerk  = calAccel   * 10.00;
  
  #ifdef CART_TRAVEL_PRINT
    Serial.println(F("Cartesian: info<Overall>"));
    Serial.print(F("  Target position[x]:\t"));   Serial.println((int)x_target);
    Serial.print(F("  Target position[y]:\t"));   Serial.println((int)y_target);
    Serial.print(F("  Resultant vector:\t"));     Serial.println((int)resultant_vect);
    Serial.print(F("  Unit vector[x]:\t"));       Serial.println(x_unit, 3);
    Serial.print(F("  Unit vector[y]:\t"));       Serial.println(y_unit, 3);
    Serial.print(F("  Velocity:\t\t"));           Serial.println((int)freq_cal);
    Serial.print(F("  Acceleration:\t\t"));       Serial.println((int)accel_cal);
  #endif
  
  // 3.prepare output variables
  float x_vel   = x_unit * calVel;
  float y_vel   = y_unit * calVel;
  float x_accel = x_unit * calAccel;
  float y_accel = y_unit * calAccel;
  float x_jerk  = x_unit * calJerk;
  float y_jerk  = y_unit * calJerk;

  // 4.pre start
  if (preEnable)
    enable(CARTESIAN);
  
  // 5.cartesian is working
  return independentTravel
  (
    y_target, y_vel, y_accel, y_jerk, !postDisable, 
    x_target, x_vel, x_accel, x_jerk, !postDisable, timeout
  );
}

void togCartesian(float x_target, float y_target, int stopTime=0, int cnt=1)
{
  // 1. enable
  brake(OFF, 0);
  enable(CARTESIAN);
  // 2. run 
  for(int i=0; i<cnt; i++)
  {
    int8_t err = cartesianTravel(x_target, y_target, false, false);   // forward
    cartesianErrPrint(err);
    delay(stopTime);
    err = cartesianTravel(-x_target, -y_target, false, false);        // backward
    cartesianErrPrint(err);
  }
  // 3. disable
  brake(ON, 0);
  disable(CARTESIAN);
}

int cartesianTravelRelative(float x_target, float y_target, bool preEnable, bool postDisable, int timeout=1000)
{
  x_target = relateToAbs(x_target, X_AXIS);
  y_target = relateToAbs(y_target, Y_AXIS);
  return cartesianTravel(x_target, y_target, preEnable, postDisable, timeout);
}

void togCartesianRelative(float x_target, float y_target, int stopTime=0, bool forward=false)
{
  // 1. enable
  if (forward)
  {
    brake(OFF, 0);
    enable(CARTESIAN);
  }
  
  // 2. run 
  int8_t err = cartesianTravelRelative(x_target, y_target, false, false);   
  cartesianErrPrint(err);
  delay(stopTime);
  
  // 3. disable
  if (!forward)
  {
    brake(ON, 0);
    disable(CARTESIAN); 
  }
}

/* final function this project */
int cartesianTravelChannel(int row, int col, bool preEnable, bool postDisable, int timeout=1000)
{
  row = constrain(row, 1, 7);
  col = constrain(col, 1, 8);
  return  cartesianTravelRelative(modulePosCol[col]+offsetPosCol, modulePosRow[row]+offsetPosRow, preEnable, postDisable, timeout);
}

void togCartesianTravelChannel(int row, int col, int stopTime=0, bool forward=false)
{
  // 1. enable
  if (forward)
  {
    brake(OFF, 0);
    enable(CARTESIAN);
  }
  
  // 2. run 
  int8_t err = cartesianTravelChannel(row, col, false, false);  // forward
  cartesianErrPrint(err);
  delay(stopTime);
  
  // 3. disable
  if (!forward)
  {
    brake(ON, 0);
    disable(CARTESIAN); 
  }
}

#endif
