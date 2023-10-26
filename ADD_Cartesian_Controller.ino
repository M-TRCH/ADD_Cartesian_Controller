
/*
*  ADD_Cartesian_Controllr.ino
*  
*  Created on: 15 Aug 2023
*  Author: m.teerachot
*  Tested with F030C6/C8, F091RC and G431CB
*/

///////////////////////////////////////////////////////////////////////////////
/* overall system */
#define UART1_RX_PIN    PA10
#define UART1_TX_PIN    PA9
#define UART1_BAUDRATE  250000
#define UART1_TIMEOUT   200
#define TRAJ_LED_PIN    PC13
#define TRAJ_LED(s)     digitalWrite(TRAJ_LED_PIN, s)
#define LIM_H_PIN       PB0
#define LIM_H           !digitalRead(LIM_H_PIN)

///////////////////////////////////////////////////////////////////////////////
/* stepper motor control */
#include "AccelStepper.h"

// pin config -> ena pin, dir pin, pul pin, ena invert, dir invert
AccelStepper motv(PA4, PA5, PA6, false, false);
AccelStepper moth(PA2, PA3, PA7, false, true);

int callback_v()  { motv.callback();  return 0; }
int callback_h()  { moth.callback();  return 0; }

///////////////////////////////////////////////////////////////////////////////
/* serial interface */
#define EndPack           '&'
#define Header            '@'
#define MsgReturnParam    '0'
#define MsgGoalReached    '1'
int8_t cmdIn;

int getCmd()
{
  if (Serial.find(Header)) 
  {
    int cmd = Serial.parseInt();
    return abs(cmd);
  }
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
/* cartesian */
#define CONST_PMM       13.87283237 // constant value between millimeter and pulse
#define mmTopul(mm)     (float)mm * CONST_PMM;
#define M2M_x           120.0   // vertical module to module
#define M2M_y           215.0   // horizontal module to module
#define k_pv            1.8
#define k_va            1.25
float relative_x = 0;
float relative_y = 0;
const float sys_x[] = {0, 380,   0, 165}; 
const float sys_y[] = {0,   5, 200, 200};
// <system position>
// index 0: home position 
// index 1: first module
// index 2: right basket
// index 3: left basket
const float org_x = 380;  // follow system position
const float org_y = 5;
int set_row = 1, set_col = 1, pointIn;

float toAbs_x(float refx)
{ 
  float absx = refx - relative_x;
  relative_x = refx;
  return absx;
}

float toAbs_y(float refy)
{ 
  float absy = refy - relative_y;
  relative_y = refy;
  return absy;
}

void operate(int axis, int state)
{
  switch (state)
  {
    // disable
    case 0:
      if (axis == 0 || axis == 2) 
      { 
        motv.operate(motv.DISABLE);
        Serial.println("Motor: vertical disable");
      }
      if (axis == 1 || axis == 2) 
      {
        moth.operate(moth.DISABLE);
        Serial.println("Motor: horizontal disable");
      }
      break;
    // enable
    case 1:
      if (axis == 0 || axis == 2) 
      { 
        motv.operate(motv.ENABLE);
        Serial.println("Motor: vertical enable");
      } 
      if (axis == 1 || axis == 2) 
      {
        moth.operate(moth.ENABLE);
        Serial.println("Motor: horizontal enable");
      }     
      break;
    // home
    case 2:
      if (axis == 0 || axis == 2) 
      {
        motv.operate(motv.DISABLE);
        Serial.println("Motor: vertical got home");
      }
      if (axis == 1 || axis == 2) 
      {
        horizontal_home();
      }
      break;
  }
}

void verticalEndlessDrive(float freq)
{
  if (freq == 0)
  {
    motv.stop();
    // return message 
    Serial.write(Header);
    Serial.write(MsgGoalReached);
    Serial.write(EndPack);
  }
  else
  {
    if (freq > 0) motv.direction(motv.DIR_CCW);
    else          motv.direction(motv.DIR_CW);  
    motv.start(freq);  
  }
}

void horizontalEndlessDrive(float freq)
{
  if (freq == 0)
  {
    moth.stop();
    // return message 
    Serial.write(Header);
    Serial.write(MsgGoalReached);
    Serial.write(EndPack);
  }
  else
  {
    if (freq > 0) moth.direction(moth.DIR_CCW);
    else          moth.direction(moth.DIR_CW);  
    moth.start(freq);  
  }
}

// main movement func
void indep_travel(int32_t pulv, float freqv, float accelv, float decelv, int32_t pulh, float freqh, float accelh, float decelh, bool lockv=false, bool lockh=false)
{
  // action
  if (pulv != 0)  motv.moveTo(pulv, freqv, accelv, decelv); 
  if (pulh != 0)  moth.moveTo(pulh, freqh, accelh, decelh); 
  int32_t difDistV = abs(pulv) - motv.AccelDist() - motv.DecelDist();
  int32_t difDistH = abs(pulh) - moth.AccelDist() - moth.DecelDist();

  Serial.write(Header);
  Serial.write(MsgReturnParam);
  Serial.println("<Vertical>");
  Serial.print("  Operating time:\t");    Serial.println(motv.timeout());
  Serial.print("  Total distance:\t");    Serial.println(pulv);
  Serial.print("  Accel distance:\t");    Serial.println(motv.AccelDist());
  Serial.print("  Decel distance:\t");    Serial.println(motv.DecelDist());
  Serial.print("  Diff distance:\t");     Serial.println(difDistV);
  Serial.println("<Horizontal>");
  Serial.print("  Operating time:\t");    Serial.println(moth.timeout());
  Serial.print("  Total distance:\t");    Serial.println(pulh);
  Serial.print("  Accel distance:\t");    Serial.println(moth.AccelDist());
  Serial.print("  Decel distance:\t");    Serial.println(moth.DecelDist());
  Serial.print("  Diff distance:\t");     Serial.println(difDistH);
  Serial.write(EndPack);
  
  // waiting for finish 
  if (pulv != 0)  motv.start();
  if (pulh != 0)  moth.start();
  Serial.println("Cartesian: moving");
  while((pulv != 0 && !motv.finished()) || (pulh != 0 && !moth.finished()));

  // motor stop
  if (!lockv)  motv.operate(motv.DISABLE);
  if (!lockh)  moth.operate(moth.DISABLE);
  Serial.println("Cartesian: goal reached");
}

void indep_travel_mm(float disv, float freqv, float accelv, float decelv, int32_t dish, float freqh, float accelh, float decelh, bool lockv=false, bool lockh=false)
{
  disv   = mmTopul(disv);
  freqv  = mmTopul(freqv);
  accelv = mmTopul(accelv);
  decelv = mmTopul(decelv);
  dish   = mmTopul(dish);
  freqh  = mmTopul(freqh);
  accelh = mmTopul(accelh);
  decelh = mmTopul(decelh);
  indep_travel(disv, freqv, accelv, decelv, dish, freqh, accelh, decelh, lockv, lockh); 
}

void sync_travel(int32_t pul, float freq, float accel, float decel)
{
  indep_travel(pul, freq, accel, decel, pul, freq, accel, decel);
}

void sync_travel_mm(float dis, float freq, float accel, float decel)
{
  indep_travel_mm(dis, freq, accel, decel, dis, freq, accel, decel); 
}

void cartesian_travel(float x_target, float y_target, float lock=false)
{
  float resultant_vect = sqrt(pow(x_target, 2) + pow(y_target, 2)); 
  float x_unit = fabs(x_target) / resultant_vect;
  float y_unit = fabs(y_target) / resultant_vect;
  float freq_cal = resultant_vect / k_pv; 
  float accel_cal = freq_cal / k_va;

  Serial.write(Header);
  Serial.write(MsgReturnParam);
  Serial.println("<Cartesian>");
  Serial.print("  Resultant vector:\t");    Serial.println((int)resultant_vect);
  Serial.print("  Target position[x]:\t");  Serial.println((int)x_target);
  Serial.print("  Target position[y]:\t");  Serial.println((int)y_target);
  Serial.print("  Unit vector[x]:\t");      Serial.println(x_unit);
  Serial.print("  Unit vector[y]:\t");      Serial.println(y_unit);
  Serial.write(EndPack);
  
  indep_travel_mm
  (
    y_target, freq_cal * y_unit, accel_cal * y_unit, accel_cal * y_unit, 
    x_target, freq_cal * x_unit, accel_cal * x_unit, accel_cal * x_unit,
    lock, lock
  );

  // return message 
  Serial.write(Header);
  Serial.write(MsgGoalReached);
  Serial.write(EndPack);
}

void cartesian_relative(float x_target, float y_target, float lock=false)
{
  x_target = toAbs_x(x_target);
  y_target = toAbs_y(y_target);
  cartesian_travel(x_target, y_target, lock);
}

void horizontal_home()
{
  if (!LIM_H)
  {
    // motor config
    const float freq = 1000;

    // motor start
    moth.operate(moth.ENABLE);
    moth.direction(moth.DIR_CCW);
    moth.start(freq);
    Serial.println("Motor: horizontal homing");
    
    // waiting for finish 
    while(!LIM_H);  
    moth.stop();
    
    moth.operate(moth.DISABLE);
    Serial.println("Motor: horizontal got home");
  }
  else 
  {
    moth.operate(moth.DISABLE);
    Serial.println("Motor: horizontal got home");
  }    
  relative_x = 0;
  relative_y = 0;
}

void setup() 
{
  /* sys pin */
  pinMode(TRAJ_LED_PIN, OUTPUT);
  pinMode(LIM_H_PIN, INPUT_PULLUP);
  
  TRAJ_LED(LOW);
   
  /* uart */ 
  Serial.setRx(UART1_RX_PIN);
  Serial.setTx(UART1_TX_PIN);
  Serial.begin(UART1_BAUDRATE);
  Serial.setTimeout(UART1_TIMEOUT);
  
  /* motor init */
  motv.init(callback_v);
  moth.init(callback_h);
  operate(2, 0);    // cartesian disable

  Serial.println("System: <Cartesian>");
  Serial.println("System: ready");
}

void loop()
{ 
  /* get command */
  cmdIn = getCmd();
  if (cmdIn + 1)
  {
    Serial.print("System: command[");
    Serial.print(cmdIn);
    Serial.println("]");
     
    switch (cmdIn)
    {
      case 0:
        operate(Serial.parseInt(), 0);
        break;
      case 1:
        operate(Serial.parseInt(), 1);
        break; 
      case 2:  
        operate(Serial.parseInt(), 2);
        break;
      case 3:  
        pointIn = Serial.parseInt();
        cartesian_relative(sys_x[pointIn], sys_y[pointIn], true);
        break;    
      case 4:  
        verticalEndlessDrive(Serial.parseInt());
        break;
      case 5: 
        horizontalEndlessDrive(Serial.parseInt());
      case 6:
        set_row = Serial.parseInt();
        Serial.print("Cartesian: row ");
        Serial.println(set_row);
        break;
      case 7:
        set_col = 7 - Serial.parseInt();
        Serial.print("Cartesian: column ");
        Serial.println(set_col);
        break;
      case 8:
        cartesian_relative(org_x + M2M_x * (set_col-1), org_y + M2M_y * (set_row-1), true);
        break;
      case 9:
        relative_x = 0;
        relative_y = 0;
        Serial.println("Cartesian: zero relative value");
        break;
    }
    cmdIn = 0;  // reset command
  }
}
