
/*
*  ADD_Cartesian_Controllr.ino
*  
*  Created on: 04 Aug 2023
*  Author: m.teerachot
*  Tested with F030C6/C8, F091RC and G431CB
*/

///////////////////////////////////////////////////////////////////////////////
/* overall system */
#define UART1_RX_PIN    PA10
#define UART1_TX_PIN    PA9
#define UART1_BAUDRATE  9600
#define UART1_TIMEOUT   200
#define TRAJ_LED_PIN    PC13
#define TRAJ_LED(s)     digitalWrite(TRAJ_LED_PIN, s)

///////////////////////////////////////////////////////////////////////////////
/* stepper motor control */
#include "AccelStepper.h"

// pin config -> enable, direction, pulse
AccelStepper motv(PA4, PA5, PA6, false, false);
AccelStepper moth(PA2, PA3, PA7, false, true);

int callback_v()  { motv.callback();  return 0; }
int callback_h()  { moth.callback();  return 0; }

///////////////////////////////////////////////////////////////////////////////
/* serial interface */
uint8_t cmdIn;

int getCmd()
{
  if (Serial.find('#') && Serial.find('#')) 
  {
    int cmd = Serial.parseInt();
    return abs(cmd);
  }
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
/* cartesian */
#define CONST_PMM     13.87283237 // constrant value between millimeter and pulse
#define mmTopul(mm)   (float)mm * CONST_PMM;
#define k_soft        2.375
#define div 2
bool fw_state = false;

void indep_travel(int32_t pulv, float freqv, float accelv, float decelv, int32_t pulh, float freqh, float accelh, float decelh)
{
  // motor start
  motv.operate(motv.ENABLE);
  moth.operate(moth.ENABLE);

  // action
  motv.moveTo(pulv, freqv, accelv, decelv); 
  moth.moveTo(pulh, freqh, accelh, decelh); 
  int32_t difDistV = abs(pulv) - motv.AccelDist() - motv.DecelDist();
  int32_t difDistH = abs(pulh) - moth.AccelDist() - moth.DecelDist();
  Serial.println("1.Vertical");
  Serial.print("  Operating time:\t");    Serial.println(motv.timeout());
  Serial.print("  Total distance:\t");    Serial.println(pulv);
  Serial.print("  Accel distance:\t");    Serial.println(motv.AccelDist());
  Serial.print("  Decel distance:\t");    Serial.println(motv.DecelDist());
  Serial.print("  Diff distance:\t");     Serial.println(difDistV);
  Serial.println("2.Horizontal");
  Serial.print("  Operating time:\t");    Serial.println(moth.timeout());
  Serial.print("  Total distance:\t");    Serial.println(pulh);
  Serial.print("  Accel distance:\t");    Serial.println(moth.AccelDist());
  Serial.print("  Decel distance:\t");    Serial.println(moth.DecelDist());
  Serial.print("  Diff distance:\t");     Serial.println(difDistH);
  
  // waiting for finish 
  motv.start();
  moth.start();
  Serial.println("\nCartesian starting");
  while(!motv.finished() || !moth.finished());
  delay(500);

  // motor stop
  motv.operate(motv.DISABLE);
  moth.operate(moth.DISABLE);
  Serial.println("Goal reached\n");
}

void indep_travel_mm(float disv, float freqv, float accelv, float decelv, int32_t dish, float freqh, float accelh, float decelh)
{
  disv   = mmTopul(disv);
  freqv  = mmTopul(freqv);
  accelv = mmTopul(accelv);
  decelv = mmTopul(decelv);
  dish   = mmTopul(dish);
  freqh  = mmTopul(freqh);
  accelh = mmTopul(accelh);
  decelh = mmTopul(decelh);
  indep_travel(disv, freqv, accelv, decelv, dish, freqh, accelh, decelh); 
}

void sync_travel(int32_t pul, float freq, float accel, float decel)
{
  indep_travel(pul, freq, accel, decel, pul, freq, accel, decel);
}

void sync_travel_mm(float dis, float freq, float accel, float decel)
{
  indep_travel_mm(dis, freq, accel, decel, dis, freq, accel, decel); 
}

void cartesian_travel(float x_target, float y_target, float freq, float accel, float decel)
{
  float resultant_vect = sqrt(pow(x_target, 2) + pow(y_target, 2)); 
  float x_unit = fabs(x_target) / resultant_vect;
  float y_unit = fabs(y_target) / resultant_vect;

  Serial.println("0.Cartesian");
  Serial.print("  Resultant vector:\t");    Serial.println(resultant_vect);
  Serial.print("  Target position[x]:\t");  Serial.println(x_target);
  Serial.print("  Target position[y]:\t");  Serial.println(y_target);
  Serial.print("  Unit vector[x]:\t");      Serial.println(x_unit);
  Serial.print("  Unit vector[y]:\t");      Serial.println(y_unit);
  
  indep_travel_mm
  (
    y_target, freq * y_unit, accel * y_unit, decel * y_unit, 
    x_target, freq * x_unit, accel * x_unit, decel * x_unit
  );
}

void setup() 
{
  /* sys pin */
  pinMode(TRAJ_LED_PIN, OUTPUT);
  TRAJ_LED(LOW);
   
  /* uart */ 
  Serial.setRx(UART1_RX_PIN);
  Serial.setTx(UART1_TX_PIN);
  Serial.begin(UART1_BAUDRATE);
  Serial.setTimeout(UART1_TIMEOUT);
  
  /* motor init */
  motv.init(callback_v);
  moth.init(callback_h);
  motv.operate(motv.DISABLE);
  moth.operate(moth.DISABLE);
  
  Serial.println("Motor is ready!\n");
}

void loop()
{
  /* var for case (1, 2) */
  const float goalPos = 950;
  const float goalVel = goalPos / k_soft;

  /* var for case (3, 4) */
  const float cartPos = 800;
  const float cartVel = cartPos / k_soft;
  const int p = 4;
  const float goal_x[5] = {400, 950, 600, 300, 100};
  const float goal_y[5] = {800, 950, 200, 500, 950};
  
  /* get command */
  cmdIn = getCmd();
  if (cmdIn)
  {
    Serial.print("Get command: ");
    Serial.println(cmdIn);
   
    switch (cmdIn)
    {
      case 1:   // sync forward
        if (!fw_state)
        {
          fw_state = true;
          sync_travel_mm(goalPos, goalVel, 200, 200);
        }
        break;
      case 2:   // sync backward
        if (fw_state)
        {
          fw_state = false;
          sync_travel_mm(-goalPos, goalVel, 200, 200);
        }
        break; 
      case 3:
        if (!fw_state)
        {
          fw_state = true;
          cartesian_travel(goal_x[p], goal_y[p], cartVel, 200, 200);
        }
        break;
      case 4:
        if (fw_state)
        {
          fw_state = false;
          cartesian_travel(-goal_x[p], -goal_y[p], cartVel, 200, 200);
        }
        break;
      default:  // nothing
        return;
    }
    cmdIn = 0;  // reset command
  }
}
