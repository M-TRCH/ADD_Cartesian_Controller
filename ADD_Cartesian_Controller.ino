
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
#define UART1_BAUDRATE  9600
#define UART1_TIMEOUT   200
#define TRAJ_LED_PIN    PC13
#define TRAJ_LED(s)     digitalWrite(TRAJ_LED_PIN, s)

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
uint8_t cmdIn;

int getCmd()
{
  if (Serial.find('@')) 
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
bool fw_state = false;

// var for case (1, 2) 
const float syncPos = 950;
const float syncVel = syncPos / k_soft;

// var for case (3, 4) 
const float cartPos = 800;
const float cartVel = cartPos / k_soft;
const float cart_x[] = {608, -608};
const float cart_y[] = {370, -160};

// main movement func
void indep_travel(int32_t pulv, float freqv, float accelv, float decelv, int32_t pulh, float freqh, float accelh, float decelh, bool lockv=false, bool lockh=false)
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
  Serial.println("\nCartesian starting...");
  while(!motv.finished() || !moth.finished());
  delay(500);

  // motor stop
  if (!lockv)  motv.operate(motv.DISABLE);
  if (!lockh)  moth.operate(moth.DISABLE);
  Serial.println("Goal reached\n");
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

void cartesian_travel(float x_target, float y_target, float freq, float accel, float decel, float lock=false)
{
  float resultant_vect = sqrt(pow(x_target, 2) + pow(y_target, 2)); 
  float x_unit = fabs(x_target) / resultant_vect;
  float y_unit = fabs(y_target) / resultant_vect;
  float freq_cal = resultant_vect / k_soft; // >> testing 
  
  Serial.println("0.Cartesian");
  Serial.print("  Resultant vector:\t");    Serial.println(resultant_vect);
  Serial.print("  Target position[x]:\t");  Serial.println(x_target);
  Serial.print("  Target position[y]:\t");  Serial.println(y_target);
  Serial.print("  Unit vector[x]:\t");      Serial.println(x_unit);
  Serial.print("  Unit vector[y]:\t");      Serial.println(y_unit);
  
  indep_travel_mm
  (
    y_target, freq_cal * y_unit, accel * y_unit, decel * y_unit, 
    x_target, freq_cal * x_unit, accel * x_unit, decel * x_unit,
    lock, lock
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
  /* get command */
  cmdIn = getCmd();
  if (cmdIn)
  {
    Serial.print("Get command: ");
    Serial.println(cmdIn);
   
    switch (cmdIn)
    {
      case 1:   // fw sync
        if (!fw_state)
        {
          fw_state = true;
          sync_travel_mm(syncPos, syncVel, 200, 200);
        }
        break;
      case 2:   // bw sync
        if (fw_state)
        {
          fw_state = false;
          sync_travel_mm(-syncPos, syncVel, 200, 200);
        }
        break; 
      case 3:   // pick pos
        cartesian_travel(cart_x[0], cart_y[0], cartVel, 200, 200, true);
        break;
      case 4:   // place pos
        cartesian_travel(cart_x[1], cart_y[1], cartVel, 200, 200, true);
        break;
      case 5:   // home
        motv.operate(motv.DISABLE);
        moth.operate(moth.DISABLE);
        break;
      default:  // nothing
        return;
    }
    cmdIn = 0;  // reset command
  }
}
