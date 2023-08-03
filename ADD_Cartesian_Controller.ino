
/*
*  ADD_Cartesian_Controllr.ino
*  
*  Created on: 31 Jul 2023
*  Author: m.teerachot
*  Tested with F030C6/C8, F091RC and G431CB
*/

#include "AccelStepper.h"

// pin config -> enable, direction, pulse
AccelStepper motv(PA4, PA5, PA6, false, false);
AccelStepper moth(PA2, PA3, PA7, false, true);

int callback_v()
{
  motv.callback();
  return 0;
}

int callback_h()
{
  moth.callback();
  return 0;
}

void setup() 
{
  /* sys pin */
  pinMode(PC13, OUTPUT);
  pinMode(PB0, INPUT_PULLUP);
  digitalWrite(PC13, LOW);
  
  /* uart */ 
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(9600);
  Serial.setTimeout(200);
  
  /* motor init */
  motv.init(callback_v);
  moth.init(callback_h);

//#define goHome
#ifdef goHome
  motv.operate(motv.ENABLE);  
  motv.direction(motv.DIR_CCW);
  while(digitalRead(PB0))
  {
    motv.start(1000);  
    delay(1);
  }
  motv.stop();      delay(500);
  motv.direction(motv.DIR_CW);
  motv.start(500);  delay(300);  
  motv.stop();
  motv.operate(motv.DISABLE);
#endif
  Serial.println("Preparing");
}

#define div_v 2
#define div_h 2

void loop()
{
#define cartesian
#ifdef cartesian
  // get command
  while(1) 
  {
    if (Serial.find('#') && Serial.find('#')) 
    {
      Serial.println("Starting");
      break;
    }
    digitalWrite(PC13, HIGH);
    delay(100);
  }

  // motor start
  digitalWrite(PC13, LOW);
  motv.operate(motv.ENABLE);
  moth.operate(moth.ENABLE);

  // >>>> travel forward
  motv.moveTo(12000, 12000/div_v, 8000/div_v, 12000/div_v); // solf vel = 8000
  moth.moveTo(12000, 12000/div_h, 8000/div_h, 12000/div_h);
  // 
  Serial.print("Vertical time:\t\t");   Serial.println(motv.timeout());
  Serial.print("Horizontal time:\t");   Serial.println(moth.timeout());
  // 
  while(!motv.finished() || !moth.finished());
  delay(2000);

  // >>>> travel backward
  motv.moveTo(-12000, 12000/div_v, 8000/div_v, 12000/div_v);
  moth.moveTo(-12000, 12000/div_h, 8000/div_h, 12000/div_h);
  // 
  Serial.print("Vertical time:\t\t");   Serial.println(motv.timeout());
  Serial.print("Horizontal time:\t");   Serial.println(moth.timeout());
  // 
  while(!motv.finished() || !moth.finished());
  delay(2000);
  
  // motor stop
  motv.operate(motv.DISABLE);
  moth.operate(moth.DISABLE);
  Serial.println("\nReached\n");
#endif
}
