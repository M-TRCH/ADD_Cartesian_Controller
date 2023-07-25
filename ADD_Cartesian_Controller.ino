
#include "AccelStepper.h"

AccelStepper mot1(PA4, PA5, PA6);

int callback1()
{
  mot1.callback();
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
  mot1.init(callback1);

  /* home */
  mot1.operate(mot1.ENABLE);  
  mot1.direction(mot1.DIR_CCW);
  while(digitalRead(PB0))
  {
    mot1.start(1000);  
    delay(1);
  }
  mot1.stop();      delay(500);
  mot1.direction(mot1.DIR_CW);
  mot1.start(500);  delay(300);  
  mot1.stop();
  mot1.operate(mot1.DISABLE);
  Serial.println("Started");
}

#define div_  1

void loop() 
{
  while(1) 
  {
    if (Serial.find('#') && Serial.find('#')) break;
    digitalWrite(PC13, LOW);
    delay(100);
  }
  
  digitalWrite(PC13, HIGH);
  mot1.operate(mot1.ENABLE);
 
  mot1.moveTo(12000, 12000/div_, 12000/div_, 12000/div_);
  Serial.print("Time: ");
  Serial.println(mot1.timeout());
  while(!mot1.finished());
  delay(2000);

  mot1.moveTo(-12000, 12000/div_, 12000/div_, 12000/div_);
  Serial.print("Time: ");
  Serial.println(mot1.timeout());
  while(!mot1.finished());
  delay(2000);
  
  mot1.operate(mot1.DISABLE);
  Serial.println("\nReached\n");
}
