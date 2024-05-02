
/*
 *  System.h
 *  
 *  Created on: 12 Mar 2024
 *  Author: M.Teerachot
 *  Tested with STM32G431CB
 */

#ifndef SYSTEM_H
#define SYSTEM_H

/* (1) constant variables */
// 1. serial communication
#define SERIAL1_RXPIN_CH1   PA10
#define SERIAL1_TXPIN_CH1   PA9
#define SERIAL1_RXPIN_CH2   PB7
#define SERIAL1_TXPIN_CH2   PB6
// 2. constant value 
#define SERIAL1_CH1   11
#define SERIAL1_CH2   12
#define SERIAL1_UART  13
#define SERIAL1_RS232 14
// 3. hardware
#define TOG_ERR_STATE     -1
#define TOG_FLOAT_STATE   0
#define TOG_UP_STATE      1
#define TOG_DOWN_STATE    2
#define LED_ONBOARD_OFF   20
#define LED_ONBOARD_RED   21
#define LED_ONBOARD_GREEN 22

/* (2) hardware */
#define BUZZ_PIN          PC6
#define TOG_UP_PIN        PA2
#define TOG_DOWN_PIN      PA3
#define LEDR_ONBOARD_PIN  PA8
#define LEDG_ONBOARD_PIN  PA11
#define DETECT24V_PIN     PA0
#define DETECT24V         !digitalRead(DETECT24V_PIN)

void selectSerial1(int ch, int baud, int timeout)
{
  if (ch == SERIAL1_CH1)
  {
    pinMode(SERIAL1_RXPIN_CH2, INPUT);
    pinMode(SERIAL1_TXPIN_CH2, INPUT);
    Serial.setRx(SERIAL1_RXPIN_CH1);
    Serial.setTx(SERIAL1_TXPIN_CH1);
  }
  else if (ch == SERIAL1_CH2)
  {
    pinMode(SERIAL1_RXPIN_CH1, INPUT);
    pinMode(SERIAL1_TXPIN_CH1, INPUT);
    Serial.setRx(SERIAL1_RXPIN_CH2);
    Serial.setTx(SERIAL1_TXPIN_CH2);
  }
  Serial.begin(baud);
  Serial.setTimeout(timeout);
}

void selectSerial1(int interface)
{
  if (interface == SERIAL1_UART)
    selectSerial1(SERIAL1_CH1, 250000, 100);
  else if (interface == SERIAL1_RS232)
    selectSerial1(SERIAL1_CH2, 9600, 200);
}

void beep(int t=100, int t2=0)
{
  analogWrite(BUZZ_PIN, 10);
  delay(t);
  analogWrite(BUZZ_PIN, 0);
  delay(t2);
}

void setLedOnboard(int color, int t=0)
{
  switch (color)
  {
    case LED_ONBOARD_OFF:
      digitalWrite(LEDR_ONBOARD_PIN, 0);
      digitalWrite(LEDG_ONBOARD_PIN, 0);
      break;
    case LED_ONBOARD_RED:
      digitalWrite(LEDR_ONBOARD_PIN, 1);
      digitalWrite(LEDG_ONBOARD_PIN, 0);
      break;
    case LED_ONBOARD_GREEN:
      digitalWrite(LEDR_ONBOARD_PIN, 0);
      digitalWrite(LEDG_ONBOARD_PIN, 1);
      break;
  }
  delay(t);
}

int getToggle()
{
  int ret;
  int tog_up = digitalRead(TOG_UP_PIN);
  int tog_down = digitalRead(TOG_DOWN_PIN);
  if (!tog_up && tog_down)
  {
    ret = TOG_FLOAT_STATE;
    setLedOnboard(LED_ONBOARD_OFF);
  }
  else if (!tog_up && !tog_down) 
  {
    ret = TOG_UP_STATE;
    setLedOnboard(LED_ONBOARD_GREEN);
  }
  else if (tog_up && tog_down)    
  {
    ret = TOG_DOWN_STATE;
    setLedOnboard(LED_ONBOARD_RED);
  }
  else                           
  {
    ret = TOG_ERR_STATE;
    setLedOnboard(LED_ONBOARD_OFF);
  }
  return ret;
}

#endif
