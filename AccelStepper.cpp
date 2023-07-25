
#include "AccelStepper.h"

AccelStepper::AccelStepper(uint8_t ena, uint8_t dir, uint8_t pul, bool ena_invert, bool dir_invert)
{
  pin(ena, dir, pul);
  invert(ena_invert, dir_invert);
}

AccelStepper::~AccelStepper()
{
  
}

void AccelStepper::pin(uint8_t ena, uint8_t dir, uint8_t pul)
{
  ENA_PIN = ena;
  DIR_PIN = dir;
  PUL_PIN = pul;
}

void AccelStepper::invert(bool ena, bool dir)
{
  ENABLE = ena? 1: 0;
  DIR_CW = dir? 1: 0;
  DISABLE = !ENABLE;
  DIR_CCW = !DIR_CW;
}

void AccelStepper::init(int (*funcptr)())
{
  /* pin config */
  pinMode(ENA_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  /* timer config */
  INS = (TIM_TypeDef*)pinmap_peripheral(digitalPinToPinName(PUL_PIN), PinMap_PWM);
  CH = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PUL_PIN), PinMap_PWM));
  PWM = new HardwareTimer(INS);
  PWM->setMode(CH, TIMER_OUTPUT_COMPARE_PWM1, PUL_PIN);   
  PWM->attachInterrupt(CH, funcptr); 
  /* state init */
  stop();
  callback_off();
  operate(DISABLE);
  direction(DIR_CW);
}

void AccelStepper::setPrd(unsigned long t)
{
  // set pwm period (us unit)
  // **remember that this function set duty cycle  constant at 50%
  PWM->setOverflow(t, MICROSEC_FORMAT);
  PWM->setCaptureCompare(CH, 50, PERCENT_COMPARE_FORMAT);
}

void AccelStepper::setPrd_ms(unsigned long t)
{
  // set pwm period (ms unit)
  setPrd(t*1000);
}

void AccelStepper::setFreq(float freq)
{
  // set pwm frequency (Hz unit)
  // **if you want to use this function alone, you must define callback_off()
  float t = 1000000.0 / fabs(freq);
  setPrd(t);
}

void AccelStepper::setFreq_k(float freq)
{
  // set pwm frequency (kHz unit)
  float t = 1000.0 / fabs(freq);
  setPrd(t);
}

void AccelStepper::operate(bool ena, uint16_t t)
{
  digitalWrite(ENA_PIN, ena);
  delay(t);
}

void AccelStepper::direction(bool dir)
{
  digitalWrite(DIR_PIN, dir);  
}
   
void AccelStepper::callback_on()
{
  // activate callback function
  CALLBACK_ACTIVATE = 1;
}

void AccelStepper::callback_off()
{
  // deactivate callback function
  CALLBACK_ACTIVATE = 0;
}

void AccelStepper::callback()
{
  // callback funcction is for trajectory control
  /* if the goal is reached */
  if (PUL_CNT >= PUL_GOAL && CALLBACK_ACTIVATE) 
  {
    POS_REACHED = 1;
    callback_off();
    PWM->pause();
  }
  /* if it's on travel, do it... */
  else
  {
    PUL_CNT++;    
    
    // accel and decel generator
    // there is a change in value for every pulse.
    if (ACCEL_ACTIVATE && !ACCEL_REACHED)
    {
      if (FREQ_CNT >= FREQ_GOAL)
        ACCEL_REACHED = 1;
      else
      {
        // calculate and increase velocity
        FREQ_INC = ACCEL_GOAL / FREQ_CNT;
        FREQ_CNT += FREQ_INC;
        setFreq(FREQ_CNT);
      }
    }
    else if (DECEL_ACTIVATE && !DECEL_REACHED && PUL_CNT >= PUL_GOAL - DECEL_DIS)
    {
      if (FREQ_CNT <= FREQ_MIN)
        DECEL_REACHED = 1;
      else
      {
        // calculate and decrease velocity
        FREQ_DEC = DECEL_GOAL / FREQ_CNT;
        FREQ_CNT -= FREQ_DEC;
        setFreq(FREQ_CNT);
      }
    }
  }
}

unsigned long AccelStepper::forecastTime(unsigned long pul, float freq)
{
  return 1000 * pul / freq;
}

unsigned long AccelStepper::forecastTime(float u, float v, float accel)
{
  return 1000 * fabs(v-u) / accel;     
}

unsigned long AccelStepper::timeout()
{
  return TIMEOUT;
}

void AccelStepper::stop()
{
  PWM->pause();
}

void AccelStepper::start(float freq)
{
  /* force start timer for driving in endless mode */
  // var init
  FREQ_GOAL = abs(freq);
  CALLBACK_ACTIVATE = 0;
  
  // timer action
  setFreq(FREQ_GOAL);   
  PWM->resume();
}

uint8_t AccelStepper::finished()
{
  return POS_REACHED;
}

void AccelStepper::moveTo(int32_t pul, float freq, float accel, float decel)
{
  /* var reset */
  PUL_CNT = 0;
  // **minimum speed cannot be zero. because limitation of the motor driver
  FREQ_CNT = FREQ_MIN;  
  // clear state
  POS_REACHED = 0;
  ACCEL_REACHED = 0;
  DECEL_REACHED = 0;
  // activate controller
  CALLBACK_ACTIVATE = 1;
  ACCEL_ACTIVATE = accel>0? 1: 0;
  DECEL_ACTIVATE = decel>0? 1: 0;

  /* hardware init */
  if (pul > 0)  direction(DIR_CW);
  else          direction(DIR_CCW);
  
  /* var init */
  PUL_GOAL = abs(pul);
  FREQ_GOAL = fabs(freq);
  // force velocity on position mode
  if (!ACCEL_ACTIVATE && !DECEL_ACTIVATE) setFreq(FREQ_GOAL);
  ACCEL_GOAL = fabs(accel);
  DECEL_GOAL = fabs(decel);
  ACCEL_DIS = (pow(FREQ_GOAL, 2) - pow(FREQ_MIN, 2)) / 2 / ACCEL_GOAL;
  DECEL_DIS = (pow(FREQ_GOAL, 2) - pow(FREQ_MIN, 2)) / 2 / DECEL_GOAL;
   
  /* prevent failure */
  TIMEOUT = forecastTime(PUL_GOAL, FREQ_GOAL);
  if (ACCEL_ACTIVATE) TIMEOUT += forecastTime(FREQ_MIN, FREQ_GOAL, ACCEL_GOAL);
  if (DECEL_ACTIVATE) TIMEOUT += forecastTime(FREQ_MIN, FREQ_GOAL, DECEL_GOAL);
  // set timeout factor = 105%
  TIMEOUT += TIMEOUT * 0.05;

  /* start timer */
  PWM->resume();
}
   
