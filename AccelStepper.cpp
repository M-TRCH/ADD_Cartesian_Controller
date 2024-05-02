
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
  callbackDisable();
  operate(DISABLE);
  setDir(DIR_CW);
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

void AccelStepper::setDir(bool dir)
{
  digitalWrite(DIR_PIN, dir);  
}
   
void AccelStepper::callbackEnable()
{
  CALLBACK_ACTIVATE = 1;
}

void AccelStepper::callbackDisable()
{
  CALLBACK_ACTIVATE = 0;
}

void AccelStepper::callback()
{
  // Callback activated for trajectory control, And disabled for endless control.
  if (CALLBACK_ACTIVATE)
  {
    // Reached to target position.
    if (PUL_CNT >= PUL_GOAL) 
    {
      POS_REACHED = 1;
      callbackDisable();
      PWM->pause();
    }
    // While traveling, do it...
    else
    {
      PUL_CNT++;    

      if (MOTION_STATE == 1)
      {
        // 1.1 Accelaration generate
        ACCEL_INC = JERK_GOAL / FREQ_CNT;
        ACCEL_CNT += ACCEL_INC;
        // 1.2 Move to next state, if the increase in accelaration is maximum.
        if (ACCEL_CNT >= ACCEL_GOAL)  MOTION_STATE = 2;
        // 1.3 Velocity generate
        FREQ_INC = ACCEL_CNT / FREQ_CNT;
        FREQ_CNT += FREQ_INC;
        // 1.4 Calculate the distance caused by the jerk.
        if (JERK_VEL == 0 && ACCEL_CNT > ACCEL_GOAL)   JERK_VEL = FREQ_CNT;
        // 1.5 Update timer frequency
        setFreq(FREQ_CNT);
      }
      else if (MOTION_STATE == 2)
      {
        // 2.1 Velocity generate
        FREQ_INC = ACCEL_CNT / FREQ_CNT;
        FREQ_CNT += FREQ_INC;     
        // 2.2 Move to next state, if the steady acceleration distance ends.
        if (FREQ_CNT >= FREQ_GOAL - JERK_VEL + FREQ_MIN)   MOTION_STATE = 3;  
        // 2.3 Update timer frequency
        setFreq(FREQ_CNT);
      }
      else if (MOTION_STATE == 3)
      {   
        // 3.1 Accelaration generate
        ACCEL_INC = JERK_GOAL / FREQ_CNT;
        ACCEL_CNT -= ACCEL_INC;
        // 3.2 Move to next state, if the decrease in accelaration is minimum.
        if (ACCEL_CNT <= 0)  
        {
         MOTION_STATE = 4;
         ACCEL_DIS = PUL_CNT;
        }
        // 3.3 Velocity generate
        FREQ_INC = ACCEL_CNT / FREQ_CNT;
        FREQ_CNT += FREQ_INC;
        // 3.4 Update timer frequency
        setFreq(FREQ_CNT); 
      }
      else if (MOTION_STATE == 4)
      {
        // 4.1 Move to next state, if traveling during the deceleration.
        if (PUL_CNT >= PUL_GOAL - ACCEL_DIS)    MOTION_STATE = 5;
      }
      else if (MOTION_STATE == 5)
      {
        // 5.1 Accelaration generate
        ACCEL_INC = JERK_GOAL / FREQ_CNT;
        ACCEL_CNT -= ACCEL_INC;
        // 5.2 Move to next state, if the decrease in accelaration is negative maximum.
        if (ACCEL_CNT <= -ACCEL_GOAL)  MOTION_STATE = 6;
        // 5.3 Velocity generate
        FREQ_INC = ACCEL_CNT / FREQ_CNT;
        FREQ_CNT += FREQ_INC;
        // 5.4 Update timer frequency
        setFreq(FREQ_CNT); 
      }
      else if (MOTION_STATE == 6)
      {
        // 6.1 Velocity generate
        FREQ_INC = ACCEL_CNT / FREQ_CNT;
        FREQ_CNT += FREQ_INC;     
        // 6.2 Move to next state, if the steady negative acceleration distance ends.
        if (FREQ_CNT <= JERK_VEL)    MOTION_STATE = 7;  
        // 6.3 Update timer frequency
        setFreq(FREQ_CNT);
      }
      else if (MOTION_STATE == 7)
      {
        // 7.1 Accelaration generate
        ACCEL_INC = JERK_GOAL / FREQ_CNT;
        ACCEL_CNT += ACCEL_INC;
        // 7.2 Velocity generate
        FREQ_INC = ACCEL_CNT / FREQ_CNT;
        FREQ_CNT += FREQ_INC; 
        // 7.3 Update timer frequency
        setFreq(FREQ_CNT); 
      }
    }
  }
}

unsigned long AccelStepper::calRunningTime(unsigned long pul, float freq)
{
  return 1000 * pul / freq;
}

unsigned long AccelStepper::calRunningTime(float u, float v, float accel)
{
  return 1000 * fabs(v-u) / accel;     
}

unsigned long AccelStepper::timeout()
{
  return TIMEOUT;
}

uint32_t AccelStepper::accelDistance()
{
  return ACCEL_DIS;  
}

void AccelStepper::stop()
{
  PWM->pause();
}

void AccelStepper::start()
{
  PWM->resume();
}

void AccelStepper::start(float freq)
{
  /* force start timer for driving in endless mode */
  // 1. initial variables
  FREQ_GOAL = abs(freq);
  CALLBACK_ACTIVATE = 0;
  
  // 2. timer starting
  setFreq(FREQ_GOAL);   
  PWM->resume();
}

uint8_t AccelStepper::finished()
{
  return POS_REACHED;
}

void AccelStepper::moveTo(int32_t pul, float freq, float accel, float jerk)
{
  // 1. variables reset 
  PUL_CNT = 0;
  FREQ_CNT = FREQ_MIN;  // Minimum speed cannot be zero. because limitation of the motor driver.
  ACCEL_CNT = 0;  
  ACCEL_DIS = 0;
  JERK_VEL = 0;   
  // 2. clear state 
  POS_REACHED = 0;
  ACCEL_REACHED = 0;
  JERK_REACHED[0] = 0;
  JERK_REACHED[1] = 0;
  JERK_REACHED[2] = 0;
  JERK_REACHED[3] = 0;
  MOTION_STATE = 1;
  // 3. activate controller 
  callbackEnable();
  ACCEL_ACTIVATE = accel>0? 1: 0;
  JERK_ACTIVATE = jerk>0? 1: 0;
  
  // 4. hardware setting
  if (pul > 0)  setDir(DIR_CW);
  else          setDir(DIR_CCW);
  
  // 5. variables initialize
  PUL_GOAL = abs(pul);
  FREQ_GOAL = fabs(freq);
  ACCEL_GOAL = fabs(accel);
  JERK_GOAL = fabs(jerk);
}

void AccelStepper::trajectoryGraphPrint()
{
//  Serial.print(MOTION_STATE);   Serial.print("\t");
//  Serial.print(millis());   Serial.print("\t");
  Serial.print(PUL_CNT);        Serial.print("\t");
  Serial.print(FREQ_CNT);       Serial.print("\t");
  Serial.print(ACCEL_CNT);      Serial.println("\t");
//  Serial.print(JERK_VEL);       Serial.print("\t");
//  Serial.print(ACCEL_DIS);      Serial.print("\t");  
}

void AccelStepper::simulate()
{
  float prd_sum = 0;
  while(!POS_REACHED)
  {
    callback();
    prd_sum += (1.0 / FREQ_CNT);
  }
  TIMEOUT = prd_sum * 1000.0; // convert ms to sec
}
