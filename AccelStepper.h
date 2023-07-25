 /*
 *  AccelStepper.h
 *  
 *  Created on: 24 Jul 2023
 *  Author: m.teerachot
 *  Tested with F030C6/C8, F091RC and G431CB
 */

#ifndef ACCELSTEPPER_H
#define ACCELSTEPPER_H

#include <Arduino.h>

class AccelStepper
{
  private:
    /* physical pin */
    uint8_t ENA_PIN, DIR_PIN, PUL_PIN; 
    /* timer */
    TIM_TypeDef *INS;
    uint32_t CH;
    HardwareTimer *PWM;
    /* trajectory control */
    uint16_t FREQ_MIN = 2;                // can be changed as appropriate     
    uint32_t FREQ_MAX = 200000;           // variable use the hertz unit.
    // position
    volatile uint32_t PUL_CNT, PUL_GOAL;
    // velocity
    volatile float FREQ_CNT, FREQ_INC, FREQ_DEC;
    volatile uint32_t FREQ_GOAL;
    // acceleration
    volatile uint32_t ACCEL_DIS, DECEL_DIS, ACCEL_GOAL, DECEL_GOAL; 
    /* flag */
    bool POS_REACHED, ACCEL_REACHED, DECEL_REACHED; 
    bool ACCEL_ACTIVATE, DECEL_ACTIVATE, CALLBACK_ACTIVATE;
    /* feedback */
    unsigned long TIMEOUT;
  
  public:
    AccelStepper(uint8_t ena, uint8_t dir, uint8_t pul, bool ena_invert=false, bool dir_invert=false);
    ~AccelStepper();
    uint8_t DISABLE, ENABLE, DIR_CW, DIR_CCW;
    void pin(uint8_t ena, uint8_t dir, uint8_t pul);
    void invert(bool ena, bool dir);
    void init(int (*funcptr)());
    void setPrd(unsigned long t);
    void setPrd_ms(unsigned long t);
    void setFreq(float freq);
    void setFreq_k(float freq);
    void operate(bool ena, uint16_t t=500);
    void direction(bool dir);
    void callback_on();
    void callback_off();
    void callback();
    unsigned long forecastTime(unsigned long pul, float freq);
    unsigned long forecastTime(float u, float v, float accel);
    unsigned long timeout();
    void stop();
    void start(float freq);
    uint8_t finished();
    void moveTo(int32_t pul, float freq, float accel=-1, float decel=-1);
    
};

#endif /* ACCELSTEPPER_H */
