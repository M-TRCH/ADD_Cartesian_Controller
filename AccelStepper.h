 /*
 *  AccelStepper.h
 *  
 *  Created on: 21 Feb 2024
 *  Author: M.Teerachot
 *  Tested with STM32G431CB
 */

#ifndef ACCELSTEPPER_H
#define ACCELSTEPPER_H

#include <Arduino.h>

class AccelStepper
{
  private:
    /* Physical pin, common stepper motor drives have three section controls: enable, direction, and pulse. */
    uint8_t ENA_PIN, DIR_PIN, PUL_PIN; 
    /* Timer, all about the timer is based on stm32 architecture. */
    TIM_TypeDef *INS;
    uint32_t CH;
    HardwareTimer *PWM;
    /* Trajectory control, point to point moition consists of position, velocity, accelaration and jerk. */
    /* constant values */
    uint16_t FREQ_MIN = 200;        // Can be changed to suit your system.     
    uint32_t FREQ_MAX = 200000;     // Variable use the hertz unit.
    /* CNT: counting value
     * INC: increase value
     * GOAL: set target position 
     * DIS: calculation distance
     */ 
    volatile uint8_t MOTION_STATE;
    /* 1. position */
    volatile uint32_t PUL_CNT; 
    volatile uint32_t PUL_GOAL;
    /* 2. velocity */
    volatile float FREQ_CNT, FREQ_INC;
    volatile uint32_t FREQ_GOAL;
    /* 3. acceleration */
    volatile float ACCEL_CNT, ACCEL_INC;
    volatile int32_t ACCEL_GOAL; 
    volatile uint32_t ACCEL_DIS;    // Total distance of states 1, 2, and 3 (of 7).
    /* 4. jerk */ 
    volatile uint32_t JERK_GOAL;  
    volatile uint32_t JERK_VEL;     // The value of velocity at any jerk value.
    
    /* flag variables */
    bool POS_REACHED, ACCEL_REACHED, DECEL_REACHED, JERK_REACHED[4]; 
    bool ACCEL_ACTIVATE, JERK_ACTIVATE;
    bool CALLBACK_ACTIVATE; 
    /* feedback variables */
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
    void operate(bool ena, uint16_t t=0);
    void setDir(bool dir);
    void callbackEnable();
    void callbackDisable();
    void callback();
    unsigned long calRunningTime(unsigned long pul, float freq);
    unsigned long calRunningTime(float u, float v, float accel);
    unsigned long timeout();
    uint32_t accelDistance();
    void stop();
    void start();
    void start(float freq);
    uint8_t finished();
    void moveTo(int32_t pul, float freq, float accel, float jerk);
    void trajectoryGraphPrint();
    void simulate();
};

#endif /* ACCELSTEPPER_H */
