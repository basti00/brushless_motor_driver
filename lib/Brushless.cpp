//
// Created by uhlse on 04.08.2020.
//

#include "Arduino.h"
#include "Brushless.h"

HardwareTimer timer(2);

// check if timer is counting up or down
#define TIMER_DIRECTION ((TIMER2->regs).bas->CR1 & 1<<4)
#define HZ_TO_US(f) (1000000/f)
#define PI_THIRD 1.04719755f


//redeclaring the private static member variables
uint8_t volatile Brushless::hall1 = 0;
uint8_t volatile Brushless::hall2 = 0;
uint8_t volatile Brushless::hall3 = 0;
uint16_t volatile Brushless::hall_int_num = 0;
uint8_t volatile Brushless::hall_sektor = 0;
uint16_t volatile Brushless::hall_rotations = 0;
uint8_t volatile Brushless::hall_old_sektor = -1;
volatile SVM_vector Brushless::vx_ = SVM_V0;
volatile SVM_vector Brushless::vy_ = SVM_V0;


Brushless::Brushless()
{
  //debug
  pinMode(PC13, OUTPUT);
  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);
  pinMode(PB0, OUTPUT);

  // Output
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  digitalWrite(ENA, 0);
  digitalWrite(ENB, 0);
  digitalWrite(ENC, 0);

  // Input
  pinMode(HAL1, INPUT_PULLUP);
  pinMode(HAL2, INPUT_PULLUP);
  pinMode(HAL3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HAL1), Hall1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL2), Hall2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL3), Hall3_ISR, CHANGE);
}

void Brushless::setupPWMTimer(uint32_t f){
  // Pause the timer while we're configuring it
  timer.pause();
  t_ = HZ_TO_US(f);
  timer.setPeriod(HZ_TO_US(f));
  timer.setMode(1, TIMER_OUTPUT_COMPARE);
  timer.setMode(2, TIMER_OUTPUT_COMPARE);
  timer.setMode(3, TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, timer.getOverflow());
  timer.setCompare(TIMER_CH2, timer.getOverflow());
  timer.setCompare(TIMER_CH3, timer.getOverflow());
  (TIMER2->regs).bas->CR1 |= 0b11<<5; //Set to center aligned mode, generate INT at up & down part
  timer.attachInterrupt(1, handler_pwm1);
  timer.attachInterrupt(2, handler_pwm2);
  timer.attachInterrupt(3, handler_pwm3);
  timer.attachInterrupt(0, handler_overflow);
  //to disable jitter due to systick interrupts, delay() does not work though
  //systick_disable();
  //alternatively you could set interrupt priority
  nvic_irq_set_priority(NVIC_TIMER2, 0);     //highest priority for Timer2
  timer.refresh();
  timer.resume();
}

uint16_t Brushless::setDutycyle(float dutycycle){
  int comp = (timer.getOverflow()*dutycycle)/100;
  timer.setCompare(TIMER_CH1, comp);
  timer.setCompare(TIMER_CH2, comp/2);
  timer.setCompare(TIMER_CH3, comp/3);
  Serial.print(comp);
  return comp;
}

uint16_t Brushless::setPosition(float m_param, float angle_raw){
  float angle = fMod(angle_raw, 2*PI);
  alpha_ = fMod(angle_raw, PI_THIRD);
  m_ = m_param;

  //// todo probably noInterrupt(); needed
  if(angle < PI_THIRD){
    vx_ = SVM_V1;
    vy_ = SVM_V3;
  } else if(angle < 2*PI_THIRD){
    vx_ = SVM_V3;
    vy_ = SVM_V2;
  } else if(angle < 3*PI_THIRD){
    vx_ = SVM_V2;
    vy_ = SVM_V6;
  } else if(angle < 4*PI_THIRD){
    vx_ = SVM_V6;
    vy_ = SVM_V4;
  } else if(angle < 5*PI_THIRD){
    vx_ = SVM_V4;
    vy_ = SVM_V5;
  } else {
    vx_ = SVM_V5;
    vy_ = SVM_V1;
  }
  t1_ = t_ * m_ * sin((PI_THIRD - alpha_));
  t2_ = t_ * m_ * sin(alpha_);
  t0_ = t_ - t1_ - t2_;
  float t0_half = t0_/2;

  float ovrflw = timer.getOverflow();
  OCR1_ = ovrflw * t0_half / t_;
  OCR2_ = OCR1_ + ovrflw * t1_ / t_;
  OCR3_ = OCR2_ + ovrflw * t2_ / t_;

  timer.setCompare(TIMER_CH1, OCR1_);
  timer.setCompare(TIMER_CH2, OCR2_);
  timer.setCompare(TIMER_CH3, OCR3_);

  GPIOB->regs->BSRR = ENABLE_ALL;

  return 0;
}

#define ERR 10

int st = 0;

String Brushless::getInfo(){
  return ""
         + dash("m",m_)
         + dash("alpha",alpha_)
         //+ dash("t", (int) t_)
         + dash("vx", vx_)
         + dash("vy", vy_)
         //+ dash("bin_vx", SVM_HW_pins[vx_])
         //+ dash("bin_vy", SVM_HW_pins[vy_])
         + dash("OCR1", OCR1_)
         + dash("OCR2", OCR2_)
         + dash("OCR3", OCR3_)
         + dash("t0", t0_)
         + dash("t1", t1_)
         + dash("t2", t2_);
}


volatile int pwm_pin = INA;
volatile uint8_t pwm_st = 0;

void Brushless::handler_sections(void) {
  //digitalWrite(PC13, st & 0x01);
  noInterrupts();
  st = (hall_sektor+1)%6;
  GPIOB->regs->BRR =  0b0000000111111000; //disable all
  switch(st){
    case 0:
      // IN1 low
      pwm_pin = IN_B_;
      GPIOB->regs->BSRR =  0b0000000101000000 | (pwm_st << pwm_pin);
      break;
    case 1:
      // IN3 low
      pwm_pin = IN_B_;
      GPIOB->regs->BSRR =  0b0000000001010000 | (pwm_st << pwm_pin);
      break;
    case 2:
      // IN3 low
      pwm_pin = IN_A_;
      GPIOB->regs->BSRR =  0b0000000100010000 | (pwm_st << pwm_pin);
      break;
    case 3:
      // IN2 low
      pwm_pin = IN_A_;
      GPIOB->regs->BSRR =  0b0000000101000000 | (pwm_st << pwm_pin);
      break;
    case 4:
      // IN2 low
      pwm_pin = IN_C_;
      GPIOB->regs->BSRR =  0b0000000001010000 | (pwm_st << pwm_pin);
      break;
    case 5:
      // IN1 low
      pwm_pin = IN_C_;
      GPIOB->regs->BSRR =  0b0000000100010000 | (pwm_st << pwm_pin);
      break;
  }
  interrupts();
}

void Brushless::calc_rotation(){
  hall_int_num++;
  if(hall1){
    if(hall2){
      if(hall3) //111
      {
        hall_sektor = ERR;
        return;
      }
      else //110
        hall_sektor = 2;
    }else{
      if(hall3) //101
        hall_sektor = 4;
      else // 100
        hall_sektor = 3;
    }
  }else{
    if(hall2){
      if(hall3) //011
        hall_sektor = 0;
      else //010
        hall_sektor = 1;
    }else{
      if(hall3) //001
        hall_sektor = 5;
      else // 000
      {
        hall_sektor = ERR+1;
        return;
      }
    }
  }
  if(hall_sektor != hall_old_sektor){
    handler_sections();  //// advancing commutation sector

    if(hall_sektor==0 && hall_old_sektor==5)
      hall_rotations++;
    if(hall_sektor==5 && hall_old_sektor==0)
      hall_rotations--;
  }
  hall_old_sektor = hall_sektor;
}

void Brushless::Hall1_ISR()
{
  if(digitalRead(HAL1) == LOW)
    hall1 = 0;
  else
    hall1 = 1;
  calc_rotation();
}
void Brushless::Hall2_ISR()
{
  if(digitalRead(HAL2) == LOW)
    hall2 = 0;
  else
    hall2 = 1;
  calc_rotation();
}
void Brushless::Hall3_ISR()
{
  if(digitalRead(HAL3) == LOW)
    hall3 = 0;
  else
    hall3 = 1;
  calc_rotation();
}


void Brushless::handler_pwm_low() {
  //Clear C13 (LOW)
  GPIOB->regs->BRR = 1<<pwm_pin; //lower 16 bits
  GPIOC->regs->BRR = 0b0010000000000000; //PC13
  pwm_st = 1;
}

void Brushless::handler_pwm_high() {
  //Set C13 (HIGH)
  GPIOB->regs->BSRR = 1<<pwm_pin; //lower 16 bits
  GPIOC->regs->BSRR = 0b0010000000000000; //PC13
  pwm_st = 0;
}

String dash(String name, int val){
  return (name + " " + String(val) + "\t");
}
String dash(String name, float val){
  return (name + " " + String(val) + "\t");
}
String dash(String name, String val){
  return (name + " " + val + "\t");
}

float fMod(float a, float b)
{
  float mod;
  if (a < 0)
    mod = -a;
  else
    mod =  a;
  if (b < 0)
    b = -b;
  mod -= ((int)(mod/b))*b;
  if (a < 0)
    return -mod;
  return mod;
}

/*
 * find difference in these bit-patterns, then check
 * which bits to set or reset.
 */
void setOutputStage(SVM_vector vector){
  GPIOB->regs->BSRR = SVM_HW_pins[SVM_V7] & SVM_HW_pins[vector];    //set
  GPIOB->regs->BRR = SVM_HW_pins[SVM_V7] & (~SVM_HW_pins[vector]);  //reset
}

void Brushless::handler_pwm1(void) {
  if(!TIMER_DIRECTION)
    //v0 to vx
    setOutputStage(vx_); // set vx
  else
    //vx to v0
    setOutputStage(SVM_V0); // set all low
}

void Brushless::handler_pwm2(void) {
  if(!TIMER_DIRECTION)
    //vx to vy
    setOutputStage(vy_);
  else
    //vy to vx
    setOutputStage(vx_);
}

void Brushless::handler_pwm3(void) {
  if(!TIMER_DIRECTION)
    //vy to v7
    setOutputStage(SVM_V7);
  else
    //v7 to vy
    setOutputStage(vy_);
}

void Brushless::handler_overflow(void) {
  if(TIMER_DIRECTION)
    GPIOB->regs->BRR = 0b0000000000000001;
  else
    GPIOB->regs->BSRR = 0b0000000000000001;
}