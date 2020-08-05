//
// Created by uhlse on 04.08.2020.
//

#include "Arduino.h"
#include "Brushless.h"

HardwareTimer timer(2);

//redeclaring the private static member variables
uint8_t volatile Brushless::hall1 = 0;
uint8_t volatile Brushless::hall2 = 0;
uint8_t volatile Brushless::hall3 = 0;
uint16_t volatile Brushless::hall_int_num = 0;
uint8_t volatile Brushless::hall_sektor = 0;
uint16_t volatile Brushless::hall_rotations = 0;
uint8_t volatile Brushless::hall_old_sektor = -1;


Brushless::Brushless()
{
  // Output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  digitalWrite(EN1, 0);
  digitalWrite(EN2, 0);
  digitalWrite(EN3, 0);
  // Input
  pinMode(HAL1, INPUT_PULLUP);
  pinMode(HAL2, INPUT_PULLUP);
  pinMode(HAL3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HAL1), Hall1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL2), Hall2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL3), Hall3_ISR, CHANGE);
}

void Brushless::setupTimer(){
  timer.pause();
  timer.setPeriod(hz_to_us(30000));
  timer.setMode(1, TIMER_OUTPUT_COMPARE);
  timer.setMode(2, TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, timer.getOverflow());  // Interrupt when count reaches
  timer.setCompare(TIMER_CH2, timer.getOverflow());  // Interrupt when count reaches 1
  timer.attachInterrupt(1, handler_pwm_high);
  timer.attachInterrupt(2, handler_pwm_low);
  //to disable jitter due to systick interrupts, delay() does not work though
  //systick_disable();
  //alternatively you could set interrupt priority
  nvic_irq_set_priority(NVIC_TIMER2, 0);     //highest priority for Timer2
  timer.refresh();
  timer.resume();
}

uint16_t Brushless::setDutycyle(float dutycycle){
  uint16_t comp = (timer.getOverflow()*dutycycle)/100;
  bool static detached = false;
  if(dutycycle > 99.5){
    timer.detachInterrupt(2);
    detached = true;
  }
  else if(comp < 50){
    timer.detachInterrupt(1);
    detached = true;
  }
  else{
    if(detached){
      timer.attachInterrupt(1, handler_pwm_high);
      timer.attachInterrupt(2, handler_pwm_low);
      detached = false;
    }
  }
  timer.setCompare(TIMER_CH2, comp);
  return comp;
}

#define ERR 10

int st = 0;

String Brushless::getInfo(){
  return "None";
}


volatile int pwm_pin = IN1;
volatile uint8_t pwm_st = 0;

void Brushless::handler_sections(void) {
  //digitalWrite(PC13, st & 0x01);
  noInterrupts();
  st = (hall_sektor+1)%6;
  GPIOB->regs->BRR =  0b0000000111111000; //disable all
  switch(st){
    case 0:
      // IN1 low
      pwm_pin = IN2_;
      GPIOB->regs->BSRR =  0b0000000101000000 | (pwm_st << pwm_pin);
      break;
    case 1:
      // IN3 low
      pwm_pin = IN2_;
      GPIOB->regs->BSRR =  0b0000000001010000 | (pwm_st << pwm_pin);
      break;
    case 2:
      // IN3 low
      pwm_pin = IN1_;
      GPIOB->regs->BSRR =  0b0000000100010000 | (pwm_st << pwm_pin);
      break;
    case 3:
      // IN2 low
      pwm_pin = IN1_;
      GPIOB->regs->BSRR =  0b0000000101000000 | (pwm_st << pwm_pin);
      break;
    case 4:
      // IN2 low
      pwm_pin = IN3_;
      GPIOB->regs->BSRR =  0b0000000001010000 | (pwm_st << pwm_pin);
      break;
    case 5:
      // IN1 low
      pwm_pin = IN3_;
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
  GPIOC->regs->BRR = 0b0010000000000000;
  pwm_st = 1;
}

void Brushless::handler_pwm_high() {
  //Set C13 (HIGH)
  GPIOB->regs->BSRR = 1<<pwm_pin; //lower 16 bits
  GPIOC->regs->BSRR = 0b0010000000000000;
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