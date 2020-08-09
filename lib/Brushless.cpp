//
// Created by uhlse on 04.08.2020.
//

#include "Arduino.h"
#include "Brushless.h"

HardwareTimer phase_pwm_timer(2);
HardwareTimer control_timer(3);

// check if timer is counting up or do wn
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

volatile SVM_phase Brushless::OCR1_phase = SVM_NONE;
volatile SVM_phase Brushless::OCR2_phase = SVM_NONE;
volatile SVM_phase Brushless::OCR3_phase = SVM_NONE;

float Brushless::m_=0;
float Brushless::alpha_=0;
float Brushless::t_=0;
float Brushless::t0_=0;
float Brushless::t1_=0;
float Brushless::t2_=0;
uint16_t Brushless::OCR1_=0;
uint16_t Brushless::OCR2_=0;
uint16_t Brushless::OCR3_=0;

volatile uint16_t Brushless::control_freq_ = 0;
volatile float Brushless::set_rps_ = 0;
volatile float Brushless::phi = 0;




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

void Brushless::setupPWMTimer(){
  // Pause the timer while we're configuring it

  // PWM-MODUL timer
  phase_pwm_timer.pause();
  t_ = HZ_TO_US(20000);
  phase_pwm_timer.setPeriod(t_);
  phase_pwm_timer.setMode(1, TIMER_OUTPUT_COMPARE);
  phase_pwm_timer.setMode(2, TIMER_OUTPUT_COMPARE);
  phase_pwm_timer.setMode(3, TIMER_OUTPUT_COMPARE);
  phase_pwm_timer.setCompare(TIMER_CH1, phase_pwm_timer.getOverflow());
  phase_pwm_timer.setCompare(TIMER_CH2, phase_pwm_timer.getOverflow());
  phase_pwm_timer.setCompare(TIMER_CH3, phase_pwm_timer.getOverflow());
  (TIMER2->regs).bas->CR1 |= 0b11<<5; //Set to center aligned mode, generate INT at up & down part
  phase_pwm_timer.attachInterrupt(1, handler_pwm1);
  phase_pwm_timer.attachInterrupt(2, handler_pwm2);
  phase_pwm_timer.attachInterrupt(3, handler_pwm3);
  phase_pwm_timer.attachInterrupt(0, handler_overflow);
  //to disable jitter due to systick interrupts, delay() does not work though
  //systick_disable();
  //alternatively you could set interrupt priority
  nvic_irq_set_priority(NVIC_TIMER2, 0);     //highest priority for Timer2

  phase_pwm_timer.refresh();
  phase_pwm_timer.resume();

  // control timer
  control_timer.pause();
  control_freq_ = 1000;
  control_timer.setPeriod(HZ_TO_US(control_freq_));
  control_timer.attachInterrupt(0, handler_control);

  control_timer.refresh();
  control_timer.resume();
}

uint16_t Brushless::setDutycyle(float dutycycle){
  /*int comp = (phase_pwm_timer.getOverflow()*dutycycle)/100;
  phase_pwm_timer.setCompare(TIMER_CH1, comp);
  phase_pwm_timer.setCompare(TIMER_CH2, comp/2);
  phase_pwm_timer.setCompare(TIMER_CH3, comp/3);
  Serial.print(comp);
  return comp;*/
}

uint16_t Brushless::setRPS(float rps)
{
  noInterrupts();
  set_rps_ = rps;
  interrupts();
}

uint16_t Brushless::setPosition(float m_param, float angle_raw){
  float angle = fMod(angle_raw, 2*PI);
  uint8_t sector;
  alpha_ = fMod(angle_raw, PI_THIRD);
  m_ = m_param;
  //// todo probably noInterrupt(); needed
  if(angle < PI_THIRD){
    vx_ = SVM_V1;
    vy_ = SVM_V3;
    sector = 0;
    OCR1_phase = SVM_A;
    OCR2_phase = SVM_B;
    OCR3_phase = SVM_C;
  } else if(angle < 2*PI_THIRD){
    vx_ = SVM_V3;
    vy_ = SVM_V2;
    sector = 1;
    OCR1_phase = SVM_B;
    OCR2_phase = SVM_A;
    OCR3_phase = SVM_C;
  } else if(angle < 3*PI_THIRD){
    vx_ = SVM_V2;
    vy_ = SVM_V6;
    sector = 2;
    OCR1_phase = SVM_B;
    OCR2_phase = SVM_C;
    OCR3_phase = SVM_A;
  } else if(angle < 4*PI_THIRD){
    vx_ = SVM_V6;
    vy_ = SVM_V4;
    sector = 3;
    OCR1_phase = SVM_C;
    OCR2_phase = SVM_B;
    OCR3_phase = SVM_A;
  } else if(angle < 5*PI_THIRD){
    vx_ = SVM_V4;
    vy_ = SVM_V5;
    sector = 4;
    OCR1_phase = SVM_C;
    OCR2_phase = SVM_A;
    OCR3_phase = SVM_B;
  } else {
    vx_ = SVM_V5;
    vy_ = SVM_V1;
    sector = 5;
    OCR1_phase = SVM_A;
    OCR2_phase = SVM_C;
    OCR3_phase = SVM_B;
  }
  t1_ = t_ * m_ * sin((PI_THIRD - alpha_));
  t2_ = t_ * m_ * sin(alpha_);
  t0_ = t_ - t1_ - t2_;
  float t0_half = t0_/2;

  if(sector % 2){
    float temp_t1 = t1_;
    t1_ = t2_;
    t2_ = temp_t1;
    SVM_vector temp_vx = vx_;
    vx_ = vy_;
    vy_ = temp_vx;
  }

  float ovrflw = phase_pwm_timer.getOverflow();
  OCR1_ = ovrflw * t0_half / t_;
  OCR2_ = OCR1_ + ovrflw * t1_ / t_;
  OCR3_ = OCR2_ + ovrflw * t2_ / t_;

  phase_pwm_timer.setCompare(TIMER_CH1, OCR1_);
  phase_pwm_timer.setCompare(TIMER_CH2, OCR2_);
  phase_pwm_timer.setCompare(TIMER_CH3, OCR3_);

  GPIOB->regs->BSRR = ENABLE_MASK;

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
         + dash("vy", vy_) //*/
         //+ dash("bin_vx", SVM_HW_pins[vx_])
         //+ dash("bin_vy", SVM_HW_pins[vy_])
         /*
         + dash("OCR1", OCR1_)
         + dash("OCR2", OCR2_)
         + dash("OCR3", OCR3_) //*/

         + dash("t0", t0_)
         + dash("t1", t1_)
         + dash("t2", t2_)// */
  ;
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
    //handler_sections();  //// advancing commutation sector

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

void inline setOutputPhase(SVM_phase phase){
  GPIOB->regs->BSRR = /*PHASE_MASK & */SVM_phase_pin[phase];
}
void inline resetOutputPhase(SVM_phase phase){
  GPIOB->regs->BRR = /*PHASE_MASK & */SVM_phase_pin[phase];
}

void Brushless::handler_pwm1(void) {
  if(TIMER_DIRECTION)
    resetOutputPhase(OCR1_phase);
  else
    setOutputPhase(OCR1_phase);
}

void Brushless::handler_pwm2(void) {
  if(TIMER_DIRECTION)
    resetOutputPhase(OCR2_phase);
  else
    setOutputPhase(OCR2_phase);
}

void Brushless::handler_pwm3(void) {
  if(TIMER_DIRECTION)
    resetOutputPhase(OCR3_phase);
  else
    setOutputPhase(OCR3_phase);
}

void Brushless::handler_overflow(void) {
  /*if(TIMER_DIRECTION)
    GPIOB->regs->BRR = 0b0000000000000001;
  else
    GPIOB->regs->BSRR = 0b0000000000000001; //*/

}

/*
 * 1000Hz = 1ms period
 */
void Brushless::handler_control(void) {

  bool static tog = 0;
  tog = !tog;
  if(tog)
    GPIOB->regs->BRR = 0b0000000000000001;
  else
    GPIOB->regs->BSRR = 0b0000000000000001;

  phi += (TWO_PI * set_rps_ / control_freq_);
  phi = fMod(phi, TWO_PI);
  setPosition(0.85, phi);
}