//
// Created by uhlse on 04.08.2020.
//

#include "Arduino.h"
#include "Brushless.h"

// center aligned timer to control PWM module switching
HardwareTimer phase_pwm_timer(2);
// to execute periodic control procedures
HardwareTimer control_timer(3);
// timer to estimate the current rotor pos from hall sensor signals
HardwareTimer hall_timer(4);

// check if pwm-timer is counting up or do wn
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

int8_t volatile Brushless::hall_direction = 0;
uint8_t volatile Brushless::hall_timer_ovrflws = 0;
uint32_t volatile Brushless::hall_timer_predicted_ticks = 0;

volatile SVM_phase Brushless::OCR1_phase = SVM_NONE;
volatile SVM_phase Brushless::OCR2_phase = SVM_NONE;
volatile SVM_phase Brushless::OCR3_phase = SVM_NONE;

float Brushless::m_=0;
float Brushless::phi_=0;
float Brushless::alpha_=0;
float Brushless::t_=0;
float Brushless::t0_=0;
float Brushless::t1_=0;
float Brushless::t2_=0;
uint16_t Brushless::OCR1_=0;
uint16_t Brushless::OCR2_=0;
uint16_t Brushless::OCR3_=0;

volatile uint16_t Brushless::control_freq_ = 0;
volatile CNTRL_mode Brushless::cntrl_mode_ = CNTRL_staticVector;
volatile float Brushless::radians_per_sec_ = 0;


Brushless::Brushless()
{
  //debug
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
  attachInterrupt(digitalPinToInterrupt(HAL1), handler_hall1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL2), handler_hall2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL3), handler_hall3, CHANGE);
}

void Brushless::setupPWMTimer(){
  // Pause the timer while we're configuring it

  // PWM-module timer
  phase_pwm_timer.pause();
  t_ = HZ_TO_US(10000);
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

  // hall timer
  hall_timer.pause();
  hall_timer.setPrescaleFactor(64);
  hall_timer.setOverflow(65535);
  hall_timer.attachInterrupt(0, handler_hall_timer);

  hall_timer.refresh();
  hall_timer.resume();
}


void Brushless::setMagnitude(float mag){
  m_ = mag;
}

void Brushless::setVector(float mag, float angle) {
  cntrl_mode_ = CNTRL_staticVector;
  setAngle(angle);
  setMagnitude(mag);
}

void Brushless::setSpeed(float rad_per_sec)
{
  noInterrupts();
  cntrl_mode_ = CNTRL_continousRotation;
  radians_per_sec_ = rad_per_sec;
  interrupts();
}

void Brushless::setFixedAngleFromHall(){
  noInterrupts();
  cntrl_mode_ = CNTRL_fixedAngleFromHall;
  interrupts();
}


uint16_t Brushless::setAngle(float angle_raw){
  phi_ = fMod(angle_raw, TWO_PI);
  uint8_t sector;
  alpha_ = fMod(angle_raw, PI_THIRD);
  //// todo probably noInterrupt(); needed
  if(phi_ < PI_THIRD){  //todo exchange if-construct with bool logic
    //switching between v1 and v3 (001 and 011)
    sector = 0;
    OCR1_phase = SVM_A; // switch on first, switch off last
    OCR2_phase = SVM_B; // switch on second,
    OCR3_phase = SVM_C; // only switch on during v7
  } else if(phi_ < 2*PI_THIRD){
    //switching between v2 and v3 (010 and 011)
    sector = 1;
    OCR1_phase = SVM_B;
    OCR2_phase = SVM_A;
    OCR3_phase = SVM_C;
  } else if(phi_ < 3*PI_THIRD){
    //switching between v2 and v6 (010 and 110)
    sector = 2;
    OCR1_phase = SVM_B;
    OCR2_phase = SVM_C;
    OCR3_phase = SVM_A;
  } else if(phi_ < 4*PI_THIRD){
    //switching between v4 and v6 (100 and 110)
    sector = 3;
    OCR1_phase = SVM_C;
    OCR2_phase = SVM_B;
    OCR3_phase = SVM_A;
  } else if(phi_ < 5*PI_THIRD){
    //switching between v4 and v5 (100 and 101)
    sector = 4;
    OCR1_phase = SVM_C;
    OCR2_phase = SVM_A;
    OCR3_phase = SVM_B;
  } else {
    //switching between v1 and v5 (001 and 101)
    sector = 5;
    OCR1_phase = SVM_A;
    OCR2_phase = SVM_C;
    OCR3_phase = SVM_B;
  }

  /* t0_ is time to be on v0 and v7
   * t1 and t2 is time to be on the two vectors
   *
   */
  if(sector % 2){ //switch switching order every second sector to reduce switching losses
    t1_ = t_ * m_ * sin(alpha_);
    t2_ = t_ * m_ * sin((PI_THIRD - alpha_));
  }else{
    t1_ = t_ * m_ * sin((PI_THIRD - alpha_));
    t2_ = t_ * m_ * sin(alpha_);
  }
  t0_ = t_ - t1_ - t2_;

  float ovrflw = phase_pwm_timer.getOverflow();
  OCR1_ = ovrflw * (t0_/2) / t_;
  OCR2_ = OCR1_ + ovrflw * t1_ / t_;
  OCR3_ = OCR2_ + ovrflw * t2_ / t_;

  phase_pwm_timer.setCompare(TIMER_CH1, OCR1_);
  phase_pwm_timer.setCompare(TIMER_CH2, OCR2_);
  phase_pwm_timer.setCompare(TIMER_CH3, OCR3_);

  GPIOB->regs->BSRR = ENABLE_MASK;

  return 0;
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
  // to generate a trigger reference on oscilloscope
  if(TIMER_DIRECTION)
    GPIOB->regs->BRR = 0b0000000000000001;
  else
    GPIOB->regs->BSRR = 0b0000000000000001; //*/
}


////////// Hall sensor start

uint8_t Brushless::resolveSektor(){
  if(hall_sektor==1) return 0;
  if(hall_sektor==3) return 1;
  if(hall_sektor==2) return 2;
  if(hall_sektor==6) return 3;
  if(hall_sektor==4) return 4;
  if(hall_sektor==5) return 5;
}

float Brushless::getHallAngle(){
  uint8_t discrete_sector = resolveSektor();
  if(hall_timer_predicted_ticks == 0)
    return PI_THIRD * resolveSektor();
  float predicted_progeress = (((uint32_t)hall_timer_ovrflws-1)*65536
                 + hall_timer.getCount()) / (float)hall_timer_predicted_ticks; // 0..1
  if(predicted_progeress>1)
    predicted_progeress = 1;
  if(discrete_sector==0 && hall_direction==-1)
    discrete_sector = 6;
  return PI_THIRD * (discrete_sector + predicted_progeress * hall_direction);
}

void Brushless::proc_hall(){
  hall_int_num++;
  hall_sektor = (hall3<<2) |(hall2<<1) |(hall1);
  if(hall_sektor == 0b111 || hall_sektor == 0b000){
    hall_sektor = -1;
    return;
  }
  if(hall_sektor != hall_old_sektor){
    hall_timer.pause();
    hall_timer_predicted_ticks = (hall_timer_ovrflws-1)*65536 + hall_timer.getCount();
    hall_timer.setCount(0);
    hall_timer_ovrflws = 0;
    hall_timer.refresh();
    hall_timer.resume();

    hall_direction = -1;
    if((hall_old_sektor == 1 && hall_sektor == 3) ||
               (hall_old_sektor == 3 && hall_sektor == 2) ||
               (hall_old_sektor == 2 && hall_sektor == 6) ||
               (hall_old_sektor == 6 && hall_sektor == 4) ||
               (hall_old_sektor == 4 && hall_sektor == 5) ||
               (hall_old_sektor == 5 && hall_sektor == 1))
      hall_direction = 1;

    if(hall_sektor==1 && hall_old_sektor==5)
      hall_rotations++;
    if(hall_sektor==5 && hall_old_sektor==1)
      hall_rotations--;
    hall_old_sektor = hall_sektor;
  }
}

void Brushless::handler_hall1()
{
  if(digitalRead(HAL1) == LOW)
    hall1 = 0;
  else
    hall1 = 1;
  proc_hall();
}
void Brushless::handler_hall2()
{
  if(digitalRead(HAL2) == LOW)
    hall2 = 0;
  else
    hall2 = 1;
  proc_hall();
}
void Brushless::handler_hall3()
{
  if(digitalRead(HAL3) == LOW)
    hall3 = 0;
  else
    hall3 = 1;
  proc_hall();
}

float fsdif(float f1, float f2){
  if(f1>f2)
    return fMod(f1-f2, TWO_PI);
  else
    return TWO_PI - fMod(f1-f2, TWO_PI);
}

////////// Hall sensor end

String Brushless::getInfo(){
  float hall = getHallAngle();
  return ""
         //+ dash("m",m_)
         + dash("phi",phi_)
         // + dash("timer_count", hall_timer.getCount())
         // + dash("timer_overflws", hall_timer_ovrflws)
         // + dash("t_until_now", (((uint32_t)hall_timer_ovrflws-1)*65536
         //                               + hall_timer.getCount()))
         + dash("predicted_t", hall_timer_predicted_ticks)
         //+ dash("hall", resolveSektor())
         + dash("hall_angle", hall)
         + dash("diff", fsdif(hall,phi_))
    /* + dash("vx", vx_)
    + dash("vy", vy_) //*/
    //+ dash("bin_vx", SVM_HW_pins[vx_])
    //+ dash("bin_vy", SVM_HW_pins[vy_])
    /*
    + dash("OCR1", OCR1_)
    + dash("OCR2", OCR2_)
    + dash("OCR3", OCR3_) //*/

    /*+ dash("t0", t0_)
    + dash("t1", t1_)
    + dash("t2", t2_)// */
          ;
}

String dash(String name, uint32_t val){ return (name + " " + String(val) + "\t"); }
String dash(String name, uint16_t val){ return (name + " " + String(val) + "\t"); }
String dash(String name, uint8_t val){ return (name + " " + String(val) + "\t"); }
String dash(String name, int32_t val){ return (name + " " + String(val) + "\t"); }
String dash(String name, int16_t val){ return (name + " " + String(val) + "\t"); }
String dash(String name, int8_t val){ return (name + " " + String(val) + "\t"); }
String dash(String name, int val){ return (name + " " + String(val) + "\t"); }
String dash(String name, float val){ return (name + " " + String(val) + "\t"); }
String dash(String name, String val){ return (name + " " + val + "\t"); }

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

void Brushless::handler_control(void) {

  switch(cntrl_mode_ ){
    case CNTRL_fixedAngleFromHall:{
      float phi = getHallAngle() + 2.84;
      phi = fMod(phi, TWO_PI);
      setMagnitude(m_);
      setAngle(phi);
    }
      break;
    case CNTRL_continousRotation:{
      float static phi = 0;
      phi += ( radians_per_sec_ / control_freq_);
      phi = fMod(phi, TWO_PI);
      setMagnitude(m_);
      setAngle(phi);
    }
      break;
    case CNTRL_staticVector:
      break;
  }
}

void Brushless::handler_hall_timer() {
  uint8_t static ovrflws = 0;
  if(hall_timer_ovrflws<255)
    hall_timer_ovrflws++;
}