#define RATE 1000000    // in microseconds

#define EN1 PB8
#define IN1 PB7
#define IN1_ 7
#define EN2 PB6
#define IN2 PB5
#define IN2_ 5
#define EN3 PB4
#define IN3 PB3
#define IN3_ 3

#define HAL1 PA8
#define HAL2 PB13
#define HAL3 PA9

// use timer 2
HardwareTimer timer(2);

int hz_to_us(int f)
{
  return 1000000/f;
}


void setup() {
  // Set up the LED to blink
  pinMode(PC13, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  digitalWrite(EN1, 0);
  digitalWrite(EN2, 0);
  digitalWrite(EN3, 0);

  pinMode(HAL1, INPUT_PULLUP);
  pinMode(HAL2, INPUT_PULLUP);
  pinMode(HAL3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HAL1), Hall1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL2), Hall2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL3), Hall3_ISR, CHANGE);


  Serial.begin(115200);

  //setting up switching PWM
  timer.pause();
  timer.setPeriod(hz_to_us(20000));
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


int avg_adc(int pin, int n){
  long sum = 0;
  for(int i = 0; i<n; i++){
    sum += analogRead(pin);
  }
  return sum/n;
}

void setDutycyle(float dutycycle){
  int comp = (timer.getOverflow()*dutycycle)/100;
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
  Serial.print(comp);
}


//// HALL Sensor start -------

uint8_t volatile hall1 = 0;
uint8_t volatile hall2 = 0;
uint8_t volatile hall3 = 0;
uint16_t volatile hall_int_num = 0;
uint8_t volatile hall_sektor = 0;
uint16_t volatile hall_rotations = 0;
uint8_t volatile hall_old_sektor = -1;
#define ERR 10

void calc_rotation(){
  hall_int_num++;
  if(hall1){
    if(hall2){
      if(hall3) //111
        hall_sektor = ERR;
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
        hall_sektor = ERR+1;
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

void Hall1_ISR()
{
  if(digitalRead(HAL1) == LOW)
    hall1 = 0;
  else
    hall1 = 1;
  calc_rotation();
}
void Hall2_ISR()
{
  if(digitalRead(HAL2) == LOW)
    hall2 = 0;
  else
    hall2 = 1;
  calc_rotation();
}
void Hall3_ISR()
{
  if(digitalRead(HAL3) == LOW)
    hall3 = 0;
  else
    hall3 = 1;
  calc_rotation();
}

//// HALL Sensor end -------


int hz;
int st = 0;

void loop() {
  float duty = avg_adc(PA0, 40)/40.96;

  Serial.print(IN1_);
  Serial.print(" ");
  Serial.print(IN2_);
  Serial.print(" ");
  Serial.print(IN3_);
  Serial.print(" ");
  Serial.print("overflow ");
  Serial.print(timer.getOverflow());
  Serial.print("\t duty ");
  Serial.print(duty);

  Serial.print("\t rot ");
  Serial.print(hall_rotations);
  Serial.print("\t hall");
  Serial.print(hall_sektor);
  Serial.print("\t sektor");
  Serial.print(st);

  Serial.print("\t interrupts");
  Serial.print(hall_int_num);
  hall_int_num=0;

  Serial.print("\t compare ");
  setDutycyle(duty);
  Serial.println("\t");

  delay(10);
}

volatile int pwm_pin = IN1;

void handler_sections(void) {
  //digitalWrite(PC13, st & 0x01);
  st = (hall_sektor+1)%6;
  switch(st){
    case 0:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      digitalWrite(EN1, 1);
      digitalWrite(EN2, 1);
      pwm_pin = IN2_;
      break;
    case 1:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN2, 1);
      digitalWrite(IN3, 0);
      digitalWrite(EN2, 1);
      digitalWrite(EN3, 1);
      pwm_pin = IN2_;
      break;
    case 2:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 1);
      digitalWrite(IN3, 0);
      digitalWrite(EN1, 1);
      digitalWrite(EN3, 1);
      pwm_pin = IN1_;
      break;
    case 3:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      digitalWrite(EN1, 1);
      digitalWrite(EN2, 1);
      pwm_pin = IN1_;
      break;
    case 4:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN2, 0);
      digitalWrite(IN3, 1);
      digitalWrite(EN2, 1);
      digitalWrite(EN3, 1);
      pwm_pin = IN3_;
      break;
    case 5:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 0);
      digitalWrite(IN3, 1);
      digitalWrite(EN1, 1);
      digitalWrite(EN3, 1);
      pwm_pin = IN3_;
      break;
  }
}

void handler_pwm_low(void) {
  //Clear C13 (LOW)
  GPIOB->regs->BRR = 1<<pwm_pin; //lower 16 bits
  GPIOC->regs->BRR = 0b0010000000000000;
}
void handler_pwm_high(void) {
  //Set C13 (HIGH)
  GPIOB->regs->BSRR = 1<<pwm_pin; //lower 16 bits
  GPIOC->regs->BSRR = 0b0010000000000000;
}

