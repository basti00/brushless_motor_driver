#define RATE 1000000    // in microseconds

#define EN1 PB8
#define IN1 PB7
#define EN2 PB6
#define IN2 PB5
#define EN3 PB4
#define IN3 PB3

#define HAL1 PA8
#define HAL2 PB13
#define HAL3 PA9

// use timer 2
HardwareTimer timer(2);

void setup() {
  // Set up the LED to blink
  pinMode(PC13, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);

  pinMode(HAL1, INPUT_PULLUP);
  pinMode(HAL2, INPUT_PULLUP);
  pinMode(HAL3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HAL1), Hall1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL2), Hall2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HAL3), Hall3_ISR, CHANGE);


  Serial.begin(115200);

  // Pause the timer while we're configuring it
  timer.pause();
  // Set up period
  timer.setPeriod(RATE); // in microseconds
  // Set up an interrupt on channel 1
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(handler_sections);

  // Refresh the timer's count, prescale, and overflow
  timer.refresh();

  // Start the timer counting
  timer.resume();
}



int hz_to_us(int f)
{
  return 1000000/f;
}


int avg_adc(int pin, int n){
  long sum = 0;
  for(int i = 0; i<n; i++){
    sum += analogRead(pin);
  }
  return sum/n;
}

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
    handler_sections();
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

int hz;
int st = 0;

void loop() {
  timer.pause();
  //digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

  /*Serial.print("\t");
  Serial.print(digitalRead(HAL1));
  Serial.print("\t");
  Serial.print(digitalRead(HAL2));
  Serial.print("\t");
  Serial.print(digitalRead(HAL3));*/
  Serial.print(" |\trot ");
  Serial.print(hall_rotations);
  Serial.print("\t hall");
  Serial.print(hall_sektor);
  Serial.print("\t state");
  Serial.print(st);
  Serial.print("\t interrupts");
  Serial.print(hall_int_num);
  hall_int_num=0;
  Serial.println("\t");

  hz = map(avg_adc(PA0, 40), 0, 4096, 1, 700);

  timer.setPeriod(hz_to_us(hz));
  delay(10);

}


void handler_sections(void) {
  digitalWrite(PC13, st & 0x01);
  st = (hall_sektor+1)%6;
  switch(st){
    case 0:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      digitalWrite(EN1, 1);
      digitalWrite(EN2, 1);
      break;
    case 1:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN2, 1);
      digitalWrite(IN3, 0);
      digitalWrite(EN2, 1);
      digitalWrite(EN3, 1);
      break;
    case 2:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 1);
      digitalWrite(IN3, 0);
      digitalWrite(EN1, 1);
      digitalWrite(EN3, 1);
      break;
    case 3:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      digitalWrite(EN1, 1);
      digitalWrite(EN2, 1);
      break;
    case 4:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN2, 0);
      digitalWrite(IN3, 1);
      digitalWrite(EN2, 1);
      digitalWrite(EN3, 1);
      break;
    case 5:
      digitalWrite(EN1, 0); digitalWrite(EN2, 0); digitalWrite(EN3, 0);

      digitalWrite(IN1, 0);
      digitalWrite(IN3, 1);
      digitalWrite(EN1, 1);
      digitalWrite(EN3, 1);
      break;
  }
}
