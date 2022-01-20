#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  2

#define RC_CH1  0
#define RC_CH2  1

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
const byte ANV = 5;
const byte IN1 = 2;
const byte IN2 = 3;
volatile unsigned long count1 = 0;
volatile unsigned long count2 = 0;
void setup() 
{
  Serial.begin(SERIAL_PORT_SPEED);
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ANV,1);
  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  
}

void loop() 
{
  rc_read_values();

  Serial.print(rc_values[RC_CH1]); Serial.print(","); Serial.println(rc_values[RC_CH2]);
}
void rc_read_values() 
{
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) 
{
  if (digitalRead(input_pin) == HIGH) 
  {
    rc_start[channel] = micros();
  } 
  else 
  {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    if(rc_compare > 1002)
    {
      rc_values[RC_CH2] = 0;
    }
    rc_shared[channel] = rc_compare;
  }
}
void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
