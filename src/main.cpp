/*
Speed measuring uses high frequency hardware timer 1Hz == 1ms) to measure the time from of one rotation, in ms
*/

// These define's must be placed at the beginning before #include "megaAVR_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#include <Arduino.h>

#define TIMER_INTERRUPT_DEBUG 2
#define _TIMERINTERRUPT_LOGLEVEL_ 0
#define USING_16MHZ true // ATMega4809
#define USING_8MHZ false
#define USING_250KHZ false

#define USE_TIMER_0 false
#define USE_TIMER_1 true // enable ITimer1
#define USE_TIMER_2 false
#define USE_TIMER_3 false

#define TIMER1_INTERVAL_MS 100 // 1s = 1000ms

#include "TimerInterrupt_Generic.h"

// Define left motor
const byte leftPWM = 5; // make sure it is a PWM pin
const byte leftIn1 = A0;
const byte leftIn2 = A1;
// Define right motor
const byte rightPWM = 6;
const byte rightIn1 = A2;
const byte rightIn2 = A3;

// Define encoder pins
const byte leftEncA = 2;  // D2 reads left encoder's Ch.A
const byte leftEncB = 3;  // D3 reads left encoder's Ch.B
const byte rightEncA = 8; // D8 reads right encoder's Ch.A
const byte rightEncB = 9; // D9 reads right encoder's Ch.B
                          // const byte leftVcc = A5;  // A5 serves as Vcc for left encoder
                          // const byte rightVcc = A6;  // A6 serves as Vcc for right encoder

// Initialize encoder pin states
int8_t motorDir = 0; // looking from motor shaft, clockwise: 1, ccw: -1
uint32_t counter_left = 0;
uint32_t counter_right = 0;
float leftCPS = 0.0;
float rightCPS = 0.0;

void TimerHandler1(void)
{
  leftCPS = counter_left * TIMER1_INTERVAL_MS / 1000;
  rightCPS = counter_right * TIMER1_INTERVAL_MS / 1000;
  if (TIMER_INTERRUPT_DEBUG > 1)
  {
    Serial.print(leftCPS);
    Serial.print(",");
    Serial.println(rightCPS);
  }
  counter_left = 0;
  counter_right = 0;
}

void ISR_countLeftEncA(void)
{
  counter_left++;
}

void ISR_countLeftEncB(void)
{
  counter_left++;
}

void ISR_countRightEncA(void)
{
  counter_right++;
}

void ISR_countRightEncB(void)
{
  counter_right++;
}

void setup()
{
  // Set all the motor control pins to outputs
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(leftIn1, OUTPUT);
  pinMode(leftIn2, OUTPUT);
  pinMode(rightIn1, OUTPUT);
  pinMode(rightIn2, OUTPUT);
  // Set all encoder pins to inputs
  pinMode(leftEncA, INPUT);
  pinMode(leftEncB, INPUT);
  pinMode(rightEncA, INPUT);
  pinMode(rightEncB, INPUT);

  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.print(F("\nStarting ISR_RPM_Measure on "));
  Serial.println(BOARD_NAME);
  Serial.println(MEGA_AVR_TIMER_INTERRUPT_VERSION);
  Serial.println(TIMER_INTERRUPT_GENERIC_VERSION);
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
  Serial.print(F("TCB Clock Frequency = "));

  ITimer1.init(); // set timer for 1sec
  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = "));
    Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  // Assumming the interruptPin will go LOW
  attachInterrupt(digitalPinToInterrupt(leftEncA), ISR_countLeftEncA, CHANGE); // Increase left counter A when speed sensor pin changes
  attachInterrupt(digitalPinToInterrupt(leftEncB), ISR_countLeftEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncA), ISR_countRightEncA, CHANGE); // Increase left counter A when speed sensor pin changes
  attachInterrupt(digitalPinToInterrupt(rightEncB), ISR_countRightEncB, CHANGE);

  // Stop motor
  digitalWrite(leftIn1, LOW);
  digitalWrite(leftIn2, LOW);
  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, LOW);
  delay(1000);

  // Initialize counter
  counter_left = 0;
  counter_right = 0;
}

void loop()
{
  // Set motor directions
  digitalWrite(leftIn1, LOW);
  digitalWrite(leftIn2, HIGH);
  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, HIGH);

  analogWrite(leftPWM, 200);
  analogWrite(rightPWM, 200);
  delay(10);
}