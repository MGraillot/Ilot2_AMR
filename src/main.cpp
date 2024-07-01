#include <Arduino.h>
#include <ESP32Servo.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>

/* // TEST SENSOR IR TELECOMMANDE + COMMAND WITH AMR

#define PIN_IR 17
#define START_AMR 0xFFA25D     // Button ON-OFF
#define STOP_AMR 0xFFE21D      // Button MENU
#define PIN_SERVO01 16         // Servo scan
#define PIN_DIRECTION_RIGHT 15 // Motor

Servo SG90Servo;
IRrecv irrecv(PIN_IR);
decode_results results;

void setup()
{
  irrecv.enableIRIn();
  Serial.begin(9600);

  SG90Servo.attach(PIN_SERVO01);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
}

void loop()
{
  // ESSAIS IR REMOTE START/STOP ACTION

  if (irrecv.decode(&results))
  {
    Serial.println(results.value, HEX);
    if (results.value == START_AMR)
    {
      SG90Servo.write(150);
      delay(10);
    }
    else if (results.value == STOP_AMR)
    {
      SG90Servo.write(0);
      delay(10);
    }
    delay(1000);
    irrecv.resume();
  }
}*/

// TEST MOTEURS - MOTOREDUCTEURS x2

// Motor 1
#define IN1 18
#define IN2 5
// Motor 2
#define IN3 17
#define IN4 16

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void run()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void move_back()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turn_right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turn_left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void loop()
{
  run();
  delay(1000);
  stop();
  delay(2500);
  turn_right();
  delay(5000);
  move_back();
  delay(3000);
}
