#include <Arduino.h>
#include <ESP32Servo.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <Ultrasonic.h>

#include <stdint.h>

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
}
*/

// TEST MOTEURS - MOTOREDUCTEURS x2 - LINE TRACKING SENSOR

// Motor 1
#define IN1 18
#define IN2 5
// Motor 2
#define IN3 17
#define IN4 16
// LINE TRACKING SENSOR
#define TR_LEFT 4
#define TR_CENTER 0
#define TR_RIGHT 2
uint8_t sensorValue[4]; // create a table with 4 values - unsigned 8 bits (0 to 255)

void setup()
{
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // LINE TRACKING
  pinMode(TR_LEFT, INPUT);
  pinMode(TR_CENTER, INPUT);
  pinMode(TR_RIGHT, INPUT);
}

void run()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print("RUN RUN");
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.print("STOP");
}

void move_back()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.print("BACK");
}

void turn_right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.print("TURN RIGHT");
}

void turn_left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print("TURN LEFT");
}

void loop()
{
  run();
  delay(1000);
  stop();
  delay(2500);
  /*turn_right();
  delay(5000);
  move_back();
  delay(3000);*/

  // TEST LINE-TRACKING READ DATA
  sensorValue[0] = digitalRead(TR_LEFT);
  sensorValue[1] = digitalRead(TR_CENTER);
  sensorValue[2] = digitalRead(TR_RIGHT);
  sensorValue[3] = sensorValue[0] << 2 | sensorValue[1] << 1 | sensorValue[2];
  Serial.print("Sensor Value (L / C / R / ALL) : ");
  for (int i = 0; i < 4; i++)
  {
    Serial.print(sensorValue[i]);
    Serial.print('\t');
  }
}

/*// TEST SCANNING SG90 WITH SENSOR US INSTALL
#define PIN_SERVO1 23
#define PIN_ECHO 26
#define PIN_TRIG 27

Servo SERVO1;
Ultrasonic Ultrason(PIN_ECHO, PIN_TRIG);

int pos = 0;
int dist;

void setup()
{
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  Serial.begin(9600);

  SERVO1.attach(PIN_SERVO1);
}

void scaning()
{
  // Changer la position de 0º à 180º, en intervalles de 25 ms
  for (pos = 20; pos <= 160; pos += 1)
  {
    SERVO1.write(pos);
    delay(10);
  }

  // Revenir de 180º à 0º, avec des pauses de 25 ms
  for (pos = 160; pos >= 20; pos -= 1)
  {
    SERVO1.write(pos);
    delay(10);
  }
}

void scan_ultrason()
{
  // Pass INC as a parameter to get the distance in inches
  dist = Ultrason.read();

  Serial.print("Distance in CM: ");
  Serial.println(dist);
  // delay(1000);
}

void loop()
{
  scaning();
  scan_ultrason();
}
*/

// TEST AGAIN WITH SONAR
/**********************************************************************
 * Product     : Freenove 4WD Car for UNO
 * Description : Ultrasonic ranging and servo.
 * Auther      : www.freenove.com
 * Modification: 2019/08/05
 **********************************************************************/
/*#define PIN_SERVO 23 // define servo pin

#define PIN_SONIC_TRIG 27 // define Trig pin
#define PIN_SONIC_ECHO 26 // define Echo pin

#define MAX_DISTANCE 300                  // cm
#define SONIC_TIMEOUT (MAX_DISTANCE * 60) // calculate timeout
#define SOUND_VELOCITY 340                // soundVelocity: 340m/s

Servo servo;          // create servo object
byte servoOffset = 0; // change the value to Calibrate servo
uint8_t distance[4];  // define an arry with type u8(same to unsigned char)
int pos = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_SONIC_TRIG, OUTPUT); // set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT);  // set echoPin to input mode
  servo.attach(PIN_SERVO);         // initialize servo
  servo.write(90 + servoOffset);   // change servoOffset to Calibrate servo
}

float getSonar()
{
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
  else
    distance = MAX_DISTANCE;
  return distance; // return the distance value
}

void loop()
{
  for (pos = 20; pos <= 160; pos += 1)
  {
    servo.write(pos);
    distance[0] = getSonar();
    distance[1] = getSonar();
    distance[2] = getSonar();
    distance[3] = getSonar();
    delay(10);
  }

  // Revenir de 180º à 0º, avec des pauses de 25 ms
  for (pos = 160; pos >= 20; pos -= 1)
  {
    servo.write(pos);
    distance[0] = getSonar();
    distance[1] = getSonar();
    distance[2] = getSonar();
    distance[3] = getSonar();
    delay(10);
  }

  /*servo.write(45);
  delay(1000);
  distance[0] = getSonar(); // get ultrasonic value and save it into distance[0]

  servo.write(90);
  delay(1000);
  distance[1] = getSonar();

  servo.write(135);
  delay(1000);
  distance[2] = getSonar();

  servo.write(90);
  delay(1000);
  distance[3] = getSonar();

  Serial.print("Distance L / M / R / M2:   "); // Left/Middle/Right/Middle2
  for (int i = 0; i < 4; i++)
  {
    Serial.print(distance[i]); // print ultrasonic in 45°, 90°, 135°, 90°
    Serial.print("/");
  }
  Serial.print('\n'); // next content will be printed in new line
}*/

/*// NEW TEST DU 21/07/24
// Defines Tirg and Echo pins of the Ultrasonic Sensor
const int trigPin = 27;
const int echoPin = 26;
// Variables for the duration and the distance
long duration;
int distance;
Servo myServo; // Creates a servo object for controlling the servo motor
void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  Serial.begin(9600);
  myServo.attach(23); // Defines on which pin is the servo motor attached
}

// Function for calculating the distance measured by the Ultrasonic sensor
int calculateDistance()
{

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;
  return distance;
}

void loop()
{
  // rotates the servo motor from 15 to 165 degrees
  for (int i = 15; i <= 165; i++)
  {
    myServo.write(i);
    delay(5);
    distance = calculateDistance(); // Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree

    Serial.print(i);        // Sends the current degree into the Serial Port
    Serial.print(",");      // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    Serial.print(distance); // Sends the distance value into the Serial Port
    Serial.print(".");      // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
  }
  // Repeats the previous lines from 165 to 15 degrees
  for (int i = 165; i > 15; i--)
  {
    myServo.write(i);
    delay(30);
    distance = calculateDistance();
    Serial.print(i);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(".");
  }
}*/