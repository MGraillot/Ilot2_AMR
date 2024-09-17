#include <Arduino.h>
#include <ESP32Servo.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <Ultrasonic.h>

#include <stdint.h>

// TEST MOTEURS - MOTOREDUCTEURS x2 - LINE TRACKING SENSOR

// Motor 1
#define IN1 34 //18
#define IN2 35 //5
// Motor 2
#define IN3 32 //17
#define IN4 33 //16
// ULTRASON + SG90
#define ULTRASONIC_TRIG_PIN 22 // Broche de déclenchement du capteur ultrason
#define ULTRASONIC_ECHO_PIN 23 // Broche d'écho du capteur ultrason
#define SERVO_PIN 21           // Broche de contrôle du servo-moteur

// Constantes pour le fonctionnement du robot
#define OBSTACLE_DISTANCE 20  // Distance (en cm) à partir de laquelle on considère un obstacle
#define SERVO_SCAN_DELAY 15   // Délai (en ms) entre chaque pas du servo lors du scan

#define ENA 27 // Broche PWM pour le moteur 1
#define ENB 14 // Broche PWM pour le moteur 2

// Capteurs de suivi de ligne
#define LEFT_SENSOR_PIN 18  // Capteur centre (utilisé comme gauche)
#define RIGHT_SENSOR_PIN 5  // Capteur droit

// Définir les canaux PWM pour les moteurs
const int PWM_CHANNEL_A = 0; // Canal PWM pour ENA (moteur 1)
const int PWM_CHANNEL_B = 1; // Canal PWM pour ENB (moteur 2)

const int PWM_FREQUENCY = 1000; // Fréquence du signal PWM (1kHz)
const int PWM_RESOLUTION = 10;   // Résolution du signal PWM (0-1024)

// Variables pour stocker la vitesse
int motorSpeed = 300;  // Valeur de la vitesse (0-255)
Servo servoMotor;      // Objet pour contrôler le servo-moteur

void setup() {
  Serial.begin(9600);

  // Initialisation des broches des moteurs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialisation des capteurs de suivi de ligne
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  // Initialisation du capteur ultrason et du servo
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  servoMotor.attach(SERVO_PIN);

  // Configuration des canaux PWM pour ENA et ENB
  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION); // Canal pour ENA
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION); // Canal pour ENB

  // Attacher les canaux PWM aux broches ENA et ENB
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
}

void run(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Appliquer la vitesse avec PWM
  ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
  ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Arrêter les moteurs en coupant les signaux ENA et ENB
  ledcWrite(PWM_CHANNEL_A, 0); // Arrêter le moteur 1
  ledcWrite(PWM_CHANNEL_B, 0); // Arrêter le moteur 2
}

void turn_right(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Appliquer la vitesse avec PWM
  ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
  ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)
}

void turn_left(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Appliquer la vitesse avec PWM
  ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
  ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)
}

// Fonction pour mesurer la distance avec le capteur ultrason
int measureDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  return duration * 0.034 / 2;  // Conversion du temps en distance (cm)
}

// Fonction pour scanner l'environnement à la recherche d'obstacles
bool scanForObstacles() {
  for (int angle = 0; angle <= 180; angle += 30) {  // Scan de 0 à 180 degrés par pas de 30
    servoMotor.write(angle);  // Déplace le servo à l'angle spécifié
    delay(SERVO_SCAN_DELAY);  // Attend que le servo se positionne
    int distance = measureDistance();  // Mesure la distance
    if (distance < OBSTACLE_DISTANCE) {
      return true;  // Obstacle détecté
    }
  }
  return false;  // Pas d'obstacle détecté
}

void loop() {
  int leftSensorValue = digitalRead(LEFT_SENSOR_PIN);    // Lecture du capteur centre (utilisé comme gauche)
  int rightSensorValue = digitalRead(RIGHT_SENSOR_PIN);  // Lecture du capteur droit
  /* Si un obstacle est détecté
  if (scanForObstacles()) {
    stop();  // Arrête le robot
    Serial.println("Obstacle détecté !");
    return;  // Ne pas continuer si un obstacle est détecté
  }*/

  // Suivi de ligne basé sur les deux capteurs
  if (leftSensorValue == 0 && rightSensorValue == 0) {
    stop();  // Si les deux capteurs voient du blanc, arrêter pendant 10 secondes
    Serial.println("Arrêt pendant 1 secondes.");
    //delay(1000);  // Pause de 1 secondes
  } 
  else if (leftSensorValue == 1 && rightSensorValue == 0) {
    turn_left(motorSpeed);  // Tourner à gauche
    Serial.println("Tourner à gauche");
  } 
  else if (leftSensorValue == 0 && rightSensorValue == 1) {
    turn_right(motorSpeed);  // Tourner à droite
    Serial.println("Tourner à droite");
  } 
  else if (leftSensorValue == 1 && rightSensorValue == 1) {
    run(motorSpeed);  // Avancer tout droit
    Serial.println("Avancer tout droit");
  }

  delay(100);  // Pause pour stabiliser les lectures
}