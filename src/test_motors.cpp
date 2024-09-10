#include <Arduino.h>
#include <ESP32Servo.h>

// Motor 1
#define IN1 32 // 18
#define IN2 33 // 5
// Motor 2
#define IN3 25 // 17
#define IN4 26 // 16

#define ENA 27 // Broche PWM pour le moteur 1
#define ENB 14 // Broche PWM pour le moteur 2

// Définir les canaux PWM pour les moteurs
const int PWM_CHANNEL_A = 0; // Canal PWM pour ENA (moteur 1)
const int PWM_CHANNEL_B = 1; // Canal PWM pour ENB (moteur 2)

const int PWM_FREQUENCY = 1000; // Fréquence du signal PWM (1kHz)
const int PWM_RESOLUTION = 8;   // Résolution du signal PWM (0-255)

// Variables pour stocker la vitesse
int motorSpeed = 150; // Valeur de la vitesse (0-255)

void setup()
{
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Configuration des canaux PWM pour ENA et ENB
    ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION); // Canal pour ENA
    ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION); // Canal pour ENB

    // Attacher les canaux PWM aux broches ENA et ENB
    ledcAttachPin(ENA, PWM_CHANNEL_A);
    ledcAttachPin(ENB, PWM_CHANNEL_B);
}

void run(int speed)
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.print("RUN RUN");

    // Appliquer la vitesse avec PWM
    ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
    ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)

    Serial.print("Robot en marche à la vitesse de ");
    Serial.println(speed);
}

void stop()
{
    /*
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.print("STOP");
    */
    // Arrêter les moteurs en coupant les signaux ENA et ENB
    ledcWrite(PWM_CHANNEL_A, 0); // Arrêter le moteur 1
    ledcWrite(PWM_CHANNEL_B, 0); // Arrêter le moteur 2

    Serial.println("Robot arrêté");
}

void move_back(int speed)
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.print("BACK");

    // Appliquer la vitesse avec PWM
    ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
    ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)
    Serial.print("Robot en marche à la vitesse de ");
    Serial.println(speed);
}

void turn_right(int speed)
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.print("TURN RIGHT");

    // Appliquer la vitesse avec PWM
    ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
    ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)
    Serial.print("Robot en marche à la vitesse de ");
    Serial.println(speed);
}

void turn_left(int speed)
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.print("TURN LEFT");

    // Appliquer la vitesse avec PWM
    ledcWrite(PWM_CHANNEL_A, speed); // Contrôle la vitesse du moteur 1 (ENA)
    ledcWrite(PWM_CHANNEL_B, speed); // Contrôle la vitesse du moteur 2 (ENB)
    Serial.print("Robot en marche à la vitesse de ");
    Serial.println(speed);
}

void loop()
{
    run(motorSpeed);
    delay(1000);
    stop();
    delay(2500);
    turn_right(motorSpeed);
    delay(5000);
    turn_left(motorSpeed);
    delay(5000);
    move_back(motorSpeed);
    delay(3000);
}
