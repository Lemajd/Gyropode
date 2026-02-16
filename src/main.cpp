#include <Arduino.h>
#include <String.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

/* ===================== PINOUT ===================== */
#define CLK1 34
#define DT1 35
#define CLK2 32
#define DT2 33

int motor1Pin1 = 23;
int motor1Pin2 = 19;
int enableAPin = 15;

int motor2Pin1 = 18;
int motor2Pin2 = 17;
int enableBPin = 0;

int ADCpin = 25;

/* ===================== cmd_P ===================== */
const int freq = 30000;
const int pwmChannel_1 = 0;
const int pwmChannel_2 = 1;
const int resolution = 8;
int dutyCycle = 150;

/* ==================== Codeur =======================*/
int pos1 = 0;
int pos2 = 0;
int deltaPos1 = 0;
int deltaPos2 = 0;
float VitesseCentreGeo = 0;

/* ===================== OBJETS ===================== */
ESP32Encoder encoder_1;
ESP32Encoder encoder_2;
Adafruit_MPU6050 mpu;

/* ===================== VARIABLES PARTAGÉES ===================== */
volatile float Ve = 0;     // entrée filtre
volatile float Vs = 0;     // sortie filtre
volatile float erreur = 0; // sortie filtre
float cmd_P = 0;
float cmd_D = 0;
volatile bool FlagCalcul = false;
int maxOutput = 255; // valeur maximale du signal de commande (pwm 8 bits)

float Te = 10;   // ms
float Tau = 100; // ms
float A, B;
int k = 1; // gain 
float ax, ay, az, gz;
float vBAT; // tension batterie
float angleA; // angle de l'accéléromètre
float angleG; // angle du gyroscope 

float Kp = 4;  // gain proportionnel 
float Kd = 0.1;  // gain dérivé 
float Kv = -0.8; // gain de compensation de la vitesse
int aCons= -4;     // consigne d'angle 
int vCons = 0;    // consigne de vitesse
int offset = 130; // offset pour compenser les imperfections mécaniques (ajusté empiriquement pour que le pwm atteigne 255 à l'équilibre)
float moteur1_ratio = 1.0;  // ratio de puissance moteur 1
float moteur2_ratio = 0.97;  // ratio de puissance moteur 2

/* ===================== TACHE Batterie ===================== */
void taskBatterie(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vBAT = analogRead(ADCpin);
    vBAT = (vBAT * 14.4) / 4095.0;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
  }
}

/* ===================== TACHE Control (FILTRE) ===================== */
void taskControl(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x; // m/s^2
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gz = g.gyro.y; // rad/s

    aCons = (vCons - VitesseCentreGeo) * Kv; // consigne d'angle ajustée en fonction de la vitesse actuelle pour compenser les imperfections mécaniques
    if (aCons > 10) aCons = 10; // limiter la consigne d'angle pour éviter les commandes trop agressives
    if (aCons < -10) aCons = -10;
    
    // Position
    deltaPos1 = -encoder_1.getCount() + pos1;
    deltaPos2 = encoder_2.getCount() - pos2;

    VitesseCentreGeo = ((deltaPos1 + deltaPos2) / 2.0);

    // angle from accelerometer: convert radians -> degrees
    float angleA_rad = atan2(az, ax);
    angleA = angleA_rad * 180.0 / M_PI; // degrees

    // gyro: g.gyro.z is in rad/s -> convert to deg/s and integrate over Tau (ms -> s)
    angleG = (gz * 180.0 / M_PI) * (Tau / 1000.0); // degrees

    Ve = angleG + angleA;
    Vs = A * Ve + B * Vs;

    erreur = aCons- Vs;

    cmd_P = Kp * erreur;
    cmd_D = cmd_P - (Kd * gz * (180 / M_PI));

    // Contrôle de direction via les pins numériques (selon le signe de cmd_D)
    digitalWrite(motor1Pin1, (cmd_D > 0) ? LOW : HIGH);
    digitalWrite(motor1Pin2, (cmd_D > 0) ? HIGH : LOW);
    digitalWrite(motor2Pin1, (cmd_D > 0) ? LOW : HIGH);
    digitalWrite(motor2Pin2, (cmd_D > 0) ? HIGH : LOW);

    // Modulation de vitesse en fonction de la magnitude
    int pwm_speed = min(maxOutput, (int)(abs(cmd_D) + offset));

    ledcWrite(pwmChannel_1, moteur1_ratio * pwm_speed);
    ledcWrite(pwmChannel_2, moteur2_ratio * pwm_speed);

    pos1 = encoder_1.getCount();
    pos2 = encoder_2.getCount();

    FlagCalcul = true;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

/* ===================== TACHE Calcul ===================== */
void taskCalcul(void *parameters)
{
  while (1)
  {
    if (FlagCalcul)
    {
      
      FlagCalcul = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ===================== SETUP ===================== */
void setup()
{
  Serial.begin(115200);
  Serial.println("Systeme demarre");

  /* Encodeurs */
  // ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder_1.attachHalfQuad(DT1, CLK1);
  encoder_2.attachHalfQuad(DT2, CLK2);

  /* PWM */
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set motor pins to fixed state (never changes again)
  // Direction is now controlled via PWM channels instead
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  ledcSetup(pwmChannel_1, freq, resolution);
  ledcSetup(pwmChannel_2, freq, resolution);
  ledcAttachPin(enableAPin, pwmChannel_1);
  ledcAttachPin(enableBPin, pwmChannel_2);
  ledcWrite(pwmChannel_1, dutyCycle);
  ledcWrite(pwmChannel_2, dutyCycle);

  /* I2C + MPU6050 */
  Wire.begin();
  if (!mpu.begin())
  {
    Serial.println("MPU6050 NON DETECTE");
    while (1)
      ;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // reverifier les courbes
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  /* ADC */
  analogReadResolution(12);
  analogSetPinAttenuation(ADCpin, ADC_11db);

  /* Coeff filtre */
  A = 1 / (1 + Tau / Te);
  B = (Tau / Te) * A;

  /* TACHES */
  xTaskCreate(taskBatterie, "Batterie", 6000, NULL, 5, NULL);
  xTaskCreate(taskControl, "Control", 4000, NULL, 10, NULL);
  xTaskCreate(taskCalcul, "Calcul", 6000, NULL, 1, NULL);
}

void reception(char ch)
{
  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }
    
    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    
    if (commande == "Kp")
    {
      Kp = valeur.toFloat();
    }
    if (commande == "Kd")
    {
      Kd = valeur.toFloat();
    }

    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

/* ===================== LOOP ===================== */
void loop()
{
  if (FlagCalcul == 1)
  {
    Serial.printf("%f %f %f %f\n", Vs, VitesseCentreGeo, cmd_P, cmd_D);
    FlagCalcul = 0;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}
