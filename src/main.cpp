#include <Arduino.h>
#include <String.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Encoder.h>

/* ===================== PINOUT ===================== */
#define CLK1 34
#define DT1  35
#define CLK2 32
#define DT2  33

int motor1Pin1 = 23;
int motor1Pin2 = 19;
int enableAPin = 15;

int motor2Pin1 = 18;
int motor2Pin2 = 17;
int enableBPin = 0;

int ADCpin = 25;

/* ===================== PWM ===================== */
const int freq = 30000;
const int pwmChannel_1 = 0;
const int pwmChannel_2 = 1;
const int resolution = 8;
int dutyCycle = 150;

/* ===================== OBJETS ===================== */
ESP32Encoder encoder_1;
ESP32Encoder encoder_2;
MPU6050 mpu;

/* ===================== VARIABLES PARTAGÉES ===================== */
volatile float Ve = 0;   // entrée filtre
volatile float Vs = 0;   // sortie filtre
volatile bool FlagCalcul = false;

float Te = 10;     // ms
float Tau = 1000;  // ms
float A, B;

int16_t az, gz;
float vBAT;

/* ===================== TACHE CAPTEURS ===================== */
void taskCapteurs(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    az = mpu.getAccelerationZ();
    gz = mpu.getRotationZ();

    Ve = az;   // entrée du filtre

    vBAT = analogRead(ADCpin);
    vBAT = (vBAT * 14.4) / 4095.0;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

/* ===================== TACHE CONTROLE (FILTRE) ===================== */
void taskControle(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    Vs = A * Ve + B * Vs;
    FlagCalcul = true;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}

/* ===================== TACHE SERIAL ===================== */
void taskSerial(void *parameters)
{
  while (1)
  {
    if (FlagCalcul)
    {
      int pos1 = -encoder_1.getCount();
      int pos2 = encoder_2.getCount();

      Serial.printf(
        "az:%6d | az_filt:%8.2f | gz:%6d | enc1:%6d | enc2:%6d | Vbat:%4.2f\n",
        az, Vs, gz, pos1, pos2, vBAT
      );

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
  //ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder_1.attachHalfQuad(DT1, CLK1);
  encoder_2.attachHalfQuad(DT2, CLK2);

  /* PWM */
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  ledcSetup(pwmChannel_1, freq, resolution);
  ledcSetup(pwmChannel_2, freq, resolution);
  ledcAttachPin(enableAPin, pwmChannel_1);
  ledcAttachPin(enableBPin, pwmChannel_2);
  ledcWrite(pwmChannel_1, dutyCycle);
  ledcWrite(pwmChannel_2, dutyCycle);

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  /* I2C + MPU6050 */
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 NON DETECTE");
    while (1);
  }

  /* ADC */
  analogReadResolution(12);
  analogSetPinAttenuation(ADCpin, ADC_11db);

  /* Coeff filtre */
  A = 1 / (1 + Tau / Te);
  B = (Tau / Te) * A;

  /* TACHES */
  xTaskCreate(taskCapteurs, "Capteurs", 6000, NULL, 5, NULL);
  xTaskCreate(taskControle, "Controle", 4000, NULL, 10, NULL);
  xTaskCreate(taskSerial,   "Serial",   6000, NULL, 1, NULL);
}

/* ===================== LOOP ===================== */
void loop()
{
  // vide, FreeRTOS travaille
}
