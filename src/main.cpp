#include <Arduino.h>
#include <String.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

// Vérification que le Bluetooth est bien activé dans l'IDE
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

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
int maxOutput = 360;

/* ===================== OBJETS ===================== */
ESP32Encoder encoder_1;
ESP32Encoder encoder_2;
Adafruit_MPU6050 mpu;

/* ===================== VARIABLES PARTAGÉES ===================== */
volatile float Ve = 0; // entrée filtre
volatile float Vs = 0; // sortie filtre
volatile float erreur = 0; // sortie filtre
int cmd_P = 0;
int cmd_D = 0;
volatile bool FlagCalcul = false;

float Te = 10;    // ms
float Tau = 100; // ms
float A, B;
int k = 1; // gain de contrôle (ajusté empiriquement pour que le pwm atteigne 255 à l'équilibre)

float ax, ay, az, gz;
float vBAT;
float angleA;
float angleG;

float Kp = 2.94; // gain proportionnel (ajusté empiriquement pour que le pwm atteigne 255 à l'équilibre)
float Kd = 2.92; // gain dérivé (ajusté empiriquement pour que le pwm atteigne 255 à l'équilibre)
int cons = 0; // constante de contrôle (ajustée empiriquement pour que le pwm atteigne 255 à l'équilibre)
int offset = 135; // offset pour compenser les imperfections mécaniques (ajusté empiriquement pour que le pwm atteigne 255 à l'équilibre)

/* ===================== TACHE Batterie ===================== */
void taskBatterie(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vBAT = analogRead(ADCpin);
    vBAT = (vBAT * 14.4) / 4095.0;

    // Envoyer la tension au téléphone via Bluetooth (avec 2 décimales)
    if (SerialBT.hasClient()) {
      SerialBT.print("*T");
      SerialBT.println(String(vBAT*10, 3));
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
  }
}

/* ===================== TACHE erreurE (FILTRE) ===================== */
void taskerreure(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ax = a.acceleration.x; // m/s^2
    ay = a.acceleration.y;
    az = a.acceleration.z;
    gz = g.gyro.z; // rad/s

    // angle from accelerometer: convert radians -> degrees
    float angleA_rad = atan2(az, ax);
    angleA = angleA_rad * 180.0 / M_PI; // degrees

    // gyro: g.gyro.z is in rad/s -> convert to deg/s and integrate over Tau (ms -> s)
    angleG = (gz * 180.0 / M_PI) * (Tau / 1000.0); // degrees

    Ve = angleG + angleA;
    Vs = A * Ve + B * Vs;
    
    erreur = cons - Vs;

    cmd_P = Kp*erreur;
    cmd_D = cmd_P - (Kd * gz * (180/M_PI));

    if (cmd_D > 0)
    {
      cmd_D += offset;
      // clamp 
      if (cmd_D > maxOutput) cmd_D = maxOutput;

      // drive motors to correct positive error
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      ledcWrite(pwmChannel_1, cmd_D);
      ledcWrite(pwmChannel_2, 0.95 * (cmd_D));
    }
    else if (cmd_D < 0)
    {
      cmd_D = offset - cmd_D; // make cmd_D positive and add offset
      // clamp 
      if (cmd_D > maxOutput) cmd_D = maxOutput;

      // reverse direction
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      ledcWrite(pwmChannel_1, cmd_D);
      ledcWrite(pwmChannel_2, 0.95 * (cmd_D));
    }
    else
    {
      // stop
      ledcWrite(pwmChannel_1, 0);
      ledcWrite(pwmChannel_2, 0);
    }

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
      /*
      Serial.printf(
        "ax:%6d | ay:%6d | angleA:%6.2f | angleG:%6.2f | enc1:%6d | enc2:%6d | Vbat:%4.2fV\n",
        ax, ay, angleA, angleG, pos1, pos2, vBAT
      );
      */

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

  SerialBT.begin("ESP32_BestGyropode"); 
  Serial.println("Le Bluetooth est prêt, appariez votre téléphone !");

  /* Encodeurs */
  // ESP32Encoder::useInternalWeakPullResistors = UP;
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

  /* I2C + MPU6050 */
  Wire.begin();
  if (!mpu.begin())
  {
    Serial.println("MPU6050 NON DETECTE");
    while (1)
      ;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); //reverife les courbes 
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  /* ADC */
  analogReadResolution(12);
  analogSetPinAttenuation(ADCpin, ADC_11db);

  /* Coeff filtre */
  A = 1 / (1 + Tau / Te);
  B = (Tau / Te) * A;

  /* TACHES */
  xTaskCreate(taskBatterie, "Batterie", 6000, NULL, 5, NULL);
  xTaskCreate(taskerreure, "erreure", 4000, NULL, 10, NULL);
  xTaskCreate(taskSerial, "Serial", 6000, NULL, 1, NULL);
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
    /*
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
    */
     if (commande == "Kp")
    {
      Kp = valeur.toInt();
    }
     if (commande == "Kd")
    {
      Kd = valeur.toInt();
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
    //Serial.printf("%lf %lf %lf %lf\n", Vs, Ve, erreur, cmd_P); // mettre gyro, 

    FlagCalcul = 0;
  }

  // Envoyer du téléphone vers le moniteur série de l'ordi
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  
  // Envoyer de l'ordi vers le téléphone
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  delay(20);
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}


