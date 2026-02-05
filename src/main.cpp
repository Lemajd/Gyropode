#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 

#define CLK1 34 // CLK ENCODER 
#define DT1 35 // DT ENCODER 
#define CLK2 32 // CLK ENCODER 
#define DT2 33 // DT ENCODER 

//moteur droit
int motor1Pin1 = 23; 
int motor1Pin2 = 19; 
int enableAPin = 15; 
//moteur gauche
int motor2Pin1 = 18; 
int motor2Pin2 = 17; 
int enableBPin = 0; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel_1 = 0;
const int pwmChannel_2 = 0;
const int resolution = 8;
int dutyCycle = 150;

//variable de mesure batterie
int ADCpin = 25;
float vBAT;

//variable eval vitesse


ESP32Encoder encoder_1;
ESP32Encoder encoder_2;

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

int16_t az;
int16_t gz;

void setup() {
  encoder_1.attachHalfQuad ( DT1, CLK1 );
  encoder_1.setCount ( 0 );
  encoder_2.attachHalfQuad ( DT2, CLK2 );
  encoder_2.setCount ( 0 );

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enableBPin, OUTPUT);

  // Configuration PWM
  ledcSetup(pwmChannel_1, freq, resolution);
  ledcSetup(pwmChannel_2, freq, resolution);

  // Attacher les pins au canal PWM
  ledcAttachPin(enableAPin, pwmChannel_1);
  ledcAttachPin(enableBPin, pwmChannel_2);

  // Appliquer le duty cycle
  ledcWrite(pwmChannel_1, dutyCycle);
  ledcWrite(pwmChannel_2, dutyCycle);

  // Move the DC motor forward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);


  /*--Start I2C interface--*/
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200); //Initializate Serial wo work well at 8MHz/16MHz

  /*Initialize device and check connection*/ 
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
  mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z
  /*Print the defined offsets*/ 
  Serial.print("\t");
  Serial.print(mpu.getZAccelOffset());
  Serial.print("\t");
  Serial.print(mpu.getZGyroOffset());
  Serial.print("\n");
}

void loop() {
  /* Read raw accel/gyro data from the module. Other methods commented*/
  int16_t az = mpu.getAccelerationZ();
  int16_t gz = mpu.getRotationZ();

  int Position_1 = (-1)*encoder_1.getCount();
  Serial.printf("enc1:%5d\t", Position_1);
  int Position_2 = encoder_2.getCount();
  Serial.printf("enc2:%5d\t", Position_2);
  vBAT = analogRead(ADCpin);
  vBAT = (vBAT*14.4)/4095;
  Serial.printf("tension bat : %2.1fV\t", vBAT);

  /*Print the obtained data on the defined format*/
  #ifdef OUTPUT_READABLE_ACCELGYRO
    Serial.printf("az : %6d\t", az);
    Serial.printf("gz : %6d\n", gz);
  #endif

  #ifdef OUTPUT_BINARY_ACCELGYRO
    Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
    Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
  #endif

  delay(100);
}