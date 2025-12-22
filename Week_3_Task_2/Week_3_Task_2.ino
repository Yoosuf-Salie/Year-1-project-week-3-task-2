#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int LMotor1 = A3;
const int LMotor2 = 2;
const int RMotor1 = A1;
const int RMotor2 = A2;

const int LSpeedPin = 11;
const int RSpeedPin = 3;

int BASE_SPEED = 200;

//https://srituhobby.com/how-to-use-mpu6050-with-arduino-step-by-step-instructions/ for reference

//

Adafruit_MPU6050 mpu;

float AngleY = 0, actualY=0, AngleZ = 0;

unsigned long previous_time = 0;

int lasttime = 0;

bool first_time = false;

float zoffset;

void setup() {
  lcd.begin(16,2);
  
  //Motor Control
  pinMode(LMotor1, OUTPUT);
  pinMode(LMotor2, OUTPUT);
  pinMode(RMotor1, OUTPUT);
  pinMode(RMotor2, OUTPUT);
  
  //Motor Speed
  pinMode(LSpeedPin, OUTPUT);
  pinMode(RSpeedPin, OUTPUT);

  Serial.begin(9600);
  mpu.begin();
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  zoffset = g.gyro.z;
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gyroY = g.gyro.y  * (float)(180.0/PI);

  float gyroZ = (g.gyro.z - zoffset)  * (float)(180.0/PI);
  
  unsigned long currentTime = micros();
  float dt = (currentTime - previous_time) / 1000000.0; // seconds
  previous_time = currentTime;

  if (fabs(gyroY)>0.05)
  {
    actualY += gyroY * dt;
  }

  if (fabs(gyroZ)>0.3)
  {
    AngleZ += gyroZ * dt;
  }

  if (actualY > AngleY)
  {
    AngleY = actualY;
  }

  lcd.setCursor(0,0);
  lcd.print("Max Y= ");
  lcd.print(AngleY, 2);

  
  if (first_time == true && fabs(AngleZ) > 42)
  {
    Stop();
    while (1 >0)
    {
      
    }
  }
  else if (AngleY > 20 && actualY > -3 && actualY < 3)
  {
    if (first_time == false)
    {
      delay(200);
      AngleZ = 0;
      Stop();
      delay(4000);
    }
    Right();
    first_time = true;
  }
  else
  {
    Forward();
  }
  /*
  lcd.setCursor(0,1);
  lcd.print("Z=");
  lcd.print(AngleZ,1);
  */
}

void Forward() {
  analogWrite(RSpeedPin, BASE_SPEED);
  analogWrite(LSpeedPin, BASE_SPEED);
  
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);

  digitalWrite(RMotor1, HIGH);
  digitalWrite(RMotor2, LOW);
  
}

void Reverse(){
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, HIGH);
  analogWrite(LSpeedPin, BASE_SPEED);
  
  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, HIGH);
  analogWrite(RSpeedPin, BASE_SPEED);
}

void Left() {
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, LOW);
  analogWrite(LSpeedPin, BASE_SPEED);
  
  digitalWrite(RMotor1, HIGH);
  digitalWrite(RMotor2, LOW);
  analogWrite(RSpeedPin, BASE_SPEED);
}

void Right() {
  analogWrite(RSpeedPin, BASE_SPEED);
  analogWrite(LSpeedPin, BASE_SPEED);
  
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);

  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, HIGH);
}

void Stop() {
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, LOW);

  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, LOW);
}
