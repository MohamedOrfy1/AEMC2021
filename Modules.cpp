#include <Arduino.h>
#include "Modules.h"
#include <PID_v1.h>
#include <QMC5883L.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

QMC5883L compass;

double startangle=0,newangle=0,Output=0,SpeedR=INITIAL_SPEED,SpeedL=INITIAL_SPEED;

char path[100] = " ";
unsigned char pathlength = 0;
int pathindex = 0;

PID myPID(&newangle, &Output, &startangle, KP, KI, KD, DIRECT);

void Forward(unsigned int Speed1,unsigned int Speed2)
{
  analogWrite(IN1,Speed1);
  analogWrite(IN2,0);
  analogWrite(IN3,Speed2);
  analogWrite(IN4,0);
}
void Backward(unsigned int Speed1,unsigned int Speed2)
{
  analogWrite(IN1,0);
  analogWrite(IN2,Speed1);
  analogWrite(IN3,0);
  analogWrite(IN4,Speed2);
}
void Right(unsigned int Speed)
{
  analogWrite(IN1,0);
  analogWrite(IN2,Speed);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}
void Left(unsigned int Speed)
{
  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,Speed);
}
void Hard_Left(unsigned int Speed)
{
  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);
}
void Hard_Right(unsigned int Speed)
{
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}

void Stop(void)
{
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);
}

void Forward_PID()
{
  Forward(SpeedR,SpeedL);
  myPID.Compute();
  
  SpeedR = SpeedR-Output;
  SpeedL = SpeedL+Output;

  SpeedR = constrain(SpeedR,0,255);
  SpeedL = constrain(SpeedL,0,255);
}


void Init(void)
{
  Serial.begin(BAUD_RATE);
  pinMode(StartSW, INPUT_PULLUP);
  pinMode(DipSW1, INPUT_PULLUP);
  pinMode(DipSW2, INPUT_PULLUP);
  pinMode(Debug_Buzzer, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  compass.init();
  compass.setSamplingRate(50);

  myPID.SetOutputLimits(OUTPUT_LIMIT*-1,OUTPUT_LIMIT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

int ReadCompass(void)
{
  return compass.readHeading();
}

void path_save(char direction)
{
  path[pathlength] = direction;
  pathlength ++;
}
void setID(void) 
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1);
  }
}
void read_three_sensors(int* reading1,int* reading2,int* reading3) 
{
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  
  // print sensor one reading
  Serial.print(F("1: "));
  if (measure1.RangeStatus != 4) {    // if not out of range
    Serial.print(measure1.RangeMilliMeter);
    *reading1=measure1.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    *reading2=measure2.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("3: "));
  if (measure3.RangeStatus != 4) {    // if not out of range
    Serial.print(measure3.RangeMilliMeter);
    *reading3=measure3.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}
void turn_right(double startangle){
   ReadCompass();
  while(ReadCompass()-startangle<90){
    Right(50);
    }
}
void turn_left(double startangle){
  float New=ReadCompass();
  while(New-startangle>-90){
    Left(50);
    }
}
