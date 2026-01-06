#include <Arduino.h>
#include <NewPing.h>
#include <Servo.h>
#include "PID.h"

void echoCheck();
void PrintCurve(float actual, float target);
void LogToCSV(float time, float actual, float target);

#define ECHO 2 //D2
#define TRG 3 //D3
#define MAX_DIST 30
#define SERVO 9 //D9
#define PINGINTERVAL 33 //30Hz
#define PRINTINTERVAL 100 //500ms
#define BRIDGELENGTH 30

#define Kp 16 //16
#define Ki 0.7//0.04 //0.03
#define Kd 1.4//1.2 //0.8

#define SETVAL 8

unsigned long lastPrint = 0;
unsigned long lastPing = 0;
volatile float distance;
volatile float _ActualDistance;
NewPing sonar(TRG, ECHO, MAX_DIST);
Servo servo;

PID _pID(Kp, Ki, Kd,  105, 85, 33.33);

void setup() {

    Serial.begin(9600);
    lastPing = millis();
    servo.attach(9); 
    sonar.ping_timer(echoCheck);
}

void echoCheck() {
  if (sonar.check_timer()) {
    // Ping finished (either echo received or timeout)
    float result = sonar.ping_result / US_ROUNDTRIP_CM;
    if(result > 0 && result < BRIDGELENGTH)
      distance = result;
  }
}

void loop() {
  if(Serial.available() > 0)
  {
    float kp = Serial.parseFloat();
    _pID.SetConstants(kp, Ki, Kd);
  }

  if(millis() - lastPrint > PRINTINTERVAL)
  {
    // lastPrint = millis();
    // Serial.print(distance);
    // Serial.println(" cm");
    PrintCurve(distance, SETVAL);
    //LogToCSV(millis(), distance, SETVAL);
  }

  if(millis() - lastPing > PINGINTERVAL)
  {
    _ActualDistance = distance;
    //Run PID
    float motorSet = _pID.Calculate(SETVAL, distance);
    servo.write( motorSet);//map(motorSet, 110, 80, 80, 110)
    sonar.ping_timer(echoCheck);
  }

}

void PrintCurve(float actual, float target)
{

  Serial.print("Actual:");
  Serial.print(actual);
  Serial.print("\tTarget:");
  Serial.println(target);
}

void LogToCSV(float time, float actual, float target)
{
    Serial.print(time);
    Serial.print(',');
    Serial.print(target);
    Serial.print(',');
    Serial.println(actual);
}
