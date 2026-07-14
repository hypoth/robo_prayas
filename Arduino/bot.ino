#include <Servo.h>

// ---------------- Motor Pins ----------------
const int motorLeftPin1 = 2;
const int motorLeftPin2 = 4;
const int ena = 11;

const int motorRightPin1 = 9;
const int motorRightPin2 = 10;
const int enb = 3;

// ---------------- Ultrasonic Sensor ----------------
const int trigPin = 8;
const int echoPin = 7;

// ---------------- Servo ----------------
const int radarServoPin = 5;
const int vehicleRadarAxisDifferenceAngle = 90; // angle the servo should be in for vehile logtitudnal axis be in line with radar axis.

Servo radar;

// ---------------- Distance Limits ----------------
const float minDistance = 10.0;
const float minSafeDistance = 25.0;
const int sensorDurationMax = 25000;

const int traceBackSpeed = 70;
const int botTurningSpeed = 75;
const int defaultforwardSpeed = 65;
int currentMotorSpeed = 0;

const int maxMotorPWMVal = 80;
const int minMotorPWMVal = 65; // deadband of motor
const float maxMotorPowerDistance = 150;

void setup()
{
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(ena, OUTPUT);

  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(enb, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  radar.attach(radarServoPin);
  radar.write(vehicleRadarAxisDifferenceAngle);          // Face forward
  delay(200);
  Serial.begin(9600);
}

void loop()
{
  int forwardSpeed = defaultforwardSpeed; 
  float frontDistance = measureDistance();
  delay(80);

  
  //Serial.println("Front Clearance: " + String(frontDistance));
	if(frontDistance > 0) {
    if(frontDistance <= minSafeDistance){
      stopRobot();
      delay(200);

      backward(traceBackSpeed);
      delay(400);

      stopRobot();
      delay(200);

      // Look Right
      radar.write(vehicleRadarAxisDifferenceAngle-90);
      delay(300);
      float right = measureDistance();
      Serial.println("Rigt Side clearance: " + String(right));
      // Look Left
      for(int i = 0; i<=vehicleRadarAxisDifferenceAngle+90; i=i+30)
      {
        radar.write(i);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
      float left = measureDistance();
      Serial.println("Left Side clearance: " + String(left));

      // Center Servo
      radar.write(vehicleRadarAxisDifferenceAngle);
      delay(300);

      if(left > right) // turn in direction where you have longer visibility
      {
        turnLeft(botTurningSpeed);
        delay(650);
        Serial.println("Turning Left with Speed: " + String(botTurningSpeed));
      }
      else
      {
        turnRight(botTurningSpeed);
        delay(650);
        Serial.println("Turning Right with Speed: " + String(botTurningSpeed));
      }

      stopRobot();
      delay(200);
    }
  } 
  
  if(frontDistance > minSafeDistance){
    //  map distance range 
    int mappedMotorSpeed = map(frontDistance, minSafeDistance,maxMotorPowerDistance, minMotorPWMVal,maxMotorPWMVal);

    if(mappedMotorSpeed != currentMotorSpeed) {
  	  forward(mappedMotorSpeed);
      delay(200);
      //Serial.println("Moving Forward with Speed: " + String(mappedMotorSpeed));
    }
  }  
  
}

// ---------------- Distance Function ----------------

float measureDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, sensorDurationMax);
  float distance = 0;
  if(duration < sensorDurationMax){
  	//distance = duration / 58.0; //change is to multiplicaiton
    distance = duration * 0.0172; //change is to multiplicaiton
    
  }
  return distance;
}

// ---------------- Robot Movements ----------------

void forward(int speed)
{
  digitalWrite(motorLeftPin1, LOW);
  digitalWrite(motorLeftPin2, HIGH);

  digitalWrite(motorRightPin1, LOW);
  digitalWrite(motorRightPin2, HIGH);

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  currentMotorSpeed = speed;
}

void backward(int speed)
{
  digitalWrite(motorLeftPin1, HIGH);
  digitalWrite(motorLeftPin2, LOW);

  digitalWrite(motorRightPin1, HIGH);
  digitalWrite(motorRightPin2, LOW);

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  currentMotorSpeed = speed;
}

void turnLeft(int speed)
{
  digitalWrite(motorLeftPin1, HIGH);
  digitalWrite(motorLeftPin2, LOW);

  digitalWrite(motorRightPin1, LOW);
  digitalWrite(motorRightPin2, HIGH);

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  currentMotorSpeed = speed;
}

void turnRight(int speed)
{
  digitalWrite(motorLeftPin1, LOW);
  digitalWrite(motorLeftPin2, HIGH);

  digitalWrite(motorRightPin1, HIGH);
  digitalWrite(motorRightPin2, LOW);

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  currentMotorSpeed = speed;
}

void stopRobot()
{
  analogWrite(ena, 0);
  analogWrite(enb, 0);

  digitalWrite(motorLeftPin1, LOW);
  digitalWrite(motorLeftPin2, LOW);

  digitalWrite(motorRightPin1, LOW);
  digitalWrite(motorRightPin2, LOW);
  currentMotorSpeed = 0;
}
	
