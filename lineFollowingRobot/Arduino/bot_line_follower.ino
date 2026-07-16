#include <Servo.h>
#define PD_CONTROL
#ifndef PD_CONTROL
  #define SIMPLE_CONTROL
#endif 

#define DEBUG

constexpr int MAX_IR_SENSORS = 5;
constexpr int N = 20;
// IR sensor pins
const int irPinMapping[MAX_IR_SENSORS] = {5,6,7,8,12};
#ifdef DEBUG
  #define MAX_COLUMNS MAX_IR_SENSORS+4 
  int irSensorVal[N][MAX_COLUMNS]; // maintain history with error vals
#else
  #define MAX_COLUMNS MAX_IR_SENSORS
  int irSensorVal[N][MAX_IR_SENSORS]; 
#endif

int n = 0; // buffer number in irSensorVal
// ---------------- Motor Pins ----------------
const int motorLeftPin1 = 9;
const int motorLeftPin2 = 10;
const int ena = 11;

const int motorRightPin1 = 2;
const int motorRightPin2 = 4;
const int enb = 3;

// Controller Configuration
// PD
float Kp = 50;
float Kd = 20.0;
float error =0;
int lastError = 0; // Derivative ctrl variables
float controlOutput = 0;
unsigned long lastTime = 0;
const int maxMotorPWMVal = 150; // max can be 255 
const int minMotorPWMVal = 50; // deadband of motor
const int baseMotorSpeed = 100; // base speed for error correction

// if else ctrl variables
const int defaultforwardSpeed = 80; //cruiseSpeed
const int defaultTraceBackSpeed = 85; //turnSpeed
const int botTurningSpeed = 70; // pivotSpeed
const int defaultSearchLineSpeed = 70;
int lastDirection = 0; // -1 Left, 1 Right, 0 Center


unsigned long loopCount = 0;
unsigned long dbgLoopTime = 0;

void setup()
{
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(ena, OUTPUT);

  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(enb, OUTPUT);

  for(int i = 0; i< MAX_IR_SENSORS; i++){
    pinMode(irPinMapping[i], INPUT);
  }
  
  Serial.begin(9600);
  lastTime = millis();

}

void loop() {
  loopCount++;
  // for profiling the code 
  unsigned long currentTime = millis();
  unsigned long loopTime = currentTime - lastTime;
  lastTime = currentTime;
  //Add  loop here

  int ctrlMode = 1; //1--> PD control, 0--> if else logic  //digitalRead(ctrlModeSelectPin);

  // read all the IR sensors
  for(int i = 0; i< MAX_IR_SENSORS; i++){
    irSensorVal[n][i] = digitalRead(irPinMapping[i]);
  }

  if(ctrlMode){
    error = calculateError(n);
    int ddt = error - lastError;
    controlOutput = (Kp * error) + (Kd * ddt);  
    lastError = error;
    int leftSpeed = baseMotorSpeed - controlOutput;
    int rightSpeed = baseMotorSpeed + controlOutput;
    updateDiffDriveMotors(leftSpeed,rightSpeed);
    #ifdef DEBUG
      irSensorVal[n][MAX_IR_SENSORS+2] = error;
      irSensorVal[n][MAX_IR_SENSORS+3] = controlOutput;
    #endif
  } else{
    int irLeftOut = irSensorVal[n][0];
    int irLeftIn  = irSensorVal[n][1];
    int irMiddle  = irSensorVal[n][2];
    int irRightIn = irSensorVal[n][3];
    int irRightOut= irSensorVal[n][4];

    if (irMiddle == HIGH && irLeftIn == LOW && irRightIn == LOW) {
      ctrlMotorMoves(defaultforwardSpeed, defaultforwardSpeed, true, true);
      lastDirection = 0;
    }
    else if (irLeftIn == HIGH && irLeftOut == LOW && irRightIn == LOW) {
      ctrlMotorMoves(botTurningSpeed / 2, botTurningSpeed, true, true); 
      lastDirection = -1;
    }
    else if (irRightIn == HIGH && irRightOut == LOW && irLeftIn == LOW) {
      ctrlMotorMoves(botTurningSpeed, botTurningSpeed / 2, true, true); 
      lastDirection = 1;
    }
    else if (irLeftOut == HIGH) {
      ctrlMotorMoves(defaultTraceBackSpeed, defaultTraceBackSpeed, false, true); 
      lastDirection = -1;
    }
    else if (irRightOut == HIGH) {
      ctrlMotorMoves(defaultTraceBackSpeed, defaultTraceBackSpeed, true, false); 
      lastDirection = 1;
    }
    else if (irLeftOut == LOW && irLeftIn == LOW && irMiddle == LOW && irRightIn == LOW && irRightOut == LOW) {
      if (lastDirection == -1) {
        ctrlMotorMoves(defaultTraceBackSpeed, defaultTraceBackSpeed, false, true); 
      } 
      else if (lastDirection == 1) {
        ctrlMotorMoves(defaultTraceBackSpeed, defaultTraceBackSpeed, true, false); 
      } 
      else {
        ctrlMotorMoves(defaultSearchLineSpeed, defaultSearchLineSpeed, true, true); 
      }
    }
  }
  // update the buffer number you need for next iteration of reading.
  n++;
  if(n >= N) n = 0;
  // debug information to serial port
  dbgLoopTime = dbgLoopTime + loopTime;
  if(!(loopCount % 20)){
    Serial.println("loopCount:" + String(loopCount) +  " loopTime:" +  String(dbgLoopTime) + " n:" + String(n) + " controlOutput:" + String(controlOutput) + " error:"+String(error));
    //loopCount = 0;
    dbgLoopTime = 0;
    Serial.println("SensorVals:");
    for(int i = 0; i< N; i++){
      for(int j = 0; j < MAX_COLUMNS ; j++){
        Serial.print(String(irSensorVal[i][j])+" "); //+" "+String(irSensorVal[i][1])+" " + String(irSensorVal[i][2]) + " " + String(irSensorVal[i][3]) + " " + String(irSensorVal[i][4])+ " " + String(irSensorVal[i][5])+ " " + String(irSensorVal[i][6]) );
      }
      Serial.print("\n");
    }
  }// basic debug loop takes around 100 92 clock cycles @ 16MHz.

}

void ctrlMotorMoves(int leftSpeed, int rightSpeed, bool leftForward, bool rightForward) {
  
  if (leftForward == true) {
    digitalWrite(motorLeftPin1, HIGH);
    digitalWrite(motorLeftPin2, LOW);
  } else {
    digitalWrite(motorLeftPin1, LOW);
    digitalWrite(motorLeftPin1, HIGH);
  }
  analogWrite(ena, leftSpeed);
  if(rightForward == true) {
    digitalWrite(motorRightPin1, HIGH);
    digitalWrite(motorRightPin2, LOW);
  } else {
    digitalWrite(motorRightPin1, LOW);
    digitalWrite(motorRightPin2, HIGH);
  }
  analogWrite(enb, rightSpeed);
}

int calculateError(int n) {
  int activeSensors = 0;
  long totalWeight = 0;

  int weights[MAX_IR_SENSORS] = {-4, -1, 0, 1, 4};
  for (int i = 0; i < MAX_IR_SENSORS; i++) {
    if (irSensorVal[n][i] == HIGH) {
      totalWeight += weights[i];
      activeSensors++;
    }
  }
  #ifdef DEBUG
    irSensorVal[n][MAX_IR_SENSORS]   = totalWeight;
    irSensorVal[n][MAX_IR_SENSORS+1] = activeSensors;
  #endif

  // sharp turn handling, when sensors leave lines
  if (activeSensors == 0) {
    if (lastError < 0) return -6; // Forces immediate hard reverse pivot on left wheel
    if (lastError > 0) return 6;  // Forces immediate hard reverse pivot on right wheel
    return 0;
  }

  return totalWeight / activeSensors;
}

int constraint(int val, int minVal, int maxVal){
  if (val < minVal) return(minVal);
  if (val > maxVal) return(maxVal);
  return val;
}

// update motors
void updateDiffDriveMotors(int leftMotorSpeed,int rightMotorSpeed){
  if(leftMotorSpeed>=0) {
    digitalWrite(motorLeftPin1, LOW);
    digitalWrite(motorLeftPin2, HIGH);
  } else {
    digitalWrite(motorLeftPin1, HIGH);
    digitalWrite(motorLeftPin2, LOW);
    leftMotorSpeed = abs(leftMotorSpeed);
  }
  leftMotorSpeed = constraint(leftMotorSpeed,minMotorPWMVal,maxMotorPWMVal);
  
  if(rightMotorSpeed>=0) {
    digitalWrite(motorRightPin1, LOW);
    digitalWrite(motorRightPin2, HIGH);
  }else{
    digitalWrite(motorRightPin1, HIGH);
    digitalWrite(motorRightPin2, LOW);
    rightMotorSpeed = abs(rightMotorSpeed);
  }
  rightMotorSpeed = constraint(rightMotorSpeed,minMotorPWMVal,maxMotorPWMVal);
  
  analogWrite(ena, leftMotorSpeed);
  analogWrite(enb, rightMotorSpeed);
}

