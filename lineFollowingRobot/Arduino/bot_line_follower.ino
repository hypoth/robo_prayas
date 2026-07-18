#include <Servo.h>
#define DEBUG

#undef TEST_MOTOR_DIRECTIONS
#define DEBUG_WITH_3_SENSORS
#define WHITE_LINE
#define PD_CONTROL
constexpr int N = 5; // depth of history buffer
// IR sensor pins
#ifdef DEBUG_WITH_3_SENSORS
  #define MAX_IR_SENSORS 3
  const int irPinMapping[MAX_IR_SENSORS] = {8,7,6};
  int weights[MAX_IR_SENSORS] = {-1, 0, 1};
#else
  #define MAX_IR_SENSORS 5
  const int irPinMapping[MAX_IR_SENSORS] = {9,8,7,6,5};
  int weights[MAX_IR_SENSORS] = {-4, -1, 0, 1, 4};
#endif

#ifdef DEBUG
  #define MAX_COLUMNS MAX_IR_SENSORS+4 
  int irSensorVal[N][MAX_COLUMNS]; // maintain history with error vals
#else
  #define MAX_COLUMNS MAX_IR_SENSORS
  int irSensorVal[N][MAX_IR_SENSORS]; 
#endif

int n = 0; // buffer number in irSensorVal
// ---------------- Motor Pins ----------------
const int motorLeftPin1 = 2;
const int motorLeftPin2 = 4;
const int ena = 3;

const int motorRightPin1 = 10;
const int motorRightPin2 = 12;
const int enb = 11;

// Controller Configuration
// PD
float Kp = 2.0;
float Kd = 30;
float error =0;
int lastError = 0; // Derivative ctrl variables
float controlOutput = 0;
unsigned long lastTime = 0;
const int maxMotorPWMVal = 70; //110; // max can be 255 
const int minMotorPWMVal = 30; // deadband of motor
const int baseMotorSpeed = 50; //70; // base speed for error correction

// if else ctrl variables
const int defaultforwardSpeed = 75; //80;
const int defaultTraceBackSpeed = 75;// 70;
const int botTurningSpeed = 75; //70;
const int defaultSearchLineSpeed = 75; //55;
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
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  lastTime = millis();
  
  //stop the motors
  digitalWrite(motorLeftPin1, LOW);
  digitalWrite(motorLeftPin2, LOW);

  digitalWrite(motorRightPin1, LOW);
  digitalWrite(motorRightPin2, LOW);
  analogWrite(ena, 0);
  analogWrite(enb, 0);

  digitalWrite(LED_BUILTIN,HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN,LOW);
  #ifdef TEST_MOTOR_DIRECTIONS
    //updateDiffDriveMotors(70,-70); //working 
    //ctrlMotorMoves(defaultforwardSpeed, defaultforwardSpeed, true, true); //drifting left
    //ctrlMotorMoves(botTurningSpeed / 2, botTurningSpeed, true, true); // goes in circle
    //ctrlMotorMoves(botTurningSpeed, botTurningSpeed / 2, true, true); // right circle larger than left circle.
    //ctrlMotorMoves(defaultTraceBackSpeed, defaultTraceBackSpeed, false, true); // left turn , left wheel stationary (fix added)
    ctrlMotorMoves(defaultTraceBackSpeed, defaultTraceBackSpeed, true, false); // right turn, left wheel moves back
  #endif
}

void loop() {
  loopCount++;
  // for profiling the code 
  unsigned long currentTime = millis();
  unsigned long loopTime = currentTime - lastTime;
  lastTime = currentTime;
  //Add  loop here
  #ifndef TEST_MOTOR_DIRECTIONS
    #ifdef PD_CONTROL
      int ctrlMode = 1; //1--> PD control, 0--> if else logic  //digitalRead(ctrlModeSelectPin);
    #else
      int ctrlMode = 0; 
    #endif
    readIRSensors(n); // read all the sensors and store values in nth Buffer

    if(ctrlMode){
      error = calculateError(n);
      int ddt = error - lastError;
      controlOutput = (Kp * error) + (Kd * ddt);  
      lastError = error;
      int leftSpeed = baseMotorSpeed - controlOutput;
      int rightSpeed = baseMotorSpeed + controlOutput;
      updateDiffDriveMotors(leftSpeed,rightSpeed);
      #ifdef DEBUG
        irSensorVal[n][MAX_IR_SENSORS] = error;
        irSensorVal[n][MAX_IR_SENSORS+1] = controlOutput;
      #endif
    } else{
      controlLogicIfElse(n);
    }
  #endif  
  // update the buffer number you need for next iteration of reading.
  n++;
  if(n >= N) n = 0;
  #ifdef DEBUG
    // debug information to serial port
    dbgLoopTime = dbgLoopTime + loopTime;
    if(!(loopCount % N)){
      Serial.println("loopCount:" + String(loopCount) +  " loopTime:" +  String(dbgLoopTime) + " n:" + String(n) + " controlOutput:" + String(controlOutput) + " error:"+String(error));
      //loopCount = 0;
      dbgLoopTime = 0;
      Serial.println("SensorVals:");
      for(int i = 0; i< N; i++){
        for(int j = 0; j < MAX_COLUMNS ; j++){
          Serial.print(String(irSensorVal[i][j])+" ");
        }
        Serial.print("\n");
      }
    }// basic debug loop takes around 100 92 clock cycles @ 16MHz.
  #endif
}

void readIRSensors(int n){
  // read all the IR sensors DEFAULT // Black surface gives out HIGH // White surfce gives out LOW
  for(int i = 0; i< MAX_IR_SENSORS; i++){
    #ifdef WHITE_LINE
      irSensorVal[n][i] = !digitalRead(irPinMapping[i]); // inverting - White gives one
    #else
      irSensorVal[n][i] = digitalRead(irPinMapping[i]); // storing data as is for following black line
    #endif
  }
}

int calculateError(int n) {
  int activeSensors = 0;
  long totalWeight = 0;
  for (int i = 0; i < MAX_IR_SENSORS; i++) {
    if (irSensorVal[n][i] == HIGH) { //int weights[MAX_IR_SENSORS] = {-4, -1, 0, 1, 4};
      totalWeight += weights[i];
      #ifdef DEBUG
        //irSensorVal[n][i] = weights[i]; // debug buffer has weight only
      #endif
      activeSensors++;
    }
  }
  if (activeSensors == 0) {   // sharp turn handling, when sensors leave lines
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
  #ifdef DEBUG
    irSensorVal[n][MAX_IR_SENSORS+2] = leftMotorSpeed;
    irSensorVal[n][MAX_IR_SENSORS+3] = rightMotorSpeed;    
  #endif
}

void controlLogicIfElse(int n){
    int irLeftOut = irSensorVal[n][0];
    int irLeftIn  = irSensorVal[n][1];
    int irMiddle  = irSensorVal[n][2];
    
    //#ifndef DEBUG_WITH_3_SENSORS
      int irRightIn = irSensorVal[n][3];
      int irRightOut= irSensorVal[n][4];
    //#endif

    if (irMiddle == HIGH && irLeftIn == LOW && irRightIn == LOW) {
      ctrlMotorMoves(defaultforwardSpeed, defaultforwardSpeed, true, true);
      lastDirection = 0;
    }
    else if (irLeftIn == HIGH && irLeftOut == LOW && irRightIn == LOW) {
      ctrlMotorMoves(3/4*botTurningSpeed, botTurningSpeed, true, true); 
      lastDirection = -1;
    }
    else if (irRightIn == HIGH && irRightOut == LOW && irLeftIn == LOW) {
      ctrlMotorMoves(3/4*botTurningSpeed, botTurningSpeed / 2, true, true); 
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
      digitalWrite(LED_BUILTIN,HIGH);
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
void ctrlMotorMoves(int leftSpeed, int rightSpeed, bool leftForward, bool rightForward) {  
  if (leftForward == true) {
    digitalWrite(motorLeftPin1, LOW);
    digitalWrite(motorLeftPin2, HIGH);
  } else {
    digitalWrite(motorLeftPin1, HIGH);
    digitalWrite(motorLeftPin2, LOW);
  }
  analogWrite(ena, leftSpeed);
  if(rightForward == true) {
    digitalWrite(motorRightPin1, LOW);
    digitalWrite(motorRightPin2, HIGH);
  } else {
    digitalWrite(motorRightPin1, HIGH);
    digitalWrite(motorRightPin2, LOW);
  }
  analogWrite(enb, rightSpeed);
}


