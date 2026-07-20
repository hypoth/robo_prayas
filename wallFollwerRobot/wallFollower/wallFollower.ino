/*
  NodeMCU (ESP8266) Wall follower robot
*/
#include <ESP8266WiFi.h>
#include <TelnetStream.h>

#define ENA D1
#define IN1 D0
#define IN2 D3
#define ENB D2
#define IN3 D4
#define IN4 D8

#define TRIG_PIN   D7
#define ECHO_SIDE  D5
#define ECHO_FRONT D6

#define MAX_SENSORS 2

#define DEBUG
#define N 200 // max depth of debug buffer.
#ifdef DEBUG
#define DBG_WITH_TELNET
  #define MAX_COLUMNS MAX_SENSORS+4 
  int rawSensorVal[N][MAX_COLUMNS]; // maintain history with error vals
#else
  #define MAX_COLUMNS MAX_SENSORS
  int rawSensorVal[N][MAX_SENSORS]; 
#endif


const char* ssid = "Galaxygalaxy";
const char* password = "nokmobPas1";

// Create a Telnet Server on port 23
WiFiServer telnetServer(23);
WiFiClient telnetClient;


const bool FOLLOW_RIGHT_WALL = true;

// ---------------- Tunable parameters ----------------
const float SETPOINT_CM   = 15.0;   // desired distance from the wall
const float FRONT_STOP_CM = 20.0;   // front distance that triggers a pivot
const float LOST_WALL_CM  = 60.0;   // treat readings beyond this as "no wall"
const int   BASE_SPEED    = 180;    // 0-255 forward cruising speed
const int   MAX_SPEED     = 255;
const int   PIVOT_SPEED   = 150;

float Kp = 6.0;
float Ki = 0.02;
float Kd = 3.0;
// ------------------------------------------------------

float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

long readDistanceCM(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000UL); // 25ms timeout (~4m range)
  if (duration == 0) return -1; // no echo received (out of range / no wall)
  return duration / 58;         // convert to cm
}

const int MIN_PWM = 50;   // minimum PWM to overcome motor static friction - tune this

void setMotor(int enaPin, int in1, int in2, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  }
  analogWrite(enaPin, speed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(ENB, IN3, IN4, leftSpeed);
  setMotor(ENA, IN1, IN2, rightSpeed);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_SIDE, INPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  analogWriteRange(255); // so analogWrite() takes 0-255 like the constants above
  analogWriteFreq(1000);
  stopMotors();

  #ifdef DBG_WITH_TELNET
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Start the server
    telnetServer.begin();
    Serial.println("Telnet server started");
    TelnetStream.begin();
  #endif

  lastTime = millis();
}

void loop() {
  long sideCM  = readDistanceCM(ECHO_SIDE);
  delay(60); // let the first SR04 echo fully settle before firing the next
  long frontCM = readDistanceCM(ECHO_FRONT);
  delay(60);

  // --- Front obstacle: pivot away from the wall we're following ---
  if (frontCM > 0 && frontCM < FRONT_STOP_CM) {
    if (FOLLOW_RIGHT_WALL) {
      drive(-PIVOT_SPEED, PIVOT_SPEED); // pivot left, away from right wall
    } else {
      drive(PIVOT_SPEED, -PIVOT_SPEED); // pivot right, away from left wall
    }
    integral = 0; // don't let the PID wind up while pivoting
    lastError = 0;
    return;
  }

  // --- Side wall PID ---
  if (sideCM < 0) sideCM = LOST_WALL_CM; // no echo -> assume wall is far away

  float error = SETPOINT_CM - sideCM; // >0 = too close, <0 = too far

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;

  integral += error * dt;
  integral = constrain(integral, -50, 50); // anti-windup
  float derivative = (error - lastError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -BASE_SPEED, BASE_SPEED);

  lastError = error;
  lastTime = now;

  int leftSpeed, rightSpeed;
  if (FOLLOW_RIGHT_WALL) {
    leftSpeed  = BASE_SPEED - output; // too close -> steer left, away
    rightSpeed = BASE_SPEED + output;
  } else {
    leftSpeed  = BASE_SPEED + output;
    rightSpeed = BASE_SPEED - output;
  }
  drive(leftSpeed, rightSpeed);

  
  Serial.printf("side=%ld front=%ld err=%.1f out=%.1f  L=%d R=%d\n",
                sideCM, frontCM, error, output, leftSpeed, rightSpeed);
  
  Serial.println("Loop is running... sending to Telnet next.");
  
  #ifdef DBG_WITH_TELNET 
    TelnetStream.handle(); 
    // Check if a new client has connected
    if (telnetServer.hasClient()) {
      if (!telnetClient || !telnetClient.connected()) {
        if (telnetClient) telnetClient.stop();
        telnetClient = telnetServer.available();
      }
    }

    // If data comes in from the Serial monitor, send it to the Telnet client
    if (Serial.available() > 0) {
      String serialData = Serial.readString();
      if (telnetClient && telnetClient.connected()) {
        TelnetStream.println("Hello Linux Terminal!");
        TelnetStream.println(serialData);
        TelnetStream.flush();
      }
    }
  #endif
}

//  stopMotors();
//  if (millis() - lastCmdTime > CMD_TIMEOUT) {
//    stopMotors();
//  }

