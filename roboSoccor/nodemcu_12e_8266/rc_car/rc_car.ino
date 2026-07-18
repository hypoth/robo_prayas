/* ============================================================

   ESP32 RoboSoccer Bot - Full Firmware with Web Controller

   Prayas 4.0 - IIT Mandi

   ============================================================

   Hardware: ESP32 DevKit, L298N, 2x DC motors, 2x SG90 servo,

             1x HC-SR04 ultrasonic sensor, 1x castor wheel

   ============================================================ */

#include <WiFi.h>

#include <WebServer.h>

#include <ESP32Servo.h>

// ---------- 1. WIFI SETTINGS - CHANGE THESE IF YOU LIKE ----------

const char* ap_ssid     = "RoboSoccer_";   // <-- CHANGE bot WiFi name here

const char* ap_password = "12345678";          // <-- CHANGE bot WiFi password here (min 8 chars)

// ---------- 2. PIN DEFINITIONS ----------

// Left motor (Motor A)

#define IN1 27

#define IN2 26

#define ENA 14

// Right motor (Motor B)

#define IN3 25

#define IN4 33

#define ENB 12

// Servos

#define SERVO1_PIN 13

#define SERVO2_PIN 4

// Ultrasonic

#define TRIG_PIN 5

#define ECHO_PIN 18

// ---------- 3. PWM (speed) SETTINGS ----------

const int pwmFreq = 1000;

const int pwmResolution = 8;      // 8-bit -> values 0-255

const int leftChannel = 0;

const int rightChannel = 1;

int motorSpeed = 200;             // default speed, 0-255 (adjustable from webpage)

WebServer server(80);

Servo servo1;

Servo servo2;

// ================= MOTOR CONTROL FUNCTIONS =================

void stopMotors() {

  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

}

void moveForward() {

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

}

void moveBackward() {

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

}

void turnLeft() {

  // left motor backward, right motor forward -> spin left

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

}

void turnRight() {

  // left motor forward, right motor backward -> spin right

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

}

// ================= ULTRASONIC =================

long readDistanceCM() {

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return -1; // no echo received (out of range)

  return duration * 0.034 / 2;

}

// ================= WEBPAGE (HTML+CSS+JS) =================

String htmlPage() {

  String html = R"HTML(

<!DOCTYPE html>

<html>

<head>

<title>RoboSoccer Bot Controller</title>

<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">

<style>

  body { font-family: Arial, sans-serif; background:#111; color:#fff; text-align:center; margin:0; padding:10px; }

  h1 { color:#0f0; font-size:22px; }

  .distance { font-size:20px; color:#0ff; margin:10px 0; }

  .row { display:flex; justify-content:center; margin:8px 0; }

  button {

    width:90px; height:70px; margin:6px; font-size:16px; font-weight:bold;

    border-radius:12px; border:none; background:#0a84ff; color:white; touch-action:manipulation;

  }

  button:active { background:#065bb5; }

  .stop { background:#e63946; }

  .kick { background:#ff9500; width:120px; }

  .speedbox { margin:20px auto; width:80%; }

  input[type=range] { width:100%; }

</style>

</head>

<body>

  <h1>RoboSoccer Bot Controller</h1>

  <div class="distance">Distance: <span id="dist">--</span> cm</div>

  <div class="row">

    <button ontouchstart="send('F')" ontouchend="send('S')" onmousedown="send('F')" onmouseup="send('S')">FORWARD</button>

  </div>

  <div class="row">

    <button ontouchstart="send('L')" ontouchend="send('S')" onmousedown="send('L')" onmouseup="send('S')">LEFT</button>

    <button class="stop" onclick="send('S')">STOP</button>

    <button ontouchstart="send('R')" ontouchend="send('S')" onmousedown="send('R')" onmouseup="send('S')">RIGHT</button>

  </div>

  <div class="row">

    <button ontouchstart="send('B')" ontouchend="send('S')" onmousedown="send('B')" onmouseup="send('S')">BACKWARD</button>

  </div>

  <div class="row">

    <button class="kick" onclick="send('K1')">KICK 1</button>

    <button class="kick" onclick="send('K2')">KICK 2</button>

  </div>

  <div class="speedbox">

    <label>Speed: <span id="spdval">200</span></label>

    <input type="range" min="100" max="255" value="200" id="spd" onchange="setSpeed(this.value)">

  </div>

<script>

function send(cmd) {

  fetch('/cmd?c=' + cmd);

}

function setSpeed(v) {

  document.getElementById('spdval').innerText = v;

  fetch('/speed?v=' + v);

}

function updateDistance() {

  fetch('/distance').then(r => r.text()).then(d => {

    document.getElementById('dist').innerText = d;

  });

}

setInterval(updateDistance, 1000); // refresh distance every 1 second

</script>

</body>

</html>

)HTML";

  return html;

}
// ================= WEB SERVER ROUTE HANDLERS =================

void handleRoot() {

  server.send(200, "text/html", htmlPage());

}

void handleCmd() {

  String c = server.arg("c");

  if (c == "F") moveForward();

  else if (c == "B") moveBackward();

  else if (c == "L") turnLeft();

  else if (c == "R") turnRight();

  else if (c == "S") stopMotors();

  else if (c == "K1") {

    servo1.write(90);   // kick position - ADJUST ANGLES TO MATCH YOUR KICKER ARM

    delay(300);

    servo1.write(0);    // return position

  }

  else if (c == "K2") {

    servo2.write(90);

    delay(300);

    servo2.write(0);

  }

  server.send(200, "text/plain", "OK");

}

void handleSpeed() {

  String v = server.arg("v");

  motorSpeed = v.toInt();

  ledcWrite(leftChannel, motorSpeed);

  ledcWrite(rightChannel, motorSpeed);

  server.send(200, "text/plain", "OK");

}

void handleDistance() {

  long d = readDistanceCM();

  server.send(200, "text/plain", String(d));

}

// ================= SETUP =================

void setup() {

  Serial.begin(115200);

  // Motor pins

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopMotors();

  // PWM setup for speed control (ESP32 LEDC peripheral)

  ledcSetup(leftChannel, pwmFreq, pwmResolution);

  ledcAttachPin(ENA, leftChannel);

  ledcSetup(rightChannel, pwmFreq, pwmResolution);

  ledcAttachPin(ENB, rightChannel);

  ledcWrite(leftChannel, motorSpeed);

  ledcWrite(rightChannel, motorSpeed);

  // Servos

  servo1.attach(SERVO1_PIN);

  servo2.attach(SERVO2_PIN);

  servo1.write(0);

  servo2.write(0);

  // Ultrasonic

  pinMode(TRIG_PIN, OUTPUT);

  pinMode(ECHO_PIN, INPUT);

  // WiFi Access Point

  WiFi.softAP(ap_ssid, ap_password);

  Serial.println("=================================");

  Serial.print("Connect your phone to WiFi: ");

  Serial.println(ap_ssid);

  Serial.print("Then open this in your browser: http://");

  Serial.println(WiFi.softAPIP());

  Serial.println("=================================");

  // Web server routes

  server.on("/", handleRoot);

  server.on("/cmd", handleCmd);

  server.on("/speed", handleSpeed);

  server.on("/distance", handleDistance);

  server.begin();

  Serial.println("Web server started!");

}

// ================= LOOP =================

void loop() {

  server.handleClient();

}
