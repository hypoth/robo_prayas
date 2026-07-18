/*
  ESP32 Robo-Soccer Car — Bluetooth LE version (Dabble app joystick)
  ------------------------------------------------------------------
  Differential drive (2 DC motors via L298N) + caster wheel.
  Control: Android phone running the free "Dabble" app (STEMpedia),
  using its Gamepad module in Joystick mode, over BLE.

  WHY BLE INSTEAD OF WIFI:
  BLE draws far less current than WiFi (no access point, no web
  server, no continuous TCP traffic) which reduces the chance of
  brownout resets on a marginal power supply. Still use a proper
  dedicated 5V regulator + decoupling capacitor for the ESP32 -
  BLE reduces the trigger, it doesn't replace clean power delivery.

  LIBRARY NEEDED (install via Arduino Library Manager):
    Search "Dabble" -> install "DabbleESP32" by STEMpedia

  ANDROID APP:
    Install "Dabble" from the Play Store (STEMpedia).
    Open it, tap "Gamepad", switch to Joystick mode (icon top-right),
    and pair with the Bluetooth name set below in Dabble.begin().

  WIRING (same as before):
    L298N ENA -> GPIO 14   (left motor PWM)
    L298N IN1 -> GPIO 27
    L298N IN2 -> GPIO 26
    L298N IN3 -> GPIO 25
    L298N IN4 -> GPIO 33
    L298N ENB -> GPIO 32   (right motor PWM)
    L298N GND -> ESP32 GND (COMMON GROUND - required)
    ESP32 powered from a separate 5V buck converter with a decoupling
    capacitor (100-470uF) across 5V/GND, NOT the L298N's onboard
    regulator.
*/

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

// ---------- Motor pins ----------
#define ENA 14   // left motor PWM (enable)
#define IN1 27   // left motor direction
#define IN2 26
#define IN3 25   // right motor direction
#define IN4 33
#define ENB 32   // right motor PWM (enable)

// PWM (LEDC) config
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;      // 8-bit -> 0-255
const int LEFT_CH   = 0;
const int RIGHT_CH  = 1;

// Minimum PWM to actually overcome motor static friction (tune this!)
const int MIN_PWM = 60;

// ---------------- Motor control ----------------
void setLeftMotor(int speed) {           // speed: -255..255
  speed = constrain(speed, -255, 255);
  digitalWrite(IN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(IN2, speed >= 0 ? LOW  : HIGH);
  int pwm = abs(speed);
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  ledcWrite(LEFT_CH, pwm);
}

void setRightMotor(int speed) {          // speed: -255..255
  speed = constrain(speed, -255, 255);
  digitalWrite(IN3, speed >= 0 ? HIGH : LOW);
  digitalWrite(IN4, speed >= 0 ? LOW  : HIGH);
  int pwm = abs(speed);
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  ledcWrite(RIGHT_CH, pwm);
}

void stopMotors() {
  ledcWrite(LEFT_CH, 0);
  ledcWrite(RIGHT_CH, 0);
}

// Arcade-style mixing: x,y in range -1.0 .. 1.0
void drive(float x, float y) {
  int fwd  = y * 255;
  int turn = x * 255;
  int left  = constrain(fwd + turn, -255, 255);
  int right = constrain(fwd - turn, -255, 255);
  setLeftMotor(left);
  setRightMotor(right);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcSetup(LEFT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, LEFT_CH);
  ledcSetup(RIGHT_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, RIGHT_CH);

  stopMotors();

  Dabble.begin("RoboSoccerCar");   // this is the Bluetooth name you'll see in the Dabble app
  Serial.println("Waiting for Dabble app to connect over Bluetooth...");
}

void loop() {
  Dabble.processInput();   // must be called every loop to refresh joystick data

  // Dabble's joystick reports -7..7 on each axis, 0 at center.
  float x = GamePad.getXaxisData() / 7.0;
  float y = GamePad.getYaxisData() / 7.0;

  drive(x, y);
  // If the phone disconnects, GamePad values fall back to 0, so the
  // car naturally stops - no extra failsafe timer needed here.
}
