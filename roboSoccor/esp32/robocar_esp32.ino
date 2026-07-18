/*
  ESP32 Robo-Soccer Car
  ---------------------
  Differential drive (2 DC motors via L298N) + caster wheel.
  Control: ESP32 hosts its own WiFi Access Point + a webpage with a
  touch joystick. Joystick position streams over WebSocket for
  low-latency control.

  LIBRARIES NEEDED (install via Arduino Library Manager):
    - "ESP32Async / ESPAsyncWebServer" (search "ESPAsyncWebServer")
    - "ESP32Async / AsyncTCP"          (search "AsyncTCP")

  WIRING (see chat for full explanation):
    L298N ENA -> GPIO 14   (left motor PWM)
    L298N IN1 -> GPIO 27
    L298N IN2 -> GPIO 26
    L298N IN3 -> GPIO 25
    L298N IN4 -> GPIO 33
    L298N ENB -> GPIO 32   (right motor PWM)
    L298N GND -> ESP32 GND (COMMON GROUND - required)
    ESP32 powered from a separate 5V buck converter, NOT the L298N's
    onboard regulator (avoids brownout resets from WiFi current spikes).
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// ---------- WiFi Access Point settings ----------
const char* AP_SSID = "RoboSoccerCar";
const char* AP_PASS = "soccer123";   // must be 8+ chars

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

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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

// Arcade-style mixing: joystick x,y in range -1.0 .. 1.0
void drive(float x, float y) {
  int fwd  = y * 255;
  int turn = x * 255;
  int left  = constrain(fwd + turn, -255, 255);
  int right = constrain(fwd - turn, -255, 255);
  setLeftMotor(left);
  setRightMotor(right);
}

// Fail-safe: if no command received for 400ms, stop (avoids a "runaway" car if WiFi drops)
unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT = 400;

// ---------------- Webpage (joystick UI) ----------------
const char htmlPage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>RoboSoccer Control</title>
<style>
  html,body{height:100%;margin:0;background:#111;color:#eee;font-family:sans-serif;
    display:flex;flex-direction:column;align-items:center;justify-content:center;touch-action:none;}
  #status{position:absolute;top:10px;font-size:14px;color:#8f8;}
  #pad{width:280px;height:280px;border-radius:50%;background:#222;border:3px solid #444;
    position:relative;touch-action:none;}
  #stick{width:90px;height:90px;border-radius:50%;background:#3a8;position:absolute;
    left:95px;top:95px;box-shadow:0 0 15px #3a8;}
</style>
</head>
<body>
<div id="status">connecting...</div>
<div id="pad"><div id="stick"></div></div>
<script>
  const pad = document.getElementById('pad');
  const stick = document.getElementById('stick');
  const status = document.getElementById('status');
  const R = 140, stickR = 45;
  let ws, x=0, y=0, dragging=false;

  function connect(){
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = ()=> status.textContent = 'connected';
    ws.onclose = ()=> { status.textContent = 'disconnected - retrying'; setTimeout(connect,1000); };
  }
  connect();

  function sendCmd(){
    if(ws && ws.readyState===1) ws.send(x.toFixed(2)+','+y.toFixed(2));
  }
  setInterval(sendCmd, 60); // stream at ~16Hz

  function handlePos(px, py){
    const rect = pad.getBoundingClientRect();
    let dx = px - (rect.left + R);
    let dy = py - (rect.top + R);
    let dist = Math.sqrt(dx*dx+dy*dy);
    if(dist > R - stickR){ const s=(R-stickR)/dist; dx*=s; dy*=s; }
    stick.style.left = (95+dx)+'px';
    stick.style.top  = (95+dy)+'px';
    x = dx/(R-stickR);
    y = -dy/(R-stickR); // invert so up = forward
  }
  function reset(){
    x=0; y=0; stick.style.left='95px'; stick.style.top='95px'; sendCmd();
  }

  pad.addEventListener('pointerdown', e=>{ dragging=true; handlePos(e.clientX,e.clientY); });
  window.addEventListener('pointermove', e=>{ if(dragging) handlePos(e.clientX,e.clientY); });
  window.addEventListener('pointerup', ()=>{ dragging=false; reset(); });
</script>
</body>
</html>
)HTML";

// ---------------- WebSocket handler ----------------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    String msg((char*)data, len);
    int comma = msg.indexOf(',');
    if (comma > 0) {
      float x = msg.substring(0, comma).toFloat();
      float y = msg.substring(comma + 1).toFloat();
      drive(x, y);
      lastCmdTime = millis();
    }
  } else if (type == WS_EVT_DISCONNECT) {
    stopMotors();
  }
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

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started. Connect phone to WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Then open http://");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", htmlPage);
  });

  server.begin();
}

void loop() {
  ws.cleanupClients();
  // Fail-safe: stop if no command received recently (e.g. WiFi hiccup)
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    stopMotors();
  }
}
