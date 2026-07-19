/*
  NodeMCU (ESP8266) Robo-Soccer Car — WiFi joystick version
  ------------------------------------------------------------
  Differential drive (2 DC motors via L298N) + caster wheel.
  Control: NodeMCU hosts its own WiFi Access Point + a webpage
  with a touch joystick. Joystick position streams over WebSocket
  for low-latency control. No Dabble, no external BT module needed
  - Dabble does not support a WiFi transport, so this sketch talks
  directly to a browser instead.

  LIBRARIES NEEDED (install via Arduino Library Manager):
    - "ESPAsyncWebServer" (search for the ESP32Async fork - it also
       supports ESP8266)
    - "ESPAsyncTCP"  <-- IMPORTANT: this is the ESP8266 dependency.
       (On ESP32 the equivalent dependency is "AsyncTCP" - different
       library, don't mix them up.)

  WIRING (see schematic - matches the earlier Bluetooth version's
  pin map, just without the HC-05 module):
    L298N ENA -> D5 (GPIO14)  left motor PWM
    L298N IN1 -> D6 (GPIO12)
    L298N IN2 -> D7 (GPIO13)
    L298N IN3 -> D3 (GPIO0)
    L298N IN4 -> D0 (GPIO16)
    L298N ENB -> D4 (GPIO2)   right motor PWM
    L298N GND -> NodeMCU GND (COMMON GROUND - required)
    NodeMCU powered from a separate 5V buck converter with a
    decoupling capacitor (100-470uF) across 5V/GND, NOT the L298N's
    onboard regulator - WiFi draws current spikes that a shared
    regulator often can't supply cleanly, causing brownout resets.
*/

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ---------- WiFi Access Point settings ----------
const char* AP_SSID = "sevenSeven";
const char* AP_PASS = "roboRobo37";   // must be 8+ chars
const int   AP_CHANNEL = 13;          // fix the WiFi channel here (1-11 typical, 1-13 in some regions)

// ---------- Motor pins ----------
#define ENA D5   // left motor PWM (enable)
#define IN1 D6   // left motor direction
#define IN2 D7
#define IN3 D3   // right motor direction
#define IN4 D0
#define ENB D4   // right motor PWM (enable)

const int MIN_PWM = 50;   // minimum PWM to overcome motor static friction - tune this

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------- Motor control ----------------
void setLeftMotor(int speed) {           // speed: -255..255
  speed = constrain(speed, -255, 255);
  digitalWrite(IN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(IN2, speed >= 0 ? LOW  : HIGH);
  int pwm = abs(speed);
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  analogWrite(ENA, pwm);
}

void setRightMotor(int speed) {          // speed: -255..255
  speed = constrain(speed, -255, 255);
  digitalWrite(IN3, speed >= 0 ? HIGH : LOW);
  digitalWrite(IN4, speed >= 0 ? LOW  : HIGH);
  int pwm = abs(speed);
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  analogWrite(ENB, pwm);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
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
  #pad{width:340px;height:340px;border-radius:50%;background:#222;border:3px solid #444;
    position:relative;touch-action:none;}
  #stick{width:80px;height:80px;border-radius:50%;background:#3a8;position:absolute;
    left:130px;top:130px;box-shadow:0 0 15px #3a8;}
</style>
</head>
<body>
<div id="status">connecting...</div>
<div id="pad"><div id="stick"></div></div>
<script>
  const pad = document.getElementById('pad');
  const stick = document.getElementById('stick');
  const status = document.getElementById('status');
  const R = 170, stickR = 40;   // R = pad radius (340/2), stickR = stick radius (80/2)
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

  // Tweak these two to change how sensitive the control feels:
  const DEADZONE = 0.15;   // 0-1: ignore movement within this fraction of center
  const EXPO = 0.6;        // 0-1: 0 = fully linear, closer to 1 = softer near center

  function shape(v){
    // apply deadzone
    if (Math.abs(v) < DEADZONE) return 0;
    // rescale so output still reaches -1..1 smoothly past the deadzone
    const sign = v < 0 ? -1 : 1;
    const mag = (Math.abs(v) - DEADZONE) / (1 - DEADZONE);
    // expo blend: small inputs get suppressed more, full stick still hits 1.0
    const shaped = EXPO*mag*mag*mag + (1-EXPO)*mag;
    return sign*shaped;
  }

  function handlePos(px, py){
    const rect = pad.getBoundingClientRect();
    let dx = px - (rect.left + R);
    let dy = py - (rect.top + R);
    let dist = Math.sqrt(dx*dx+dy*dy);
    if(dist > R - stickR){ const s=(R-stickR)/dist; dx*=s; dy*=s; }
    stick.style.left = (130+dx)+'px';
    stick.style.top  = (130+dy)+'px';
    x = shape(dx/(R-stickR));
    y = shape(dy/(R-stickR)); // changed from -y invert so up = forward
  }
  function reset(){
    x=0; y=0; stick.style.left='130px'; stick.style.top='130px'; sendCmd();
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
    String msg((char*)data);
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
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  stopMotors();

  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
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