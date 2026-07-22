/*
  Wall Following + RC Drive Robot — NodeMCU ESP8266 (ESP-12E)

  Hosts a web page (open http://<esp-ip>/ in a browser on the same
  hotspot network) with two modes:
    - Wall Follow: autonomous PID wall-following, gains/setpoints tunable live
    - RC Drive   : manual on-screen D-pad control

  A WebSocket carries commands in and telemetry out in real time.
  Raw sensor data can still be captured via the existing UDP broadcast +
  udp_logger.py, now gated by a Capture toggle in the web UI.
*/

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>

// ---------------- WiFi config ----------------
const char* WIFI_SSID = "lp";
const char* WIFI_PASS = "lappyLappy1";

// ---------------- UDP telemetry / capture config ----------------
const bool USE_BROADCAST = true;
IPAddress PC_IP(192, 168, 137, 1);   //using broadcast as it is easier, else need to assign static IP to device capturing the data
const uint16_t PC_PORT = 4210;

WiFiUDP udp;
unsigned long udpSeq = 0;
IPAddress broadcastIP;
bool captureEnabled = false; // by default capture is disabled

// ---------------- Web server / websocket ----------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
unsigned long wsSeq = 0;

// ---------------- Pins ----------------
#define ENA D1
#define IN1 D0
#define IN2 D3
#define ENB D2
#define IN3 D4
#define IN4 D8

#define TRIG_SIDE  D7
#define ECHO_SIDE  D5
#define TRIG_FRONT D7    // D7 is easier as GPIO3 - use is clumsy, need to  disconnect before flashing
#define ECHO_FRONT D6

bool  FOLLOW_RIGHT_WALL = false;
float LOST_WALL_CM = 60.0;
int   PIVOT_SPEED  = 65;

// ---------------- Tunable parameters (live-adjustable via UI) ----------------
float Kp = 6.0;
float Ki = 0.02;
float Kd = 3.0;
float SETPOINT_CM   = 20.0;
float FRONT_STOP_CM = 20.0;
int   BASE_SPEED    = 65;
int   MAX_SPEED     = 150;

// ---------------- Mode + RC state ----------------
enum RobotMode { MODE_RC, MODE_WALL_FOLLOW };
RobotMode currentMode = MODE_RC; // start in RC mode: robot won't move on its own at boot

int rcLeftSpeed = 0;
int rcRightSpeed = 0;
unsigned long lastRcCommandTime = 0;
const unsigned long RC_TIMEOUT_MS = 500; // stop motors if no RC command received recently

float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

bool wallFollowActive = false; // motors stay at 0 in WALL mode until this is true

unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 50; // this is interval between two successive sensor readings.
// only min is confirmed, worst case max delay can go beyond 50 is processor is busy with any other blocking calls.
// to reduce the jitter since the data capture is also being done

// ---------------- Web UI (served from flash, no filesystem needed) ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Control</title>
<style>
  body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:12px;}
  h1{font-size:1.1em;text-align:center;margin:6px 0 14px;}
  .tabs{display:flex;gap:8px;margin-bottom:12px;}
  .tab{flex:1;padding:10px;text-align:center;background:#333;border-radius:6px;cursor:pointer;}
  .tab.active{background:#0a7;}
  .panel{display:none;}
  .panel.active{display:block;}
  .row{display:flex;justify-content:space-between;margin:8px 0;align-items:center;}
  input[type=number]{width:90px;background:#222;color:#eee;border:1px solid #555;padding:6px;border-radius:4px;}
  button{padding:10px;background:#0a7;color:#fff;border:none;border-radius:6px;font-size:1em;}
  #estop{background:#c33;width:100%;padding:16px;font-size:1.2em;margin-bottom:14px;}
  #applyParams{width:100%;margin-top:6px;}
  .dpad{display:grid;grid-template-columns:74px 74px 74px;grid-gap:8px;justify-content:center;margin:20px 0;}
  .dpad button{width:74px;height:74px;font-size:1.6em;user-select:none;-webkit-user-select:none;}
  #telemetry{background:#222;padding:10px;border-radius:6px;font-family:monospace;font-size:0.82em;margin-top:14px;line-height:1.6;}
  #captureBtn{width:100%;margin-top:10px;background:#555;}
  #captureBtn.active{background:#f90;}
</style>
</head>65
<body>
<h1>Robot Control</h1>
<button id="estop">STOP</button>

<div class="tabs">
  <div class="tab" data-tab="rc">RC Drive</div>
  <div class="tab" data-tab="wall">Wall Follow</div>
</div>

<div id="panel-wall" class="panel">
  <div class="row"><label>Kp</label><input id="kp" type="number" step="0.1" value="6.0"></div>
  <div class="row"><label>Ki</label><input id="ki" type="number" step="0.01" value="0.02"></div>
  <div class="row"><label>Kd</label><input id="kd" type="number" step="0.1" value="3.0"></div>
  <div class="row"><label>Setpoint (cm)</label><input id="setpoint" type="number" value="20"></div>
  <div class="row"><label>Base Speed</label><input id="baseSpeed" type="number" value="65"></div>
  <div class="row"><label>Front Stop (cm)</label><input id="frontStop" type="number" value="20"></div>
  <div class="row"><label>Max Speed</label><input id="maxSpeed" type="number" value="100"></div>
  <div class="row"><label>Follow Side</label>
    <select id="followRight" style="background:#222;color:#eee;border:1px solid #555;padding:6px;border-radius:4px;">
      <option value="1">Right wall</option>
      <option value="0">Left wall</option>
    </select>
  </div>
  <div class="row"><label>Lost Wall (cm)</label><input id="lostWall" type="number" value="60"></div>
  <div class="row"><label>Pivot Speed</label><input id="pivotSpeed" type="number" value="65"></div>
  <button id="applyParams">Apply</button>
  <button id="wallRunBtn" style="width:100%;margin-top:10px;background:#0a7;">Start Wall Follow</button>
</div>

<div id="panel-rc" class="panel active">
  <div class="row"><label>RC Speed</label><input id="rcSpeed" type="number" value="100"></div>
  <div class="dpad">
    <div></div><button data-l="1" data-r="1">&#8593;</button><div></div>
    <button data-l="-1" data-r="1">&#8630;</button><button data-l="0" data-r="0">&#9632;</button><button data-l="1" data-r="-1">&#8631;</button>
    <div></div><button data-l="-1" data-r="-1">&#8595;</button><div></div>
  </div>
</div>

<button id="captureBtn">Start Capture</button>
<div id="telemetry">waiting for data...</div>

<script>
let ws;
let capturing = false;
let wallRunning = false;

function setWallRunBtn(running) {
  wallRunning = running;
  const btn = document.getElementById("wallRunBtn");
  btn.innerText = running ? "Stop Wall Follow" : "Start Wall Follow";
  btn.style.background = running ? "#c33" : "#0a7";
}

function connect() {
  ws = new WebSocket("ws://" + location.host + "/ws");
  ws.onmessage = (evt) => {
    const d = JSON.parse(evt.data);
    document.getElementById("telemetry").innerText =
      "mode=" + d.mode + "  running=" + d.running +
      "\nside=" + d.side + "cm  front=" + d.front + "cm" +
      "\nKp=" + d.kp + " Ki=" + d.ki + " Kd=" + d.kd +
      "\ndt=" + d.dt.toFixed(3) + "  err=" + d.error.toFixed(2) +
      "\nintegral=" + d.integral.toFixed(2) + "  derivative=" + d.derivative.toFixed(2) +
      "\noutput=" + d.output.toFixed(2) +
      "\nL=" + d.left + "  R=" + d.right + "\ncapture=" + d.capture;
  };
  ws.onclose = () => setTimeout(connect, 1000);
}
connect();

function send(obj) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}

document.querySelectorAll(".tab").forEach(t => {
  t.onclick = () => {
    document.querySelectorAll(".tab").forEach(x => x.classList.remove("active"));
    document.querySelectorAll(".panel").forEach(x => x.classList.remove("active"));
    t.classList.add("active");
    document.getElementById("panel-" + t.dataset.tab).classList.add("active");
    send({type: "mode", value: t.dataset.tab === "rc" ? "RC" : "WALL"});
    if (t.dataset.tab === "wall") {
      setWallRunBtn(false); // always land in "stopped" so params can be set safely first
      send({type: "wallRun", value: false});
    }
  };
});

document.getElementById("wallRunBtn").onclick = () => {
  setWallRunBtn(!wallRunning);
  send({type: "wallRun", value: wallRunning});
};

document.getElementById("estop").onclick = () => {
  document.querySelector('.tab[data-tab="rc"]').click();
  setWallRunBtn(false);
  send({type: "drive", left: 0, right: 0});
};

document.getElementById("applyParams").onclick = () => {
  send({
    type: "params",
    kp: parseFloat(document.getElementById("kp").value),
    ki: parseFloat(document.getElementById("ki").value),
    kd: parseFloat(document.getElementById("kd").value),
    setpoint: parseFloat(document.getElementById("setpoint").value),
    baseSpeed: parseInt(document.getElementById("baseSpeed").value),
    frontStop: parseFloat(document.getElementById("frontStop").value),
    maxSpeed: parseInt(document.getElementById("maxSpeed").value),
    followRight: parseInt(document.getElementById("followRight").value) === 1,
    lostWall: parseFloat(document.getElementById("lostWall").value),
    pivotSpeed: parseInt(document.getElementById("pivotSpeed").value)
  });
};

document.querySelectorAll(".dpad button").forEach(btn => {
  const go = () => {
    const spd = parseInt(document.getElementById("rcSpeed").value);
    send({type: "drive", left: parseInt(btn.dataset.l) * spd, right: parseInt(btn.dataset.r) * spd});
  };
  const stop = () => send({type: "drive", left: 0, right: 0});
  btn.addEventListener("mousedown", go);
  btn.addEventListener("touchstart", (e) => { e.preventDefault(); go(); });
  btn.addEventListener("mouseup", stop);
  btn.addEventListener("mouseleave", stop);
  btn.addEventListener("touchend", stop);
});

document.getElementById("captureBtn").onclick = () => {
  capturing = !capturing;
  send({type: "capture", value: capturing});
  document.getElementById("captureBtn").innerText = capturing ? "Stop Capture" : "Start Capture";
  document.getElementById("captureBtn").classList.toggle("active", capturing);
};
</script>
</body>
</html>
)rawliteral";

// ---------------- Sensor + motor helpers ----------------
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1;
  return duration / 58;
}

void setMotor(int enaPin, int in1, int in2, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    speed = -speed;
  }
  analogWrite(enaPin, speed);
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(ENB, IN3, IN4, leftSpeed);
  setMotor(ENA, IN1, IN2, rightSpeed);
}
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------------- Telemetry ----------------
void sendTelemetryWs(long sideCM, long frontCM, float error, float dt, float derivative,
                      float output, int leftSpeed, int rightSpeed, bool running) {
  if (ws.count() == 0) return;
  StaticJsonDocument<384> doc;
  doc["seq"] = wsSeq++;
  doc["t"] = millis();
  doc["mode"] = (currentMode == MODE_RC) ? "RC" : "WALL";
  doc["running"] = running;
  doc["side"] = sideCM;
  doc["front"] = frontCM;
  doc["kp"] = Kp;
  doc["ki"] = Ki;
  doc["kd"] = Kd;
  doc["dt"] = dt;
  doc["error"] = error;
  doc["integral"] = integral;
  doc["derivative"] = derivative;
  doc["output"] = output;
  doc["left"] = leftSpeed;
  doc["right"] = rightSpeed;
  doc["capture"] = captureEnabled;

  char buf[384];
  size_t len = serializeJson(doc, buf);
  ws.textAll(buf, len);
}

void sendTelemetryUdp(long sideCM, long frontCM, float error, float dt, float derivative,
                       float output, int leftSpeed, int rightSpeed, bool running) {
  if (WiFi.status() != WL_CONNECTED) return;
  char buf[220];
  int len = snprintf(buf, sizeof(buf),
                      "%lu,%lu,%s,%d,%ld,%ld,%.3f,%.3f,%.3f,%.4f,%.2f,%.2f,%.2f,%.2f,%d,%d",
                      udpSeq++, millis(), (currentMode == MODE_RC) ? "RC" : "WALL", running,
                      sideCM, frontCM, Kp, Ki, Kd, dt, error, integral, derivative, output,
                      leftSpeed, rightSpeed);
  udp.beginPacket(USE_BROADCAST ? broadcastIP : PC_IP, PC_PORT);
  udp.write((const uint8_t*)buf, len);
  udp.endPacket();
}

// ---------------- WebSocket command handling ----------------
void handleWsMessage(char* msg) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg)) return;

  const char* type = doc["type"];
  if (!type) return;

  if (strcmp(type, "drive") == 0) {
    rcLeftSpeed = doc["left"] | 0;
    rcRightSpeed = doc["right"] | 0;
    lastRcCommandTime = millis();
  } else if (strcmp(type, "mode") == 0) {
    const char* m = doc["value"] | "";
    if (strcmp(m, "RC") == 0) {
      currentMode = MODE_RC;
      rcLeftSpeed = 0;
      rcRightSpeed = 0;
      wallFollowActive = false;
    } else if (strcmp(m, "WALL") == 0) {
      currentMode = MODE_WALL_FOLLOW;
      wallFollowActive = false; // always start stopped - must press Start explicitly
      integral = 0;
      lastError = 0;
      lastTime = millis(); // avoid a dt spike from time spent in RC mode
    }
  } else if (strcmp(type, "wallRun") == 0) {
    wallFollowActive = doc["value"] | false;
  } else if (strcmp(type, "params") == 0) {
    if (doc.containsKey("kp")) Kp = doc["kp"];
    if (doc.containsKey("ki")) Ki = doc["ki"];
    if (doc.containsKey("kd")) Kd = doc["kd"];
    if (doc.containsKey("setpoint")) SETPOINT_CM = doc["setpoint"];
    if (doc.containsKey("baseSpeed")) BASE_SPEED = doc["baseSpeed"];
    if (doc.containsKey("frontStop")) FRONT_STOP_CM = doc["frontStop"];
    if (doc.containsKey("maxSpeed")) MAX_SPEED = doc["maxSpeed"];
    if (doc.containsKey("followRight")) FOLLOW_RIGHT_WALL = doc["followRight"];
    if (doc.containsKey("lostWall")) LOST_WALL_CM = doc["lostWall"];
    if (doc.containsKey("pivotSpeed")) PIVOT_SPEED = doc["pivotSpeed"];
  } else if (strcmp(type, "capture") == 0) {
    captureEnabled = doc["value"] | false;
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      handleWsMessage((char*)data);
    }
  }
}

// ---------------- Main control cycle (called every SAMPLE_INTERVAL_MS) ----------------
void doSensorSampleAndControl() {
  long sideCM  = readDistanceCM(TRIG_SIDE, ECHO_SIDE);
  delay(15);
  long frontCM = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
  delay(15);

  float error = 0;
  float output = 0;
  float dt = 0;
  float derivative = 0;
  int leftSpeed = 0;
  int rightSpeed = 0;
  bool running = false;

  if (currentMode == MODE_RC) {
    running = true; // motors always follow the driver's commands in RC mode
    if (millis() - lastRcCommandTime > RC_TIMEOUT_MS) {
      rcLeftSpeed = 0;
      rcRightSpeed = 0;
    }
    leftSpeed = rcLeftSpeed;
    rightSpeed = rcRightSpeed;
    drive(leftSpeed, rightSpeed);

  } else { // MODE_WALL_FOLLOW
    running = wallFollowActive;
    bool frontBlocked = (frontCM > 0 && frontCM < FRONT_STOP_CM);
    long effSide = (sideCM < 0) ? (long)LOST_WALL_CM : sideCM;
    error = SETPOINT_CM - effSide;

    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    lastTime = now;

    if (frontBlocked) {
      // reset PID state while pivoting so it doesn't wind up against a wall we're avoiding
      integral = 0;
      derivative = 0;
      lastError = 0;
    } else {
      integral += error * dt;
      integral = constrain(integral, -50, 50);
      derivative = (error - lastError) / dt;
      lastError = error;
    }

    output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, (float)-BASE_SPEED, (float)BASE_SPEED);

    // Compute what the motors *would* do - always, so telemetry reflects live PID behavior
    if (frontBlocked) {
      if (FOLLOW_RIGHT_WALL) { leftSpeed = -PIVOT_SPEED; rightSpeed = PIVOT_SPEED; }
      else                   { leftSpeed = PIVOT_SPEED;  rightSpeed = -PIVOT_SPEED; }
    } else {
      if (FOLLOW_RIGHT_WALL) {
        leftSpeed  = BASE_SPEED - output;
        rightSpeed = BASE_SPEED + output;
      } else {
        leftSpeed  = BASE_SPEED + output;
        rightSpeed = BASE_SPEED - output;
      }
    }

    // Only actually move the motors once Start has been pressed
    if (wallFollowActive) {
      drive(leftSpeed, rightSpeed);
    } else {
      drive(0, 0);
    }
  }

  sendTelemetryWs(sideCM, frontCM, error, dt, derivative, output, leftSpeed, rightSpeed, running);
  if (captureEnabled) sendTelemetryUdp(sideCM, frontCM, error, dt, derivative, output, leftSpeed, rightSpeed, running);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_SIDE, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_SIDE, INPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  analogWriteRange(255);
  analogWriteFreq(1000);
  stopMotors();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected. Open this in a browser: http://");
    Serial.println(WiFi.localIP());

    IPAddress ip = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();
    for (int i = 0; i < 4; i++) broadcastIP[i] = ip[i] | (~mask[i] & 0xFF);
    Serial.print("UDP broadcast address: ");
    Serial.println(broadcastIP);

    udp.begin(PC_PORT);
  } else {
    Serial.println("\nWiFi FAILED - web UI and telemetry will not work.");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
  lastTime = millis();
}

void loop() {
  ws.cleanupClients();

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    doSensorSampleAndControl();
  }
}
