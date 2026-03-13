// NodeMCU V3 - CRSF TX Controller with Telemetry
// TX -> FC R2 (CRSF out), RX <- FC T2 (telemetry in)
// Connect to WiFi AP "FC-Monitor" and open http://192.168.4.1

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <AlfredoCRSF.h>
#include "flash_light.h"

const char* AP_SSID = "FC-Controller";
const char* AP_PASS = "12345678";

#define FLASH_LIGHT_IN1 D2
#define FLASH_LIGHT_IN2 D5
FlashLight flashLight(FLASH_LIGHT_IN1, FLASH_LIGHT_IN2);

AlfredoCRSF crsf;
ESP8266WebServer server(80);

crsf_channels_t crsfChannels = {0};
uint32_t lastCrsfTime = 0;

// Channel values (1000-2000 range)
int channels[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

// Telemetry data
bool telemetryReceived = false;
uint32_t lastTelemetryTime = 0;
uint32_t framesReceived = 0;

// Battery
int batteryVoltage = 0;
int batteryCurrent = 0;
int batteryFuel = 0;
int batteryPercent = 0;

// Attitude
int attPitch = 0;
int attRoll = 0;
int attYaw = 0;
char flightMode[16] = "---";

// CRSF frame types
#define CRSF_FRAMETYPE_BATTERY 0x08
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21

// CRSF parsing
uint8_t crsfBuffer[64];
uint8_t crsfIdx = 0;
uint8_t crsfLen = 0;
enum CrsfState { WAIT_SYNC, WAIT_LEN, READ_PAYLOAD };
CrsfState crsfState = WAIT_SYNC;

void parseCrsfTelemetry(uint8_t frameType, uint8_t* p, uint8_t len) {
    telemetryReceived = true;
    lastTelemetryTime = millis();

    switch (frameType) {
        case CRSF_FRAMETYPE_BATTERY:
            if (len >= 8) {
                batteryVoltage = (p[0] << 8) | p[1];
                batteryCurrent = (p[2] << 8) | p[3];
                batteryFuel = (p[4] << 16) | (p[5] << 8) | p[6];
                batteryPercent = p[7];
            }
            break;
        case CRSF_FRAMETYPE_ATTITUDE:
            if (len >= 6) {
                attPitch = (int16_t)((p[0] << 8) | p[1]);
                attRoll = (int16_t)((p[2] << 8) | p[3]);
                attYaw = (int16_t)((p[4] << 8) | p[5]);
            }
            break;
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            if (len > 0 && len < 15) {
                memcpy(flightMode, p, len);
                flightMode[len] = '\0';
            }
            break;
    }
}

void processCrsfByte(uint8_t b) {
    switch (crsfState) {
        case WAIT_SYNC:
            if (b == 0xC8 || b == 0xEA || b == 0xEC || b == 0xEE) {
                crsfState = WAIT_LEN;
            }
            break;
        case WAIT_LEN:
            if (b > 2 && b < 64) {
                crsfLen = b;
                crsfIdx = 0;
                crsfState = READ_PAYLOAD;
            } else {
                crsfState = WAIT_SYNC;
            }
            break;
        case READ_PAYLOAD:
            crsfBuffer[crsfIdx++] = b;
            if (crsfIdx >= crsfLen) {
                framesReceived++;
                parseCrsfTelemetry(crsfBuffer[0], &crsfBuffer[1], crsfLen - 2);
                crsfState = WAIT_SYNC;
            }
            break;
    }
}

void updateCrsfChannels() {
    crsfChannels.ch0 = map(channels[0], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch1 = map(channels[1], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch2 = map(channels[2], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch3 = map(channels[3], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch4 = map(channels[4], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch5 = map(channels[5], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch6 = map(channels[6], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch7 = map(channels[7], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch8 = map(channels[8], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch9 = map(channels[9], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch10 = map(channels[10], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch11 = map(channels[11], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch12 = map(channels[12], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch13 = map(channels[13], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch14 = map(channels[14], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
    crsfChannels.ch15 = map(channels[15], 1000, 2000, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
}

const char* HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>FC Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <style>
    * { box-sizing: border-box; touch-action: none; }
    body { font-family: monospace; background: #111; color: #0f0; margin: 0; padding: 10px; user-select: none; }
    h2 { font-size: 14px; margin: 10px 0 5px; color: #0a0; border-bottom: 1px solid #333; }
    .status { padding: 5px 10px; margin-bottom: 10px; font-size: 12px; display: inline-block; }
    .ok { background: #040; }
    .no-data { background: #400; color: #f00; }

    .sticks { display: flex; justify-content: space-around; margin: 10px 0; }
    .stick-container { text-align: center; }
    .stick-label { font-size: 11px; color: #888; margin-bottom: 5px; }
    .stick { width: 140px; height: 140px; background: #222; border-radius: 10px; position: relative; border: 2px solid #333; }
    .stick-knob { width: 50px; height: 50px; background: radial-gradient(circle, #4a4, #282); border-radius: 50%; position: absolute; transform: translate(-50%, -50%); border: 2px solid #6c6; }
    .stick-center { position: absolute; width: 10px; height: 10px; background: #444; border-radius: 50%; top: 50%; left: 50%; transform: translate(-50%, -50%); }

    .channels { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin: 10px 0; }
    .ch { text-align: center; padding: 5px; background: #222; border-radius: 5px; }
    .ch-name { font-size: 10px; color: #888; }
    .ch-val { font-size: 14px; color: #0f0; }

    .switches { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin: 10px 0; }
    .sw { display: flex; flex-direction: column; align-items: center; }
    .sw-label { font-size: 10px; color: #888; margin-bottom: 3px; }
    .sw-btn { width: 50px; height: 30px; border-radius: 15px; border: none; cursor: pointer; transition: all 0.2s; }
    .sw-off { background: #333; }
    .sw-on { background: #4a4; }
    .sw-mid { background: #664; }

    .tri-sw { display: flex; flex-direction: column; gap: 3px; }
    .tri-btn { width: 50px; height: 20px; border: none; cursor: pointer; font-size: 10px; color: #fff; }
    .tri-btn.active { background: #4a4; }
    .tri-btn:not(.active) { background: #333; }

    .telemetry { display: grid; grid-template-columns: repeat(2, 1fr); gap: 5px; font-size: 12px; }
    .telem-item { background: #1a1a1a; padding: 5px; border-radius: 3px; }
    .telem-label { color: #666; font-size: 10px; }
    .telem-val { color: #0f0; font-size: 14px; }

    .arm-btn { width: 100%; padding: 15px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; margin: 10px 0; }
    .arm-btn.armed { background: #a00; color: #fff; }
    .arm-btn.disarmed { background: #333; color: #888; }

    .flash-ctrl { background: #1a1a1a; padding: 10px; border-radius: 5px; }
    .flash-modes { display: flex; gap: 5px; }
    .flash-btn { flex: 1; padding: 12px; border: none; border-radius: 5px; cursor: pointer; font-size: 14px; font-weight: bold; }
    .flash-btn:not(.active) { background: #333; color: #888; }
    .flash-btn.active { background: #fc0; color: #000; }
  </style>
</head>
<body>
  <div style="display:flex;justify-content:space-between;align-items:center;">
    <h2 style="border:none;margin:0;">FC Controller</h2>
    <div id="status" class="status no-data">---</div>
  </div>

  <div class="sticks">
    <div class="stick-container">
      <div class="stick-label">THROTTLE / YAW</div>
      <div class="stick" id="stickL">
        <div class="stick-center"></div>
        <div class="stick-knob" id="knobL"></div>
      </div>
    </div>
    <div class="stick-container">
      <div class="stick-label">PITCH / ROLL</div>
      <div class="stick" id="stickR">
        <div class="stick-center"></div>
        <div class="stick-knob" id="knobR"></div>
      </div>
    </div>
  </div>

  <div class="channels" id="channelDisplay"></div>

  <h2>Switches</h2>
  <div class="switches" id="switches"></div>

  <button class="arm-btn disarmed" id="armBtn" onclick="toggleArm()">ARM (CH5)</button>

  <h2>Flashlight</h2>
  <div class="flash-ctrl">
    <div class="flash-modes">
      <button class="flash-btn active" onclick="setLight(0)" id="fl0">OFF</button>
      <button class="flash-btn" onclick="setLight(1)" id="fl1">LOW</button>
      <button class="flash-btn" onclick="setLight(2)" id="fl2">MED</button>
      <button class="flash-btn" onclick="setLight(3)" id="fl3">HIGH</button>
    </div>
  </div>

  <h2>Telemetry</h2>
  <div class="telemetry">
    <div class="telem-item"><div class="telem-label">Voltage</div><div class="telem-val" id="voltage">-</div></div>
    <div class="telem-item"><div class="telem-label">Current</div><div class="telem-val" id="current">-</div></div>
    <div class="telem-item"><div class="telem-label">Used</div><div class="telem-val" id="fuel">-</div></div>
    <div class="telem-item"><div class="telem-label">Battery</div><div class="telem-val" id="batPct">-</div></div>
    <div class="telem-item"><div class="telem-label">Pitch</div><div class="telem-val" id="pitch">-</div></div>
    <div class="telem-item"><div class="telem-label">Roll</div><div class="telem-val" id="roll">-</div></div>
    <div class="telem-item"><div class="telem-label">Yaw</div><div class="telem-val" id="yaw">-</div></div>
    <div class="telem-item"><div class="telem-label">Mode</div><div class="telem-val" id="mode">-</div></div>
  </div>

<script>
let ch = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000];
let armed = false;
let sendInterval;

// Initialize channel display
function initChannelDisplay() {
  let html = '';
  for (let i = 0; i < 8; i++) {
    html += `<div class="ch"><div class="ch-name">CH${i+1}</div><div class="ch-val" id="ch${i}">-</div></div>`;
  }
  document.getElementById('channelDisplay').innerHTML = html;
}

// Initialize switches (CH6-CH12)
function initSwitches() {
  let html = '';
  for (let i = 6; i <= 12; i++) {
    html += `
      <div class="sw">
        <div class="sw-label">CH${i}</div>
        <div class="tri-sw">
          <button class="tri-btn" onclick="setSw(${i-1}, 2000)" id="sw${i}h">H</button>
          <button class="tri-btn" onclick="setSw(${i-1}, 1500)" id="sw${i}m">M</button>
          <button class="tri-btn active" onclick="setSw(${i-1}, 1000)" id="sw${i}l">L</button>
        </div>
      </div>`;
  }
  document.getElementById('switches').innerHTML = html;
}

function setSw(idx, val) {
  ch[idx] = val;
  // Update button states
  const swNum = idx + 1;
  document.getElementById(`sw${swNum}h`).className = 'tri-btn' + (val === 2000 ? ' active' : '');
  document.getElementById(`sw${swNum}m`).className = 'tri-btn' + (val === 1500 ? ' active' : '');
  document.getElementById(`sw${swNum}l`).className = 'tri-btn' + (val === 1000 ? ' active' : '');
}

function toggleArm() {
  armed = !armed;
  ch[4] = armed ? 2000 : 1000;
  const btn = document.getElementById('armBtn');
  btn.className = 'arm-btn ' + (armed ? 'armed' : 'disarmed');
  btn.textContent = armed ? 'DISARM (CH5)' : 'ARM (CH5)';
}

function setLight(level) {
  const brightness = [0, 60, 150, 255][level];
  const mode = level === 0 ? 0 : 3; // 0=off, 3=steady
  fetch('/flash?m=' + mode + '&b=' + brightness).catch(() => {});
  for (let i = 0; i < 4; i++) {
    document.getElementById('fl' + i).className = 'flash-btn' + (i === level ? ' active' : '');
  }
}

// Multitouch joystick handling
const stickData = {};
const touchToStick = {};

function setupStick(stickId, knobId, isLeft) {
  const stick = document.getElementById(stickId);
  const knob = document.getElementById(knobId);
  stickData[stickId] = { stick, knob, isLeft };

  stick.addEventListener('touchstart', (e) => {
    e.preventDefault();
    const touch = e.changedTouches[0];
    touchToStick[touch.identifier] = stickId;
    updateStick(stickId, touch.clientX, touch.clientY);
  });

  stick.addEventListener('mousedown', (e) => {
    stickData[stickId].mouseDown = true;
    updateStick(stickId, e.clientX, e.clientY);
  });

  // Initial position
  setTimeout(() => {
    const rect = stick.getBoundingClientRect();
    knob.style.left = rect.width / 2 + 'px';
    knob.style.top = isLeft ? rect.height - 25 + 'px' : rect.height / 2 + 'px';
  }, 100);
}

function updateStick(stickId, clientX, clientY) {
  const { stick, knob, isLeft } = stickData[stickId];
  const rect = stick.getBoundingClientRect();
  const centerX = rect.width / 2;
  const centerY = rect.height / 2;
  const maxD = centerX - 25;

  let dx = clientX - rect.left - centerX;
  let dy = clientY - rect.top - centerY;

  // Square boundary - clamp X and Y independently
  dx = Math.max(-maxD, Math.min(maxD, dx));
  dy = Math.max(-maxD, Math.min(maxD, dy));

  knob.style.left = (centerX + dx) + 'px';
  knob.style.top = (centerY + dy) + 'px';

  const valX = Math.round(1500 + dx / maxD * 500);
  const valY = Math.round(1500 - dy / maxD * 500);

  if (isLeft) {
    ch[3] = valX;  // Yaw
    ch[2] = valY;  // Throttle
  } else {
    ch[0] = valX;  // Roll
    ch[1] = valY;  // Pitch
  }
}

function resetStick(stickId) {
  const { stick, knob, isLeft } = stickData[stickId];
  const rect = stick.getBoundingClientRect();
  const centerX = rect.width / 2;
  const centerY = rect.height / 2;

  if (isLeft) {
    knob.style.left = centerX + 'px';
    ch[3] = 1500;
  } else {
    knob.style.left = centerX + 'px';
    knob.style.top = centerY + 'px';
    ch[0] = 1500;
    ch[1] = 1500;
  }
}

// Global touch handlers
document.addEventListener('touchmove', (e) => {
  e.preventDefault();
  for (let t of e.changedTouches) {
    const stickId = touchToStick[t.identifier];
    if (stickId) {
      updateStick(stickId, t.clientX, t.clientY);
    }
  }
}, { passive: false });

document.addEventListener('touchend', (e) => {
  for (let t of e.changedTouches) {
    const stickId = touchToStick[t.identifier];
    if (stickId) {
      delete touchToStick[t.identifier];
      resetStick(stickId);
    }
  }
});

document.addEventListener('touchcancel', (e) => {
  for (let t of e.changedTouches) {
    const stickId = touchToStick[t.identifier];
    if (stickId) {
      delete touchToStick[t.identifier];
      resetStick(stickId);
    }
  }
});

// Global mouse handlers
document.addEventListener('mousemove', (e) => {
  for (let id in stickData) {
    if (stickData[id].mouseDown) {
      updateStick(id, e.clientX, e.clientY);
    }
  }
});

document.addEventListener('mouseup', () => {
  for (let id in stickData) {
    if (stickData[id].mouseDown) {
      stickData[id].mouseDown = false;
      resetStick(id);
    }
  }
});

// Send channels to NodeMCU
function sendChannels() {
  fetch('/ch?d=' + ch.join(',')).catch(() => {});
}

// Get telemetry
function getTelemetry() {
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('status').className = 'status ' + (d.ok ? 'ok' : 'no-data');
    document.getElementById('status').textContent = d.ok ? 'LINK OK' : 'NO LINK';

    document.getElementById('voltage').textContent = (d.batV / 10).toFixed(1) + 'V';
    document.getElementById('current').textContent = (d.batA / 10).toFixed(1) + 'A';
    document.getElementById('fuel').textContent = d.batMah + 'mAh';
    document.getElementById('batPct').textContent = d.batPct + '%';
    document.getElementById('pitch').textContent = (d.pitch / 10000 * 180 / Math.PI).toFixed(0) + '°';
    document.getElementById('roll').textContent = (d.roll / 10000 * 180 / Math.PI).toFixed(0) + '°';
    document.getElementById('yaw').textContent = (d.yaw / 10000 * 180 / Math.PI).toFixed(0) + '°';
    document.getElementById('mode').textContent = d.mode;
  }).catch(() => {});
}

// Update channel display
function updateDisplay() {
  for (let i = 0; i < 8; i++) {
    document.getElementById('ch' + i).textContent = ch[i];
  }
}

// Init
initChannelDisplay();
initSwitches();
setupStick('stickL', 'knobL', true);
setupStick('stickR', 'knobR', false);

setInterval(sendChannels, 50);
setInterval(getTelemetry, 200);
setInterval(updateDisplay, 100);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send(200, "text/html", HTML_PAGE);
}

void handleChannels() {
    if (server.hasArg("d")) {
        String data = server.arg("d");
        int idx = 0;
        int start = 0;
        for (int i = 0; i <= data.length() && idx < 16; i++) {
            if (i == data.length() || data[i] == ',') {
                channels[idx++] = data.substring(start, i).toInt();
                start = i + 1;
            }
        }
    }
    server.send(200, "text/plain", "OK");
}

void handleData() {
    String json = "{";
    json += "\"ok\":" + String(telemetryReceived ? "true" : "false");
    json += ",\"batV\":" + String(batteryVoltage);
    json += ",\"batA\":" + String(batteryCurrent);
    json += ",\"batMah\":" + String(batteryFuel);
    json += ",\"batPct\":" + String(batteryPercent);
    json += ",\"pitch\":" + String(attPitch);
    json += ",\"roll\":" + String(attRoll);
    json += ",\"yaw\":" + String(attYaw);
    json += ",\"mode\":\"" + String(flightMode) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleFlash() {
    if (server.hasArg("m")) {
        int mode = server.arg("m").toInt();
        flashLight.setMode((FlashLightMode)mode);
    }
    if (server.hasArg("b")) {
        int brightness = server.arg("b").toInt();
        flashLight.setBrightness(brightness);
    }
    server.send(200, "text/plain", "OK");
}

void setup() {
    Serial.begin(420000);
    crsf.begin(Serial);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    server.on("/", handleRoot);
    server.on("/ch", handleChannels);
    server.on("/data", handleData);
    server.on("/flash", handleFlash);
    server.begin();

    flashLight.begin();
    flashLight.setMode(FLASH_MODE_OFF);

    updateCrsfChannels();
}

void loop() {
    // Read telemetry
    while (Serial.available()) {
        processCrsfByte(Serial.read());
    }

    // Send CRSF channels every 20ms
    if (millis() - lastCrsfTime >= 20) {
        lastCrsfTime = millis();
        updateCrsfChannels();
        crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
    }

    // Timeout
    if (millis() - lastTelemetryTime > 2000) {
        telemetryReceived = false;
    }

    flashLight.update();

    server.handleClient();
    yield();
}
