/*
 * ESP32 + INA219 power monitor for WETI ultrasonic distance meter.
 * Hosts a web dashboard showing live current/voltage/power.
 *
 * WiFi: "Grota Kindziuk"
 * INA219: I2C (SDA=GPIO21, SCL=GPIO22), default address 0x40
 * Shunt: 10 ohm THT replaces stock 0.1 ohm SMD on HW-831B
 *
 * After upload, open Serial Monitor at 115200 to see assigned IP.
 * Open that IP in browser (or http://weti-meter.local with mDNS).
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

const char* WIFI_SSID = "Grota Kindziuka";
const char* WIFI_PASS = "wdupiezgaslo";
const char* MDNS_NAME = "weti-meter";

const float SHUNT_OHMS  = 1.5;      // actual measured shunt value
const float SHUNT_SCALE = SHUNT_OHMS / 0.1;   // = 15 (library assumes 0.1)

Adafruit_INA219 ina(0x45);          // address from i2c_scan
WebServer       server(80);

// Statistics
struct Stats {
  float   current_uA = 0;
  float   voltage_V  = 0;
  float   power_uW   = 0;
  float   min_uA     = 999999;
  float   max_uA     = 0;
  double  sum_uA     = 0;
  uint32_t samples   = 0;
  uint32_t boot_ms   = 0;
};
Stats st;

unsigned long lastSample = 0;
const unsigned long SAMPLE_MS = 50;   // 20 Hz sampling

// ============================== WEB PAGE ===============================
const char* PAGE_HTML PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="pl"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WETI Power Monitor</title>
<style>
  :root { color-scheme: dark; }
  body { background:#1a1a2e; color:#eee; font:16px monospace;
         margin:0; padding:20px; }
  h1 { color:#0af; margin:0 0 20px 0; font-size:22px; }
  .card { background:#222244; border-radius:12px; padding:20px;
          margin-bottom:14px; box-shadow:0 4px 12px #0008; }
  .label { color:#8af; font-size:12px; text-transform:uppercase;
           letter-spacing:1.5px; margin-bottom:6px; }
  .value { font-size:36px; font-weight:bold; color:#fff; }
  .unit  { font-size:18px; color:#ccc; margin-left:8px; }
  .row { display:flex; gap:14px; flex-wrap:wrap; }
  .row .card { flex:1; min-width:140px; }
  button { background:#0af; color:#000; border:none; padding:10px 20px;
           border-radius:8px; font-weight:bold; cursor:pointer; }
  button:hover { background:#0cf; }
  .small { font-size:14px; color:#999; }
</style>
</head><body>
<h1>WETI Ultrasonic Meter — Power Monitor</h1>

<div class="card">
  <div class="label">Prąd</div>
  <div><span class="value" id="cur">--</span><span class="unit">µA</span></div>
</div>

<div class="row">
  <div class="card">
    <div class="label">Napięcie</div>
    <div><span class="value" id="vol">--</span><span class="unit">V</span></div>
  </div>
  <div class="card">
    <div class="label">Moc</div>
    <div><span class="value" id="pow">--</span><span class="unit">µW</span></div>
  </div>
</div>

<div class="row">
  <div class="card">
    <div class="label">Min</div>
    <div><span class="value" id="min">--</span><span class="unit">µA</span></div>
  </div>
  <div class="card">
    <div class="label">Średnio</div>
    <div><span class="value" id="avg">--</span><span class="unit">µA</span></div>
  </div>
  <div class="card">
    <div class="label">Max</div>
    <div><span class="value" id="max">--</span><span class="unit">µA</span></div>
  </div>
</div>

<div class="card">
  <div class="label">Statystyki</div>
  <div class="small">Próbek: <span id="n">0</span> &nbsp; Czas: <span id="t">0</span> s</div>
  <br>
  <button onclick="reset()">RESET statystyk</button>
</div>

<script>
async function update() {
  try {
    const r = await fetch('/data');
    const d = await r.json();
    document.getElementById('cur').textContent = d.cur.toFixed(1);
    document.getElementById('vol').textContent = d.vol.toFixed(3);
    document.getElementById('pow').textContent = d.pow.toFixed(1);
    document.getElementById('min').textContent = d.min.toFixed(1);
    document.getElementById('avg').textContent = d.avg.toFixed(1);
    document.getElementById('max').textContent = d.max.toFixed(1);
    document.getElementById('n').textContent   = d.n;
    document.getElementById('t').textContent   = (d.ms/1000).toFixed(0);
  } catch(e) {}
}
async function reset() { await fetch('/reset'); update(); }
setInterval(update, 250);
update();
</script>
</body></html>
)HTML";

// ============================== HANDLERS ===============================
void handleRoot()  { server.send(200, "text/html", PAGE_HTML); }

void handleData() {
  float avg = st.samples ? (float)(st.sum_uA / st.samples) : 0;
  String j = "{";
  j += "\"cur\":" + String(st.current_uA, 2);
  j += ",\"vol\":" + String(st.voltage_V, 4);
  j += ",\"pow\":" + String(st.power_uW, 2);
  j += ",\"min\":" + String(st.min_uA == 999999 ? 0 : st.min_uA, 2);
  j += ",\"max\":" + String(st.max_uA, 2);
  j += ",\"avg\":" + String(avg, 2);
  j += ",\"n\":" + String(st.samples);
  j += ",\"ms\":" + String(millis() - st.boot_ms);
  j += "}";
  server.send(200, "application/json", j);
}

void handleReset() {
  st.min_uA  = 999999;
  st.max_uA  = 0;
  st.sum_uA  = 0;
  st.samples = 0;
  st.boot_ms = millis();
  server.send(200, "text/plain", "ok");
}

// =============================== SAMPLE ================================
void sampleINA() {
  float mA = ina.getCurrent_mA() / SHUNT_SCALE;   // library assumes 0.1 ohm
  float V  = ina.getBusVoltage_V();
  float uA = mA * 1000.0;

  st.current_uA = uA;
  st.voltage_V  = V;
  st.power_uW   = uA * V;

  if (uA < st.min_uA) st.min_uA = uA;
  if (uA > st.max_uA) st.max_uA = uA;
  st.sum_uA += uA;
  st.samples++;
}

// =============================== SETUP =================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== WETI Power Monitor ===");

  // I2C + INA219
  Wire.begin(21, 22);                  // ESP32 default I2C pins
  if (!ina.begin()) {
    Serial.println("INA219 not found! Check wiring.");
    while (1) delay(1000);
  }
  ina.setCalibration_16V_400mA();
  Serial.println("INA219 OK");

  // WiFi
  Serial.printf("Connecting to '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi FAILED. Restarting...");
    delay(2000); ESP.restart();
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());

  // mDNS
  if (MDNS.begin(MDNS_NAME)) {
    Serial.printf("mDNS: http://%s.local\n", MDNS_NAME);
  }

  // HTTP routes
  server.on("/",      handleRoot);
  server.on("/data",  handleData);
  server.on("/reset", handleReset);
  server.begin();
  Serial.println("HTTP server started");

  st.boot_ms = millis();
}

// =============================== LOOP ==================================
void loop() {
  server.handleClient();

  if (millis() - lastSample >= SAMPLE_MS) {
    lastSample = millis();
    sampleINA();
  }
}
