#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ElegantOTA.h>

// ── Pins ────────────────────────────────────────────────────────────────────
#define RELAY_PIN   5   // HIGH = relay ON
#define LD2410_RX   3
#define LD2410_TX   4
#define LD2410_BAUD 256000

// ── AP fallback ──────────────────────────────────────────────────────────────
#define AP_SSID "ESP32-LD2410C"
#define AP_PASS "12345678"

// ── Objects ──────────────────────────────────────────────────────────────────
HardwareSerial radarSerial(1);
Preferences    prefs;
AsyncWebServer server(80);
DNSServer      dns;
WiFiClient     wifiClient;
PubSubClient   mqtt(wifiClient);

// ── Config (loaded from NVS) ─────────────────────────────────────────────────
char cfgSsid[64]      = "";
char cfgPass[64]      = "";
char cfgMqttHost[64]  = "";
int  cfgMqttPort      = 1883;
char cfgMqttUser[64]  = "";
char cfgMqttPass[64]  = "";
char cfgMqttTopic[64] = "ld2410c";
int  cfgCooldown      = 10;     // seconds

// Radar gate config (stored in NVS, applied to sensor on demand)
uint8_t  rcMaxMove  = 8;
uint8_t  rcMaxStill = 8;
uint16_t rcIdleSec  = 5;
uint8_t  rcMoveSens[9]  = {50,50,40,40,40,40,30,30,30};
uint8_t  rcStillSens[9] = { 0, 0,40,40,40,40,30,30,30};

// ── Runtime state ────────────────────────────────────────────────────────────
bool     presence      = false;
bool     relayOn       = false;
uint16_t moveDist      = 0;
uint8_t  moveEnergy    = 0;
uint16_t stillDist     = 0;
uint8_t  stillEnergy   = 0;
uint16_t detectDist    = 0;
uint8_t  targetState   = 0;   // 0=none, 1=moving, 2=still, 3=both
unsigned long lastSeenMs   = 0;

bool apMode            = false;
bool prevPresence      = false;
volatile bool applyRadarCfg = false;

unsigned long lastMqttPublishMs = 0;
unsigned long lastMqttReconnMs  = 0;

// ── LD2410C frame parser ─────────────────────────────────────────────────────
// Basic data frame: F4 F3 F2 F1 [len 2B LE] 02 AA [state] [moveDist 2B] [moveEn]
//                  [stillDist 2B] [stillEn] [detectDist 2B] 55 00 F8 F7 F6 F5

static uint8_t rxBuf[100];
static int     rxLen = 0;

static void parseFrame(const uint8_t* d, int len) {
    if (len < 13 || d[0] != 0x02 || d[1] != 0xAA) return;
    targetState = d[2];
    moveDist    = d[3]  | (d[4]  << 8);
    moveEnergy  = d[5];
    stillDist   = d[6]  | (d[7]  << 8);
    stillEnergy = d[8];
    detectDist  = d[9]  | (d[10] << 8);
    if (targetState == 0) { moveDist = 0; moveEnergy = 0; stillDist = 0; stillEnergy = 0; detectDist = 0; }
}

static void readRadar() {
    while (radarSerial.available()) {
        uint8_t b = radarSerial.read();
        if (rxLen < (int)sizeof(rxBuf)) rxBuf[rxLen++] = b;

        if (rxLen >= 4 &&
            rxBuf[rxLen-4] == 0xF8 && rxBuf[rxLen-3] == 0xF7 &&
            rxBuf[rxLen-2] == 0xF6 && rxBuf[rxLen-1] == 0xF5) {
            for (int i = 0; i + 8 < rxLen; i++) {
                if (rxBuf[i]==0xF4 && rxBuf[i+1]==0xF3 &&
                    rxBuf[i+2]==0xF2 && rxBuf[i+3]==0xF1) {
                    int dlen = rxBuf[i+4] | (rxBuf[i+5] << 8);
                    parseFrame(&rxBuf[i+6], dlen);
                    break;
                }
            }
            rxLen = 0;
        }
    }
}

// ── LD2410C config commands ──────────────────────────────────────────────────

static void radarSendCmd(const uint8_t* cmd, size_t len) {
    static const uint8_t H[] = {0xFD,0xFC,0xFB,0xFA};
    static const uint8_t F[] = {0x04,0x03,0x02,0x01};
    radarSerial.write(H, 4);
    radarSerial.write((uint8_t)(len & 0xFF));
    radarSerial.write((uint8_t)(len >> 8));
    radarSerial.write(cmd, len);
    radarSerial.write(F, 4);
    radarSerial.flush();
}

static void radarEnableConfig() {
    uint8_t cmd[] = {0xFF,0x00,0x01,0x00};
    radarSendCmd(cmd, sizeof(cmd));
    delay(100);
}

static void radarDisableConfig() {
    uint8_t cmd[] = {0xFE,0x00};
    radarSendCmd(cmd, sizeof(cmd));
    delay(100);
}

static void radarSetMaxGates() {
    uint8_t cmd[20] = {
        0x60,0x00,
        0x00,0x00, rcMaxMove,  0x00,0x00,0x00,
        0x01,0x00, rcMaxStill, 0x00,0x00,0x00,
        0x02,0x00, (uint8_t)(rcIdleSec & 0xFF), (uint8_t)(rcIdleSec >> 8), 0x00,0x00
    };
    radarSendCmd(cmd, sizeof(cmd));
    delay(100);
}

static void radarSetGateSens(uint8_t gate) {
    uint8_t cmd[20] = {
        0x64,0x00,
        0x00,0x00, gate,               0x00,0x00,0x00,
        0x01,0x00, rcMoveSens[gate],   0x00,0x00,0x00,
        0x02,0x00, rcStillSens[gate],  0x00,0x00,0x00
    };
    radarSendCmd(cmd, sizeof(cmd));
    delay(100);
}

static void doApplyRadarConfig() {
    radarEnableConfig();
    radarSetMaxGates();
    for (int i = 0; i <= 8; i++) radarSetGateSens(i);
    radarDisableConfig();
}

// ── NVS helpers ──────────────────────────────────────────────────────────────

void loadConfig() {
    prefs.begin("cfg", true);
    prefs.getString("ssid",      cfgSsid,      sizeof(cfgSsid));
    prefs.getString("pass",      cfgPass,      sizeof(cfgPass));
    prefs.getString("mqttHost",  cfgMqttHost,  sizeof(cfgMqttHost));
    cfgMqttPort = prefs.getInt("mqttPort", 1883);
    prefs.getString("mqttUser",  cfgMqttUser,  sizeof(cfgMqttUser));
    prefs.getString("mqttPass",  cfgMqttPass,  sizeof(cfgMqttPass));
    prefs.getString("mqttTopic", cfgMqttTopic, sizeof(cfgMqttTopic));
    cfgCooldown = prefs.getInt("cooldown", 10);
    rcMaxMove   = prefs.getUChar("rcMaxMove",  8);
    rcMaxStill  = prefs.getUChar("rcMaxStill", 8);
    rcIdleSec   = prefs.getUShort("rcIdle",   5);
    for (int i = 0; i <= 8; i++) {
        rcMoveSens[i]  = prefs.getUChar(("rcM" + String(i)).c_str(), rcMoveSens[i]);
        rcStillSens[i] = prefs.getUChar(("rcS" + String(i)).c_str(), rcStillSens[i]);
    }
    prefs.end();
}

void saveWifi() {
    prefs.begin("cfg", false);
    prefs.putString("ssid", cfgSsid);
    prefs.putString("pass", cfgPass);
    prefs.end();
}

void saveMqtt() {
    prefs.begin("cfg", false);
    prefs.putString("mqttHost",  cfgMqttHost);
    prefs.putInt("mqttPort",     cfgMqttPort);
    prefs.putString("mqttUser",  cfgMqttUser);
    prefs.putString("mqttPass",  cfgMqttPass);
    prefs.putString("mqttTopic", cfgMqttTopic);
    prefs.putInt("cooldown",     cfgCooldown);
    prefs.end();
}

void saveRadarCfg() {
    prefs.begin("cfg", false);
    prefs.putUChar("rcMaxMove",  rcMaxMove);
    prefs.putUChar("rcMaxStill", rcMaxStill);
    prefs.putUShort("rcIdle",    rcIdleSec);
    for (int i = 0; i <= 8; i++) {
        prefs.putUChar(("rcM" + String(i)).c_str(), rcMoveSens[i]);
        prefs.putUChar(("rcS" + String(i)).c_str(), rcStillSens[i]);
    }
    prefs.end();
}

// ── Relay ────────────────────────────────────────────────────────────────────

void setRelay(bool on) {
    if (relayOn == on) return;
    relayOn = on;
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

// ── MQTT ─────────────────────────────────────────────────────────────────────

void mqttPublish() {
    if (!mqtt.connected()) return;
    String base = String(cfgMqttTopic);
    mqtt.publish((base + "/presence").c_str(), presence ? "ON" : "OFF", true);
    mqtt.publish((base + "/relay").c_str(),    relayOn  ? "ON" : "OFF", true);

    JsonDocument doc;
    doc["presence"]      = presence;
    doc["relay"]         = relayOn;
    doc["moving_dist"]   = moveDist;
    doc["still_dist"]    = stillDist;
    doc["detect_dist"]   = detectDist;
    doc["moving_energy"] = moveEnergy;
    doc["still_energy"]  = stillEnergy;
    char buf[256];
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish((base + "/state").c_str(), buf, true);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String t = String(cfgMqttTopic) + "/relay/set";
    if (String(topic) != t) return;
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
    msg.toUpperCase();
    msg.trim();
    if (msg == "ON"  || msg == "1" || msg == "TRUE")  setRelay(true);
    if (msg == "OFF" || msg == "0" || msg == "FALSE") setRelay(false);
}

void mqttConnect() {
    if (strlen(cfgMqttHost) == 0) return;
    mqtt.setServer(cfgMqttHost, cfgMqttPort);
    mqtt.setCallback(mqttCallback);
    String cid = "ld2410c-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = (strlen(cfgMqttUser) > 0)
        ? mqtt.connect(cid.c_str(), cfgMqttUser, cfgMqttPass)
        : mqtt.connect(cid.c_str());
    if (ok) {
        mqtt.subscribe((String(cfgMqttTopic) + "/relay/set").c_str());
        mqttPublish();
    }
}

// ── HTML helpers ─────────────────────────────────────────────────────────────

static const char CSS[] PROGMEM = R"(
body{font-family:Arial,sans-serif;margin:0;background:#1e1e2e;color:#cdd6f4}
nav{background:#313244;padding:10px 20px;display:flex;gap:20px;flex-wrap:wrap}
nav a{color:#89b4fa;text-decoration:none}
nav a:hover{color:#cba6f7}
.c{max-width:680px;margin:28px auto;padding:0 14px}
h2{color:#89b4fa;border-bottom:1px solid #45475a;padding-bottom:7px}
.box{background:#313244;border-radius:8px;padding:18px;margin-bottom:18px}
label{display:block;margin:9px 0 3px;color:#a6adc8;font-size:14px}
input[type=text],input[type=password],input[type=number]{
  width:100%;padding:7px;background:#45475a;border:1px solid #585b70;
  border-radius:4px;color:#cdd6f4;box-sizing:border-box}
td input[type=number]{width:65px;padding:4px}
button{background:#89b4fa;color:#1e1e2e;border:none;padding:9px 22px;
  border-radius:4px;cursor:pointer;font-size:14px;margin-top:12px;font-weight:bold}
button:hover{background:#74c7ec}
.row{display:flex;justify-content:space-between;align-items:center;
  padding:6px 0;border-bottom:1px solid #45475a}
.row:last-child{border-bottom:none}
.on{background:#a6e3a1;color:#1e1e2e;padding:2px 11px;border-radius:10px;font-weight:bold}
.off{background:#45475a;color:#cdd6f4;padding:2px 11px;border-radius:10px}
table{width:100%;border-collapse:collapse;font-size:13px}
th{text-align:left;color:#a6adc8;padding:4px 6px;border-bottom:1px solid #45475a}
td{padding:3px 6px})";

static String nav() {
    return R"(<nav><a href='/'>Status</a><a href='/wifi'>Wi-Fi</a><a href='/mqtt'>MQTT</a><a href='/ld2410'>Sensor</a><a href='/update'>OTA Update</a></nav><div class='c'>)";
}
static String head(const char* extra = "") {
    return String("<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>LD2410C</title><style>") + FPSTR(CSS) + "</style>" + extra +
        "</head><body>" + nav();
}
static const char FOOT[] = "</div></body></html>";

// ── Page builders ─────────────────────────────────────────────────────────────

String pageStatus() {
    String h = head("<meta http-equiv='refresh' content='3'>");
    h += "<h2>Status</h2><div class='box'>";
    h += "<div class='row'><span>Presence</span><span class='" + String(presence ? "on'>DETECTED" : "off'>NONE") + "</span></div>";
    h += "<div class='row'><span>Relay</span><span class='" + String(relayOn ? "on'>ON" : "off'>OFF") + "</span></div>";
    h += "<div class='row'><span>Moving target</span><span>" + String(moveDist) + " cm &nbsp;energy " + String(moveEnergy) + "</span></div>";
    h += "<div class='row'><span>Still target</span><span>"  + String(stillDist) + " cm &nbsp;energy " + String(stillEnergy) + "</span></div>";
    h += "<div class='row'><span>Detection distance</span><span>" + String(detectDist) + " cm</span></div>";
    h += "<div class='row'><span>Cooldown</span><span>" + String(cfgCooldown) + " s</span></div>";
    h += "</div>";
    if (!apMode) {
        h += "<div class='box'>";
        h += "<div class='row'><span>Wi-Fi SSID</span><span>" + String(cfgSsid) + "</span></div>";
        h += "<div class='row'><span>IP address</span><span>" + WiFi.localIP().toString() + "</span></div>";
        h += "<div class='row'><span>MQTT broker</span><span>" + String(cfgMqttHost) + ":" + String(cfgMqttPort) + "</span></div>";
        h += "<div class='row'><span>MQTT status</span><span class='" + String(mqtt.connected() ? "on'>Connected" : "off'>Disconnected") + "</span></div>";
        h += "</div>";
    }
    h += FOOT;
    return h;
}

String pageWifi() {
    String h = head();
    h += "<h2>Wi-Fi Settings</h2><div class='box'>"
         "<form method='POST' action='/save/wifi'>"
         "<label>SSID</label><input type='text' name='ssid' value='" + String(cfgSsid) + "' required>"
         "<label>Password</label><input type='password' name='pass' placeholder='leave blank to keep current'>"
         "<button type='submit'>Save &amp; Reboot</button>"
         "</form></div>";
    h += FOOT;
    return h;
}

String pageMqtt() {
    String h = head();
    h += "<h2>MQTT Settings</h2><div class='box'><form method='POST' action='/save/mqtt'>"
         "<label>Broker host</label><input type='text' name='host' value='" + String(cfgMqttHost) + "'>"
         "<label>Port</label><input type='number' name='port' value='" + String(cfgMqttPort) + "' min='1' max='65535'>"
         "<label>Username (optional)</label><input type='text' name='user' value='" + String(cfgMqttUser) + "'>"
         "<label>Password (optional)</label><input type='password' name='mqttpass' placeholder='leave blank to keep current'>"
         "<label>Base topic</label><input type='text' name='topic' value='" + String(cfgMqttTopic) + "'>"
         "<label>Presence cooldown (seconds)</label><input type='number' name='cooldown' value='" + String(cfgCooldown) + "' min='1' max='3600'>"
         "<button type='submit'>Save &amp; Reboot</button>"
         "</form></div>";
    h += FOOT;
    return h;
}

String pageSensor() {
    String h = head();
    h += "<h2>Sensor Settings</h2><div class='box'><form method='POST' action='/save/ld2410'>"
         "<label>Max moving gate (0–8, each gate = 0.75 m)</label>"
         "<input type='number' name='maxmove' value='" + String(rcMaxMove) + "' min='2' max='8'>"
         "<label>Max stationary gate (0–8)</label>"
         "<input type='number' name='maxstill' value='" + String(rcMaxStill) + "' min='2' max='8'>"
         "<label>No-one idle time (seconds)</label>"
         "<input type='number' name='idle' value='" + String(rcIdleSec) + "' min='0' max='65535'>"
         "<h3 style='color:#89b4fa;margin:18px 0 8px'>Gate Sensitivities</h3>"
         "<table><tr><th>Gate</th><th>Distance</th><th>Moving (0–100)</th><th>Stationary (0–100)</th></tr>";
    for (int i = 0; i <= 8; i++) {
        h += "<tr><td>" + String(i) + "</td><td>" + String(i * 75) + " cm</td>"
             "<td><input type='number' name='ms" + String(i) + "' value='" + String(rcMoveSens[i])  + "' min='0' max='100'></td>"
             "<td><input type='number' name='ss" + String(i) + "' value='" + String(rcStillSens[i]) + "' min='0' max='100'></td></tr>";
    }
    h += "</table><button type='submit'>Apply to sensor</button></form></div>";
    h += FOOT;
    return h;
}

// ── Web server ────────────────────────────────────────────────────────────────

void setupServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", pageStatus());
    });
    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", pageWifi());
    });
    server.on("/mqtt", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", pageMqtt());
    });
    server.on("/ld2410", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", pageSensor());
    });

    server.on("/save/wifi", HTTP_POST, [](AsyncWebServerRequest* r) {
        if (r->hasParam("ssid", true))
            strlcpy(cfgSsid, r->getParam("ssid", true)->value().c_str(), sizeof(cfgSsid));
        auto p = r->getParam("pass", true);
        if (p && p->value().length() > 0)
            strlcpy(cfgPass, p->value().c_str(), sizeof(cfgPass));
        saveWifi();
        r->send(200, "text/html", "<p>Saved. Rebooting&hellip;</p>");
        delay(800);
        ESP.restart();
    });

    server.on("/save/mqtt", HTTP_POST, [](AsyncWebServerRequest* r) {
        if (r->hasParam("host",     true)) strlcpy(cfgMqttHost,  r->getParam("host",     true)->value().c_str(), sizeof(cfgMqttHost));
        if (r->hasParam("port",     true)) cfgMqttPort = r->getParam("port",     true)->value().toInt();
        if (r->hasParam("user",     true)) strlcpy(cfgMqttUser,  r->getParam("user",     true)->value().c_str(), sizeof(cfgMqttUser));
        if (r->hasParam("topic",    true)) strlcpy(cfgMqttTopic, r->getParam("topic",    true)->value().c_str(), sizeof(cfgMqttTopic));
        if (r->hasParam("cooldown", true)) cfgCooldown = r->getParam("cooldown", true)->value().toInt();
        auto p = r->getParam("mqttpass", true);
        if (p && p->value().length() > 0)
            strlcpy(cfgMqttPass, p->value().c_str(), sizeof(cfgMqttPass));
        saveMqtt();
        r->send(200, "text/html", "<p>Saved. Rebooting&hellip;</p>");
        delay(800);
        ESP.restart();
    });

    server.on("/save/ld2410", HTTP_POST, [](AsyncWebServerRequest* r) {
        if (r->hasParam("maxmove",  true)) rcMaxMove  = r->getParam("maxmove",  true)->value().toInt();
        if (r->hasParam("maxstill", true)) rcMaxStill = r->getParam("maxstill", true)->value().toInt();
        if (r->hasParam("idle",     true)) rcIdleSec  = r->getParam("idle",     true)->value().toInt();
        for (int i = 0; i <= 8; i++) {
            auto ms = r->getParam("ms" + String(i), true);
            auto ss = r->getParam("ss" + String(i), true);
            if (ms) rcMoveSens[i]  = ms->value().toInt();
            if (ss) rcStillSens[i] = ss->value().toInt();
        }
        saveRadarCfg();
        applyRadarCfg = true;   // applied in loop() to avoid ISR/serial conflict
        r->send(200, "text/html", "<meta http-equiv='refresh' content='2;url=/ld2410'><p>Applying settings&hellip;</p>");
    });

    // Captive portal catch-all
    server.onNotFound([](AsyncWebServerRequest* r) {
        if (apMode) r->redirect("/wifi");
        else        r->send(404, "text/plain", "Not found");
    });

    server.begin();
}

// ── Wi-Fi ─────────────────────────────────────────────────────────────────────

bool connectWifi() {
    if (strlen(cfgSsid) == 0) return false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSsid, cfgPass);
    Serial.print("WiFi connecting");
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("IP: " + WiFi.localIP().toString());
        return true;
    }
    return false;
}

void startAP() {
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    dns.start(53, "*", WiFi.softAPIP());
    Serial.println("AP mode " + WiFi.softAPIP().toString() + "  SSID: " AP_SSID "  Pass: " AP_PASS);
}

// ── OTA ───────────────────────────────────────────────────────────────────────

void setupOTA() {
    // ArduinoOTA — PlatformIO network upload (pio run -t upload --upload-port <IP>)
    ArduinoOTA.setHostname("esp32-ld2410c");
    ArduinoOTA.onStart([]() {
        Serial.println("OTA start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA done");
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("OTA error %u\n", e);
    });
    ArduinoOTA.begin();

    // ElegantOTA — browser upload at http://<IP>/update
    ElegantOTA.begin(&server);
    Serial.println("OTA ready  (ArduinoOTA + /update)");
}

// ── setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    loadConfig();

    radarSerial.begin(LD2410_BAUD, SERIAL_8N1, LD2410_RX, LD2410_TX);
    delay(200);   // let LD2410C boot

    if (!connectWifi()) startAP();

    setupServer();

    if (!apMode) setupOTA();

    if (!apMode && strlen(cfgMqttHost) > 0) mqttConnect();

    Serial.println("Ready");
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop() {
    if (apMode) {
        dns.processNextRequest();
        return;
    }

    // Wi-Fi watchdog
    if (WiFi.status() != WL_CONNECTED) {
        connectWifi();
        return;
    }

    // OTA
    ArduinoOTA.handle();
    ElegantOTA.loop();

    // LD2410C
    readRadar();

    // Apply pending sensor config (set by web handler)
    if (applyRadarCfg) {
        applyRadarCfg = false;
        doApplyRadarConfig();
    }

    // Presence detection & relay
    bool detected = (targetState != 0);

    if (detected) {
        lastSeenMs = millis();
        if (!presence) {
            presence = true;
            setRelay(true);
        }
    } else {
        if (presence && (millis() - lastSeenMs > (unsigned long)cfgCooldown * 1000)) {
            presence = false;
            setRelay(false);
        }
    }

    // MQTT
    if (!mqtt.connected()) {
        unsigned long now = millis();
        if (now - lastMqttReconnMs > 5000) {
            lastMqttReconnMs = now;
            mqttConnect();
        }
    } else {
        mqtt.loop();

        if (presence != prevPresence) {
            prevPresence = presence;
            mqttPublish();
        }
        // Heartbeat every 30 s
        if (millis() - lastMqttPublishMs > 30000) {
            lastMqttPublishMs = millis();
            mqttPublish();
        }
    }
}
