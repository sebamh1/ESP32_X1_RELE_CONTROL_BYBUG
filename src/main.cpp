/*
 * ============================================================
 *  Blue Metrics — Control Extractores Sala de Producción
 *  Placa : ESP32-WROOM-32E (SimplyMaker 4-ch relay module)
 *  Relés : GPIO32 (R1), GPIO33 (R2), GPIO25 (R3)
 *  Cloud : AWS IoT Core vía MQTT/TLS
 * ============================================================
 *  Topics MQTT:
 *    SUB  sala/extractores/cmd     ← comandos desde Lambda/Grafana
 *    PUB  sala/extractores/status  → estado actual cada STATUS_INTERVAL_MS
 * ============================================================
 *  Librerías requeridas (Library Manager):
 *    - ArduinoJson   >= 7.x   (Benoit Blanchon)
 *    - PubSubClient  >= 2.8   (Nick O'Leary)
 *    - WiFiClientSecure (incluida en ESP32 Arduino Core)
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

// ── Pines relés ───────────────────────────────────────────────────────────────
//  Módulo SimplyMaker: HIGH energiza el relé, LOW lo desactiva
//  (verificado en terreno 15-oct-2026; si cambia el módulo, invertir aquí)
//  Extractores en terminal NC:
//    Extractor ON  → relé DESACTIVADO → GPIO RELAY_IDLE
//    Extractor OFF → relé ACTIVADO    → GPIO RELAY_ACTIVE
#define RELAY_ACTIVE   HIGH
#define RELAY_IDLE     LOW
#define RELAY_PIN_1    32
#define RELAY_PIN_2    33
#define RELAY_PIN_3    25
#define NUM_RELAYS     3

const int RELAY_PINS[NUM_RELAYS] = { RELAY_PIN_1, RELAY_PIN_2, RELAY_PIN_3 };

// ── Topics MQTT ───────────────────────────────────────────────────────────────
#define TOPIC_CMD      "sala/extractores/cmd"
#define TOPIC_STATUS   "sala/extractores/status"

// ── Intervalos ────────────────────────────────────────────────────────────────
#define STATUS_INTERVAL_MS    30000UL  // publica estado cada 30 s
#define RECONNECT_INTERVAL_MS  5000UL  // reintento conexión cada 5 s
#define FAILSAFE_TIMEOUT_MS  300000UL  // sin MQTT por 5 min → extractores ON

// ── Estado del sistema ────────────────────────────────────────────────────────
struct SystemState {
    bool  relay[NUM_RELAYS] = { true, true, true };  // fail-safe: arrancan ON
    char  mode[8]           = "manual";  // "auto" | "manual"
    float temp_on           = 28.0f;
    float temp_off          = 24.0f;
    float last_temp         = 0.0f;
    float co2_on            = 1500.0f;
    float co2_off           = 1000.0f;
    float last_co2          = 0.0f;
    bool  co2_received      = false;
    bool  config_received   = false;
};

static SystemState state;

// ── Clientes WiFi / MQTT ──────────────────────────────────────────────────────
static WiFiClientSecure net;
static PubSubClient     mqtt(net);

// ── Timers ────────────────────────────────────────────────────────────────────
static unsigned long lastStatusMs    = 0;
static unsigned long lastReconnectMs = 0;
static unsigned long lastMqttOkMs    = 0;
static bool          failsafeActive  = false;

// ─────────────────────────────────────────────────────────────────────────────
//  Control de relés
// ─────────────────────────────────────────────────────────────────────────────

void setRelay(int index, bool on) {
    if (index < 0 || index >= NUM_RELAYS) return;
    state.relay[index] = on;
    // Extractores en NC: encender extractor = desactivar relé
    //                    apagar extractor   = activar relé
    digitalWrite(RELAY_PINS[index], on ? RELAY_IDLE : RELAY_ACTIVE);
    Serial.printf("[RELAY] R%d → extractor %s (relé %s)\n",
                  index + 1,
                  on ? "ON" : "OFF",
                  on ? "DESACTIVADO" : "ACTIVADO");
}

void setAllRelays(bool on) {
    for (int i = 0; i < NUM_RELAYS; i++) setRelay(i, on);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lógica automática
// ─────────────────────────────────────────────────────────────────────────────

void evaluateAutoControl() {
    if (strcmp(state.mode, "auto") != 0) return;

    // CO2 solo participa si ya llegó al menos una lectura
    bool co2High = state.co2_received && state.last_co2 >= state.co2_on;
    bool co2Low  = !state.co2_received || state.last_co2 <= state.co2_off;

    // Enciende si CUALQUIERA supera su umbral ON;
    // apaga solo si AMBOS están bajo su umbral OFF
    if (state.last_temp >= state.temp_on || co2High) {
        Serial.printf("[AUTO] Temp %.1f°C (ON≥%.1f) CO2 %.0fppm (ON≥%.0f) → encendiendo extractores\n",
                      state.last_temp, state.temp_on, state.last_co2, state.co2_on);
        setAllRelays(true);
    } else if (state.last_temp <= state.temp_off && co2Low) {
        Serial.printf("[AUTO] Temp %.1f°C (OFF≤%.1f) CO2 %.0fppm (OFF≤%.0f) → apagando extractores\n",
                      state.last_temp, state.temp_off, state.last_co2, state.co2_off);
        setAllRelays(false);
    }
    // Zona histéresis: no se hace nada
}

// ─────────────────────────────────────────────────────────────────────────────
//  Lectura tolerante de campos JSON
//  El panel de Grafana toma valores de atributos data-* (siempre strings),
//  así que se aceptan tanto  true / 1 / "true" / "on"  como  26.5 / "26.5"
// ─────────────────────────────────────────────────────────────────────────────

bool readBool(JsonVariantConst v, bool& out) {
    if (v.is<bool>()) { out = v.as<bool>(); return true; }
    if (v.is<long>()) { out = v.as<long>() != 0; return true; }
    if (v.is<const char*>()) {
        const char* s = v.as<const char*>();
        if (!strcasecmp(s, "true")  || !strcasecmp(s, "on")  || !strcmp(s, "1")) { out = true;  return true; }
        if (!strcasecmp(s, "false") || !strcasecmp(s, "off") || !strcmp(s, "0")) { out = false; return true; }
    }
    return false;
}

bool readFloat(JsonVariantConst v, float& out) {
    if (v.is<float>()) { out = v.as<float>(); return true; }
    if (v.is<const char*>()) {
        const char* s = v.as<const char*>();
        char* end;
        float f = strtof(s, &end);
        if (end != s) { out = f; return true; }
    }
    return false;
}

bool readMode(JsonVariantConst v, char* out, size_t len) {
    if (!v.is<const char*>()) return false;
    const char* s = v.as<const char*>();
    if (!strcasecmp(s, "auto"))   { strlcpy(out, "auto",   len); return true; }
    if (!strcasecmp(s, "manual")) { strlcpy(out, "manual", len); return true; }
    Serial.printf("[CMD] Modo desconocido '%s'\n", s);
    return false;
}

// Aplica umbrales presentes en el mensaje; devuelve true si cambió alguno
bool applyThresholds(JsonDocument& doc) {
    bool changed = false;
    changed |= readFloat(doc["temp_on"],  state.temp_on);
    changed |= readFloat(doc["temp_off"], state.temp_off);
    changed |= readFloat(doc["co2_on"],   state.co2_on);
    changed |= readFloat(doc["co2_off"],  state.co2_off);
    return changed;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Publicar estado
// ─────────────────────────────────────────────────────────────────────────────

void publishStatus() {
    JsonDocument doc;
    doc["device_id"] = THINGNAME;
    doc["mode"]      = state.mode;
    doc["temp_on"]   = state.temp_on;
    doc["temp_off"]  = state.temp_off;
    doc["last_temp"] = state.last_temp;
    doc["co2_on"]    = state.co2_on;
    doc["co2_off"]   = state.co2_off;
    doc["last_co2"]  = state.last_co2;

    JsonObject relays = doc["relays"].to<JsonObject>();
    relays["relay_1"] = state.relay[0];
    relays["relay_2"] = state.relay[1];
    relays["relay_3"] = state.relay[2];

    doc["uptime_s"] = millis() / 1000;

    char buf[400];
    size_t n = serializeJson(doc, buf);
    if (n >= sizeof(buf) - 1) {
        Serial.println("[MQTT] Status truncado — aumentar buffer");
        return;
    }
    if (mqtt.publish(TOPIC_STATUS, buf)) {
        Serial.printf("[MQTT] Status publicado: %s\n", buf);
    } else {
        Serial.println("[MQTT] Fallo al publicar status");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callback MQTT — procesa mensajes entrantes
// ─────────────────────────────────────────────────────────────────────────────

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    char buf[512] = {};
    if (length >= sizeof(buf)) {
        Serial.println("[MQTT] Mensaje demasiado largo, ignorado");
        return;
    }
    memcpy(buf, payload, length);

    Serial.printf("[MQTT] Recibido en %s: %s\n", topic, buf);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf);
    if (err) {
        Serial.printf("[MQTT] JSON inválido: %s\n", err.c_str());
        return;
    }

    const char* action = doc["action"] | "";

    // ── set_config / set_threshold ────────────────────────────────────────────
    if (!strcmp(action, "set_config") || !strcmp(action, "set_threshold") ||
        !strcmp(action, "set_thresholds")) {
        applyThresholds(doc);
        readMode(doc["mode"], state.mode, sizeof(state.mode));
        // relay_all: enviado por Lambda AUTO para cambiar todos los relés a la vez
        bool target;
        if (readBool(doc["relay_all"], target)) {
            Serial.printf("[CMD] set_config relay_all → extractores %s\n",
                          target ? "ON" : "OFF");
            setAllRelays(target);
        }
        state.config_received = true;
        Serial.printf("[CMD] %s → mode=%s temp_on=%.1f temp_off=%.1f co2_on=%.0f co2_off=%.0f\n",
                      action, state.mode, state.temp_on, state.temp_off,
                      state.co2_on, state.co2_off);
        evaluateAutoControl();
        publishStatus();
    }

    // ── set_mode ──────────────────────────────────────────────────────────────
    else if (strcmp(action, "set_mode") == 0) {
        if (readMode(doc["mode"], state.mode, sizeof(state.mode))) {
            Serial.printf("[CMD] set_mode → %s\n", state.mode);
            evaluateAutoControl();
            publishStatus();
        }
    }

    // ── relay_cmd ─────────────────────────────────────────────────────────────
    else if (strcmp(action, "relay_cmd") == 0) {
        if (strcmp(state.mode, "auto") == 0) {
            Serial.println("[CMD] relay_cmd ignorado — sistema en modo AUTO");
            publishStatus();  // re-sincroniza el panel con el modo real
            return;
        }
        float relayF = 0;
        readFloat(doc["relay"], relayF);  // acepta 1 o "1"
        int relayNum = (int)relayF;

        bool relayOn;
        if (!readBool(doc["state"], relayOn)) {
            Serial.println("[CMD] relay_cmd: campo 'state' ausente o inválido");
            return;
        }
        if (relayNum < 1 || relayNum > NUM_RELAYS) {
            Serial.printf("[CMD] relay_cmd: número de relé inválido (%d)\n", relayNum);
            return;
        }
        setRelay(relayNum - 1, relayOn);
        publishStatus();
    }

    // ── temp_update / co2_update / sensor_update ──────────────────────────────
    else if (!strcmp(action, "temp_update") || !strcmp(action, "co2_update") ||
             !strcmp(action, "sensor_update")) {
        bool got = false;
        if (readFloat(doc["temperature"], state.last_temp)) {
            Serial.printf("[CMD] %s → temp %.2f°C\n", action, state.last_temp);
            got = true;
        }
        if (readFloat(doc["co2"], state.last_co2)) {
            state.co2_received = true;
            Serial.printf("[CMD] %s → CO2 %.0f ppm\n", action, state.last_co2);
            got = true;
        }
        if (got) evaluateAutoControl();
    }

    else {
        Serial.printf("[CMD] Acción desconocida: '%s'\n", action);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conexión WiFi
// ─────────────────────────────────────────────────────────────────────────────

void connectWifi() {
    Serial.printf("[WiFi] Conectando a %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t > 15000UL) {
            Serial.println("\n[WiFi] Timeout — reiniciando");
            ESP.restart();
        }
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[WiFi] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Conexión MQTT (AWS IoT Core)
// ─────────────────────────────────────────────────────────────────────────────

bool connectMqtt() {
    Serial.printf("[MQTT] Conectando a %s...\n", AWS_IOT_ENDPOINT);

    if (mqtt.connect(THINGNAME)) {
        Serial.println("[MQTT] Conectado");
        // Si la policy de AWS IoT no permite iot:Subscribe/iot:Receive en este
        // topic, el broker rechaza la suscripción y los comandos nunca llegan
        if (!mqtt.subscribe(TOPIC_CMD, 1)) {  // QoS 1
            Serial.printf("[MQTT] ERROR al suscribirse a %s — revisar policy\n", TOPIC_CMD);
            mqtt.disconnect();
            return false;
        }
        Serial.printf("[MQTT] Suscrito a %s\n", TOPIC_CMD);
        publishStatus();
        return true;
    }

    Serial.printf("[MQTT] Fallo, rc=%d\n", mqtt.state());
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[BOOT] Blue Metrics — Control Extractores Sala");

    // Fail-safe: relés en reposo al boot → NC cerrado → extractores ON.
    // Es el mismo estado que con el controlador sin energía.
    // Se fija el nivel antes de pinMode para evitar un pulso al configurar el pin.
    for (int i = 0; i < NUM_RELAYS; i++) {
        digitalWrite(RELAY_PINS[i], RELAY_IDLE);
        pinMode(RELAY_PINS[i], OUTPUT);
    }
    Serial.println("[BOOT] Relés en reposo (extractores ON por terminal NC)");

    connectWifi();

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    mqtt.setServer(AWS_IOT_ENDPOINT, 8883);
    mqtt.setCallback(onMqttMessage);
    mqtt.setBufferSize(512);
    mqtt.setKeepAlive(60);
    mqtt.setSocketTimeout(15);

    connectMqtt();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    // Mantener conexión WiFi
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Desconectado — reconectando...");
        connectWifi();
    }

    // Sin conexión MQTT: reintentar con backoff
    if (!mqtt.connected()) {
        unsigned long now = millis();

        // Fail-safe: sin control remoto por mucho tiempo → extractores ON
        if (!failsafeActive && now - lastMqttOkMs >= FAILSAFE_TIMEOUT_MS) {
            Serial.println("[FAILSAFE] Sin MQTT — encendiendo extractores");
            setAllRelays(true);
            failsafeActive = true;
        }

        if (now - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
            lastReconnectMs = now;
            if (connectMqtt()) {
                lastStatusMs = millis();  // reset timer tras reconexión exitosa
            }
        }
        return;  // no continuar si no hay conexión
    }

    // mqtt.loop() debe llamarse en cada iteración para procesar mensajes entrantes
    mqtt.loop();
    lastMqttOkMs   = millis();
    failsafeActive = false;

    // Publicar estado periódicamente
    unsigned long now = millis();
    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = now;
        publishStatus();
    }
}
