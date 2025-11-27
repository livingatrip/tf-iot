#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <coap-simple.h>

// ---------- DISPLAY ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- WIFI AP (para os clients CoAP/TCP) ----------
const char* apSsid = "573_LB_IOT";
const char* apPass = "udesc2025";

// ---------- WIFI STA (para falar com Mosquitto) ----------
const char* staSsid   = "<A_MESMA_REDE_DO_SERVIDOR_LOCAL>"; // <- AJUSTAR se mudar
const char* staPass   = "<SENHA>";                 // <- AJUSTAR se mudar
const char* mqttServer = "172.20.10.3";             // <- IP do Mosquitto
const uint16_t mqttPort = 1883;

WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);

// ---------- HARDWARE ----------
const uint8_t BUTTON_PIN = 14;  // GPIO14 (D5 no NodeMCU)

// ---------- ESTADO DO RELÉ ----------
bool relayOn = false;           // estado real, reportado pelo ESP01_RELE

// ---------- BOTÃO ----------
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ---------- DISPLAY ----------
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 500; // ms

// ---------- CoAP (sensores) ----------
WiFiUDP udp;
Coap coap(udp);                  // porta padrão 5683

// ---------- TCP para o RELÉ ----------
WiFiServer relayServer(5000);    // mesma porta que o ESP01_RELE usa
WiFiClient relayClient;
bool relayClientConnected = false;

// ---------- TIMESTAMPS DOS CLIENTS ----------
unsigned long lastSeenDht   = 0;
unsigned long lastSeenMq135 = 0;
unsigned long lastSeenRele  = 0;
const unsigned long onlineTimeout = 10000; // 10s

// ---------- STATUS MQTT DOS CLIENTS ----------
const char* statusTopic = "lab/status/clients";
unsigned long lastStatusPublish = 0;
const unsigned long statusPublishInterval = 5000; // 5s

// ---------- PROTÓTIPOS ----------
void mqttCallback(char* topic, byte* payload, unsigned int length);
void ensureMqttConnected();
void dadosHandler(CoapPacket &packet, IPAddress ip, int port);
void applyRelayCommand(const String& cmd);
void handleButton();
void updateDisplayIfNeeded();
void handleRelayTcp();
void publishClientsStatusIfNeeded();
bool isOnline(unsigned long lastSeen, unsigned long now);

// ==================== FUNÇÃO AUXILIAR ====================

// Considera OFF até receber a 1ª mensagem (lastSeen == 0)
bool isOnline(unsigned long lastSeen, unsigned long now) {
  if (lastSeen == 0) return false;
  return (now - lastSeen) < onlineTimeout;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // --- OLED ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Falha ao iniciar OLED");
    while (true) { delay(1000); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Inicializando...");
  display.display();

  // --- WiFi AP + STA ---
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid, apPass);
  IPAddress apIp = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(apIp);

  Serial.print("Conectando STA em ");
  Serial.println(staSsid);
  WiFi.begin(staSsid, staPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nSTA IP: ");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Gateway ESP");
  display.print("AP IP: ");
  display.println(apIp);
  display.print("STA IP: ");
  display.println(WiFi.localIP());
  display.display();

  // --- Servidor TCP do relé ---
  relayServer.begin();
  Serial.println("Servidor TCP do relé iniciado na porta 5000");

  // --- MQTT ---
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  ensureMqttConnected();

  // --- CoAP: recurso /dados para receber DHT11 e MQ135 ---
  coap.server(dadosHandler, "dados");
  coap.start();

  Serial.println("Gateway CoAP + MQTT + TCP(relé) iniciado.");
}

// ==================== LOOP ====================
void loop() {
  ensureMqttConnected();
  mqttClient.loop();   // processa mensagens MQTT
  coap.loop();         // processa mensagens CoAP (DHT/MQ135)

  handleRelayTcp();    // gerencia conexão TCP com o relé

  handleButton();
  updateDisplayIfNeeded();
  publishClientsStatusIfNeeded(); // publica status DHT/MQ135/RELE
}

// ==================== MQTT ====================

void ensureMqttConnected() {
  if (mqttClient.connected()) return;

  Serial.print("Conectando ao MQTT em ");
  Serial.print(mqttServer);
  Serial.print(":");
  Serial.println(mqttPort);

  String clientId = "ESP_Gateway_";
  clientId += String(ESP.getChipId(), HEX);

  while (!mqttClient.connected()) {
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Conectado ao MQTT!");
      // Assina o tópico de comando do relé
      mqttClient.subscribe("lab/rele/cmd");
    } else {
      Serial.print("Falha MQTT, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" tentando novamente em 3s");
      delay(3000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();

  Serial.print("MQTT [");
  Serial.print(t);
  Serial.print("] ");
  Serial.println(msg);

  if (t == "lab/rele/cmd") {
    // msg: "ON", "OFF", "TOGGLE"
    applyRelayCommand(msg);
  }
}

// ==================== TCP do RELÉ ====================

void handleRelayTcp() {
  // Se ainda não temos cliente ligado, tenta aceitar um novo
  if (!relayClientConnected) {
    WiFiClient newClient = relayServer.available();
    if (newClient) {
      relayClient = newClient;
      relayClientConnected = true;
      Serial.print("Relé conectado via TCP: ");
      Serial.println(relayClient.remoteIP());
      // Marca como visto agora que conectou
      lastSeenRele = millis();
    }
  } else {
    // Se já temos cliente, verifica se continua conectado
    if (!relayClient.connected()) {
      Serial.println("Relé TCP desconectado.");
      relayClient.stop();
      relayClientConnected = false;
    }
  }

  if (!relayClientConnected) return;

  // Lê mensagens do relé
  while (relayClient.available()) {
    String msg = relayClient.readStringUntil('\n');
    msg.trim();
    if (msg.length() == 0) continue;

    Serial.print("TCP do relé -> ");
    Serial.println(msg);

    // QUALQUER mensagem indica que o relé está vivo
    lastSeenRele = millis();

    if (msg.startsWith("ID:")) {
      // Ex: ID:ESP01_RELE
      Serial.print("ID do relé: ");
      Serial.println(msg.substring(3));
    }
    else if (msg.startsWith("STATE:RELAY:")) {
      String s = msg.substring(String("STATE:RELAY:").length());
      s.trim();
      relayOn = (s == "ON");

      const char* stText = relayOn ? "ON" : "OFF";
      Serial.print("Estado reportado pelo relé: ");
      Serial.println(stText);

      // publica no MQTT
      mqttClient.publish("lab/rele/state", stText, true);
    }
  }
}

// ==================== CoAP: /dados (sensores) ====================

// Recebe JSON de DHT11 e MQ135
void dadosHandler(CoapPacket &packet, IPAddress ip, int port) {
  char buf[packet.payloadlen + 1];
  memcpy(buf, packet.payload, packet.payloadlen);
  buf[packet.payloadlen] = 0;

  Serial.print("CoAP /dados de ");
  Serial.print(ip);
  Serial.print(": ");
  Serial.println(buf);

  // publica JSON bruto num tópico (se quiser)
  mqttClient.publish("lab/raw", buf);

  // DHT11 -> {"id":"dht01","temp":X,"umid":Y}
  if (strstr(buf, "\"id\":\"dht01\"")) {
    float temp = 0, umid = 0;
    int parsed = sscanf(buf, "{\"id\":\"dht01\",\"temp\":%f,\"umid\":%f}", &temp, &umid);
    if (parsed == 2) {
      char t[16], h[16];
      dtostrf(temp, 4, 1, t);
      dtostrf(umid, 4, 1, h);
      mqttClient.publish("lab/dht11/temp", t);
      mqttClient.publish("lab/dht11/hum",  h);
      Serial.print("DHT -> temp=");
      Serial.print(t);
      Serial.print(" umid=");
      Serial.println(h);
      lastSeenDht = millis();
    } else {
      Serial.println("⚠ Erro parse DHT JSON");
    }
  }
  // MQ135 -> {"id":"mq135","air":X}
  else if (strstr(buf, "\"id\":\"mq135\"")) {
    float air = 0;
    int parsed = sscanf(buf, "{\"id\":\"mq135\",\"air\":%f}", &air);
    if (parsed == 1) {
      char a[16];
      dtostrf(air, 4, 1, a);
      mqttClient.publish("lab/mq135", a);
      Serial.print("MQ135 -> air=");
      Serial.println(a);
      lastSeenMq135 = millis();
    } else {
      Serial.println("⚠ Erro parse MQ135 JSON");
    }
  } else {
    Serial.println("⚠ id desconhecido no JSON recebido (não é DHT/MQ135)");
  }

  // responde "OK" pro client
  coap.sendResponse(ip, port, packet.messageid, "OK");
}

// ==================== LÓGICA DO RELÉ (botão + MQTT → TCP) ====================

void applyRelayCommand(const String& cmd) {
  // Só aceitamos esses comandos
  if (cmd != "ON" && cmd != "OFF" && cmd != "TOGGLE") {
    Serial.print("Comando desconhecido: ");
    Serial.println(cmd);
    return;
  }

  Serial.println("=== applyRelayCommand (TCP) ===");
  Serial.print("Comando recebido: ");
  Serial.println(cmd);

  if (!relayClientConnected || !relayClient.connected()) {
    Serial.println("⚠ Relé TCP não conectado, não é possível enviar comando.");
    return;
  }

  String wire;
  if (cmd == "ON")          wire = "RELAY_ON";
  else if (cmd == "OFF")    wire = "RELAY_OFF";
  else if (cmd == "TOGGLE") wire = "RELAY_TOGGLE";

  Serial.print("Enviando para o relé via TCP: ");
  Serial.println(wire);

  relayClient.println(wire);

  // NÃO mexe em relayOn aqui.
  // Quem confirma o estado é o próprio relé,
  // quando manda "STATE:RELAY:ON/OFF" de volta.
}

// ==================== STATUS MQTT DOS CLIENTS ====================

void publishClientsStatusIfNeeded() {
  if (!mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastStatusPublish < statusPublishInterval) return;
  lastStatusPublish = now;

  bool dhtOnline   = isOnline(lastSeenDht,   now);
  bool mqOnline    = isOnline(lastSeenMq135, now);
  bool releOnline  = isOnline(lastSeenRele,  now);

  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"gateway\":\"ON\",\"dht\":\"%s\",\"mq135\":\"%s\",\"rele\":\"%s\"}",
           dhtOnline   ? "ON" : "OFF",
           mqOnline    ? "ON" : "OFF",
           releOnline  ? "ON" : "OFF");

  Serial.print("Publicando status dos clients em ");
  Serial.print(statusTopic);
  Serial.print(": ");
  Serial.println(payload);

  // retained = true para dashboards sempre verem o último status
  mqttClient.publish(statusTopic, payload, true);
}

// ==================== BOTÃO ====================

void handleButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    static int stableState = HIGH;

    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {
        Serial.println("Botao pressionado -> TOGGLE relé");
        applyRelayCommand("TOGGLE");
      }
    }
  }

  lastButtonState = reading;
}

// ==================== DISPLAY ====================

void updateDisplayIfNeeded() {
  if (millis() - lastDisplayUpdate < displayInterval) return;
  lastDisplayUpdate = millis();

  unsigned long now = millis();

  bool dhtOnline   = isOnline(lastSeenDht,   now);
  bool mqOnline    = isOnline(lastSeenMq135, now);
  bool releOnline  = isOnline(lastSeenRele,  now);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Gateway ESP");

  // Estado lógico do relé (último conhecido)
  display.print("Relay: ");
  display.println(relayOn ? "ON " : "OFF");

  // Conectividade dos dispositivos
  display.print("DHT:  ");
  display.println(dhtOnline   ? "ON" : "OFF");

  display.print("MQ135:");
  display.println(mqOnline    ? "ON" : "OFF");

  display.print("RELE: ");
  display.println(releOnline  ? "ON" : "OFF");

  display.display();
}
