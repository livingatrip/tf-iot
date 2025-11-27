#include <ESP8266WiFi.h>

const char* ssid     = "573_LB_IOT";
const char* password = "udesc2025";

const char* host = "192.168.4.1";   // IP do AP do gateway
const uint16_t port = 5000;         // Porta do servidor TCP do gateway

// No módulo de relé compacto para ESP-01, o relé é acionado pelo GPIO0
// Geralmente: LOW = relé ligado, HIGH = relé desligado
const uint8_t RELAY_PIN = 0;

WiFiClient client;
bool relayOn = false;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // começa DESLIGADO

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao AP ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado ao AP");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  connectToGateway();
}

void loop() {
  if (!client.connected()) {
    Serial.println("Desconectado do gateway, tentando reconectar...");
    connectToGateway();
  }

  // Lê comandos do gateway
  while (client.connected() && client.available()) {
    String msg = client.readStringUntil('\n');
    msg.trim();
    if (msg.length() == 0) continue;

    Serial.print("Comando recebido: ");
    Serial.println(msg);

    if (msg == "RELAY_ON") {
      relayOn = true;
      setRelay();
      sendRelayState();
    } else if (msg == "RELAY_OFF") {
      relayOn = false;
      setRelay();
      sendRelayState();
    } else if (msg == "RELAY_TOGGLE") {
      relayOn = !relayOn;
      setRelay();
      sendRelayState();
    }
  }

  // Heartbeat/identificação periódica (opcional)
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000 && client.connected()) {
    lastHeartbeat = millis();
    client.println("ID:ESP01_RELE");
  }
}

void connectToGateway() {
  while (!client.connect(host, port)) {
    Serial.println("Falha ao conectar no gateway, tentando de novo em 2s...");
    delay(2000);
  }

  Serial.println("Conectado ao gateway!");
  client.println("ID:ESP01_RELE"); // identificação inicial
  sendRelayState(); // já informa o estado atual ao conectar
}

void setRelay() {
  if (relayOn) {
    digitalWrite(RELAY_PIN, LOW);   // Liga (ativo em LOW)
    Serial.println("Rele LIGADO");
  } else {
    digitalWrite(RELAY_PIN, HIGH);  // Desliga
    Serial.println("Rele DESLIGADO");
  }
}

void sendRelayState() {
  if (!client.connected()) return;
  client.print("STATE:RELAY:");
  client.println(relayOn ? "ON" : "OFF");
  Serial.print("Estado enviado ao gateway: ");
  Serial.println(relayOn ? "ON" : "OFF");
}
