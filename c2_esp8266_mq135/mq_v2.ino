#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>

// ====== WIFI ======
const char* apSsid = "573_LB_IOT";
const char* apPass = "udesc2025";

// ====== GATEWAY CoAP ======
IPAddress gatewayIP(192, 168, 4, 1); // AJUSTE pro IP do gateway
const int gatewayPort = 5683;

// ====== MQ135 ======
#define MQ135_PIN A0

// ====== CoAP ======
WiFiUDP udp;
Coap coap(udp);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Conectando WiFi (MQ135)...");
  WiFi.begin(apSsid, apPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado (MQ135)");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  coap.start();
}

void loop() {
  coap.loop();

  static unsigned long lastSend = 0;
  unsigned long now = millis();

  if (now - lastSend >= 5000) { // a cada 5s
    lastSend = now;

    int raw = analogRead(MQ135_PIN); // 0-1023
    float air = (float) raw;         // se quiser, depois calibra pra ppm

    char json[48];
    snprintf(json, sizeof(json), "{\"id\":\"mq135\",\"air\":%.1f}", air);

    Serial.print("MQ135 enviando: ");
    Serial.println(json);

    coap.put(gatewayIP, gatewayPort, "dados", json);
  }
}
