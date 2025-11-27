#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <DHT.h>

// ====== WIFI ======
const char* apSsid = "573_LB_IOT";
const char* apPass = "udesc2025";

// ====== GATEWAY CoAP ======
// AJUSTE PARA O IP REAL DO GATEWAY NA REDE
IPAddress gatewayIP(192, 168, 4, 1);
const int gatewayPort = 5683;

// ====== DHT11 ======
#define DHTPIN 2        // GPIO2 no ESP-01S
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ====== CoAP CLIENTE ======
WiFiUDP udp;
Coap coap(udp, 5683);   // porta local CoAP do cliente

// callback opcional para resposta do gateway
void callback_response(CoapPacket &packet, IPAddress ip, int port) {
  Serial.print("Resposta CoAP do gateway: ");
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = 0;
  Serial.println(p);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Conectando ao WiFi...");
  WiFi.begin(apSsid, apPass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado!");
  Serial.print("IP do ESP-01S: ");
  Serial.println(WiFi.localIP());

  dht.begin();

  // configura callback de resposta e inicia CoAP
  coap.response(callback_response);
  coap.start();
}

void loop() {
  // processa CoAP interno (retransmissão, respostas, etc.)
  coap.loop();

  static unsigned long lastSend = 0;
  unsigned long now = millis();

  // envia a cada 5 segundos
  if (now - lastSend >= 5000) {
    lastSend = now;

    float temp = dht.readTemperature(); // °C
    float umid = dht.readHumidity();    // %

    if (isnan(temp) || isnan(umid)) {
      Serial.println("Falha ao ler DHT11");
      return;
    }

    // Monta JSON: {"temp":25.3,"umid":60.1}
    char msg[64];
    snprintf(msg, sizeof(msg),
         "{\"id\":\"dht01\",\"temp\":%.1f,\"umid\":%.1f}",
         temp, umid);

    Serial.print("Enviando CoAP PUT para gateway: ");
    Serial.println(msg);

    // forma suportada pela lib: payload como const char*
    coap.put(gatewayIP, gatewayPort, "dados", msg);
  }
}
