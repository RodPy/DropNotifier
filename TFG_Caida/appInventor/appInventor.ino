#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

MDNSResponder mdns;

const char* ssid = "RS_PC";
const char* password = "chocolate";

WebServer server(80);
String webpage = "";

int led1 = 4;
int led2 = 5;

void setup(void) {

  webpage += "<h1>ESP32 Web Server</h1><p>LED at GPIO4 <a href=\"led1ON\"><button>ON</button></a>&nbsp;<a href=\"led1OFF\"><button>OFF</button></a></p>";
  webpage += "<p>LED at GPIO5 <a href=\"led2ON\"><button>ON</button></a>&nbsp;<a href=\"led2OFF\"><button>OFF</button></a></p>";

  pinMode(led1, OUTPUT);
  digitalWrite(led1, LOW);
  pinMode(led2, OUTPUT);
  digitalWrite(led2, LOW);
  delay(1000);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
   if (mdns.begin("esp")) {
    Serial.println("MDNS responder started");
  }
  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });
  
  server.on("/led1ON", []() {
    server.send(200, "text/html", webpage);
    digitalWrite(led1, HIGH);
    delay(1000);
  });
  
  server.on("/led1OFF", []() {
    server.send(200, "text/html", webpage);
    digitalWrite(led1, LOW);
    delay(1000);
  });
 
  server.on("/led2ON", []() {
    server.send(200, "text/html", webpage);
    digitalWrite(led2, HIGH);
    delay(1000);
  });

  server.on("/led2OFF", []() {
    server.send(200, "text/html", webpage);
    digitalWrite(led2, LOW);
    delay(1000);
  });
  server.begin();
  Serial.println("HTTP server started");
}
void loop(void) {
  server.handleClient();
}
