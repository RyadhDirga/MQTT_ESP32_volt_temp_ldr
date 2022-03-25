#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#ifdef __cplusplus 
extern "C" {
#endif 
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADDRESS";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];

int value = 0;
int analogPin = 34;
int LDRPin = 35;

float temperature = 0;
float volt = 0;
float Vmodul = 0.0; 
float hasil = 0.0;
float R1 = 30000.0; //30k
float R2 = 7500.0; //7500 ohm resistor,

void setup() {
  Serial.begin(115200);  
  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(analogPin, INPUT);
  pinMode(LDRPin, INPUT);
}


void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String tugas;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    tugas += (char)message[i];
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {;
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    temperature = (temprature_sens_read() - 32) / 1.8;   
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);

    value = analogRead(analogPin);
    Vmodul = (value * 5.0) / 1024.0;
    hasil = Vmodul / (R2/(R1+R2));
    char humString[8];
    dtostrf(hasil, 1, 2, humString);
    Serial.print("volt: ");
    Serial.println(humString);
    client.publish("esp32/volt", humString);

    int LDR = analogRead(LDRPin);
    char LDRString[8];
    dtostrf(LDR, 1, 2, humString);
    Serial.print("volt: ");
    Serial.println(humString);
    client.publish("esp32/LDR", humString);
  }
}
