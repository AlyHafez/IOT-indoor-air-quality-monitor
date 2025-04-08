#include "Secret.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"


#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

#define RXD1 18  // IO18 on ESP32-S2
#define TXD1 17  // IO17 on ESP32-S2

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

char data[150];
bool newData = false;

void connectAWS() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    configTime(0, 0, "pool.ntp.org");  // Sync time using NTP
while (time(nullptr) < 100000) {
  delay(500);
  Serial.print(".");
}
Serial.println("\nTime synchronized!");

    Serial.println("Connecting to Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

  client.begin(AWS_IOT_ENDPOINT, 8883, net);
    client.onMessage(messageHandler);

    Serial.print("Connecting to AWS IoT...");
    while (!client.connect(THINGNAME)) {
        Serial.print(".");
        delay(500);
    }

    if (!client.connected()) {
        Serial.println("\nAWS IoT Timeout!");
        return;
    }

    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.println("\nAWS IoT Connected!");
}

void setup() {
  Serial.begin(115200);  // USB debug
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);  // UART to STM32
  connectAWS(); 
}

void loop() {
    client.loop();
      static unsigned long lastDebug = 0;

    int i = 0;
    if (Serial1.available() > 0) {//read data on uart
      Serial.println("\nreading data");
        while (Serial1.available() > 0 && i < 149) {
            data[i] = Serial1.read();
            i++;
        }
        data[i] = '\0'; // Null-terminate the string
        Serial.print("Raw data: ");
        Serial.println(data);
        delay(2000);
        newData = true;
        Serial.println("\ndata read");
    }

    if (newData) {
        parseData(data);
        newData = false;
    }
}

void parseData(char *buffer) {
  Serial.println("\nprinting data");

  unsigned int CO = 0, PM25 = 0, checksum = 0;//parse data
  char* line = strtok(buffer, "\n");

  while (line != NULL) {
    if (strstr(line, "CO:") != NULL) {
      CO = atoi(line + 4);  // Skip "CO: "
    } else if (strstr(line, "PM25:") != NULL) {
      PM25 = atoi(line + 6);  // Skip "PM25: "
    } else if (strstr(line, "checksum:") != NULL) {
      checksum = atoi(line + 10);  // Skip "checksum: "
    }
    line = strtok(NULL, "\n");
  }
  unsigned int sum = 0;
  sum = CO + PM25 + checksum;

  Serial.print("sum: "); Serial.println(sum);

 if(sum == 65535){
  Serial.print("Parsed CO: "); Serial.println(CO);
  Serial.print("Parsed PM25: "); Serial.println(PM25);
  Serial.print("Parsed checksum: "); Serial.println(checksum);
  Serial.println("checksum succeeded");

  // skip checksum check for now if you're testing
  publishData(CO, PM25 / 1000.0);
}
else{Serial.println("checksum failed");}
}



void publishData(int CO, float PM_25) {

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["CO"] = CO;
    jsonDoc["PM25"] = PM_25;

    char jsonBuffer[512];
    serializeJson(jsonDoc, jsonBuffer);

bool result = client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);//publish to aws
if (result) {
  Serial.println("✅ Data published to AWS IoT Core:");
} else {
  Serial.println("❌ Failed to publish!");
}
Serial.println(jsonBuffer);

}

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}
