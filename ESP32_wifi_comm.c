#include "Secret.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"


#define AWS_IOT_PUBLISH_TOPIC   "ESP32_CO2/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "ESP32_CO2/sub"
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

char data[150];
bool newData = false;

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

    net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);


  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.println(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void setup() {
  Serial.begin(115200);
  connectAWS();
}

void loop() {
  client.loop();
  int i = 0;
  if (Serial.available() > 0) {
    while (Serial.available() > 0 && i < 149) {
      data[i] = Serial.read();
      i++;
    }
    data[i] = '\0'; // Null-terminate the string
    newData = true;
  }

  if (newData) {
    parseData(data);
    newData = false;
  }
}
void parseData(char *buffer){
  int value[4];
  int i = 0;
  char* delim = "\n";
  char *res= strtok(buffer, delim);
  while(res!=NULL&&i<4){
    value[i]= atoi(res);
    i++;
    res = strtok(NULL,delim);

  }
  if (i==4){
    int CO2 = value[0];
    int CO = value[1];
    int PM25 = value[2];
    int checksum = value[3];
      int sum = CO2+CO+PM25+checksum;
  if (sum>=0x10000){
    sum = (sum&0xFFFF) + 1;
  }
  if (sum == 0xFFFF){
    Serial.println("value is correct publishing data:");
    publishData(CO2,CO,PM25/1000);
  }
  else{Serial.println("error occured, checksum in valid");}
  }
}
void publishData(int CO2, int CO, float PM_25){
    StaticJsonDocument<200> jsonDoc;
  jsonDoc["CO2"] = CO2;
  jsonDoc["CO"] = CO;
  jsonDoc["PM2.5"] = PM_25;
  char jsonBuffer[512];
  serializeJson(jsonDoc, jsonBuffer);
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  Serial.println("Data published to AWS IoT Core:");
  Serial.println(jsonBuffer);
}
