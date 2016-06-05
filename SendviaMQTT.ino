#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <WiFiManager.h>
#include <PubSubClient.h>                                               //MQTT library

//..........................Wifi Config.........................................................
WiFiClient espClient;
PubSubClient client(espClient);

//..........................MQTT Config.........................................................
char* topic; // emonCMS with subscribe and receive data from topics rx/*
char msg[75];


void setupMQTT(char* mqtt_server, char* mqtt_port) {
  //  uint8_t ip_sector[4];
  uint16_t int_mqtt_port;
  int_mqtt_port = atoi(mqtt_port);
//
//  Serial.print("MQTT Server: ");
//  Serial.println(mqtt_server);
//  Serial.print("MQTT Port: ");
//  Serial.println(int_mqtt_port);

  //client.setServer("192.168.1.151", int_mqtt_port);
  client.setServer(mqtt_server, int_mqtt_port);
  //client.setCallback(callback);
  
  //  // connect to MQTT broker
  if (!client.connected()) {
    Serial.println("not conneccted to MQTT");
    reconnect();
  }
  else {
    Serial.println("connected to MQTT");
  }
  
  client.loop();
}


void send_MQTT_data(char* mqtt_topic, char* mqtt_data) {
  
  Serial.println("Checking MQTT connection");
  if (!client.connected()) {
    reconnect();
  }
  Serial.println("Connected to MQTT");
  Serial.println("Packets done. Publishing");
  client.publish(mqtt_topic,mqtt_data);
  //client.publish(char_mqtt_topic, char_mqtt_data);
  Serial.println("MQTT publish done");
}


void callback(char* topic, byte * payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "ESP8266 node re/connected");
      Serial.println("c");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


