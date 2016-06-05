#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "1883";
char NodeID[16] = "rx/8";
char* char_mqtt_data;

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]); //strcpy returns a char array from a strint strcpy(destination, string)
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(NodeID, json["NodeID"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_NodeID("NodeID", "NodeID", NodeID, 15);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);


  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_NodeID);

  //reset settings - for testing
  wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration

  //get MAC address to give unique name...

  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "Mark's ESP emonTx " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);


  if (!wifiManager.autoConnect(AP_NameChar, "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(NodeID, custom_NodeID.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["NodeID"] = NodeID;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  // call setup off other programs
  setupEmonTX();

  //setupMQTT(char mqtt_server, int mqtt_port) {
  // looks like the Json code returns character arrays. Pass them in and deal with conversion in setupMQTT.
  setupMQTT(mqtt_server, mqtt_port);

}

void wifiReconnect() {
  WiFiManager wifiManager;


  //get MAC address to give unique name...

  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "Mark's ESP emonTx " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);



  if (!wifiManager.autoConnect(AP_NameChar, "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");




}

void loop() {

  // three main steps here
  // read data
  char* mqtt_data = measure_power(); //type mismatch here
  Serial.println("data collected");
  Serial.println(mqtt_data);
  //check wifi connection and reconnect if needed
  //checkWiFi(ssid, passowrd);
//  if (WiFi.status() != WL_CONNECTED) {
////        WiFiManager wifiManager;
//  //      wifiManager.connectWifi; //uses saved parameters from config web page
//  }

  // send via MQTT
  //send_MQTT_data(mqtt_topic, mqtt_data);
  //  mqtt_server = wifiManager.getValue(mqtt_server); //not sure how this will behave in the event of a reset/poer failure
  //  mqtt_port = wifiManager.getValue(mqtt_port);
  //  mqtt_topic = wifiManager.getValue(mqtt_topic);
  Serial.print("Server: ");
  Serial.print(mqtt_server);
  Serial.print("\n");
  Serial.print("Port: ");
  Serial.println(mqtt_port);
  Serial.print("\n");
  Serial.print("NodeID: ");
  Serial.println(NodeID);
  Serial.print("\n");

  Serial.println("Start send MQTT data");

  // nodeID comes in as char* already.
  // MQTT data is a long string, which needs to be converted.
  
  Serial.println("Prepare MQTT packets");
  //mqtt_topic.toCharArray(char_mqtt_topic,mqtt_topic.length()+1);
  //mqtt_data="test packet";
  //mqtt_data.toCharArray(char_mqtt_data,mqtt_data.length()+1);
  
  // max mqtt packet size is 128.
  // publish is expecting char* variables
  
  send_MQTT_data(NodeID, mqtt_data);
  Serial.println("Finished data transmit");
  delay(300);
}


