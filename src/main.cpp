// Program for ESP32 Sensor_Test1                    KLH V0.9 26.12.2022
// serial connection (via USB Bridge CP2102), setup WIFI
// reading Chip-Data, MAC-Adress
// sending (coloured) output via serial Port (115200 Baud)
// scanning for WIFI-Networks, Ping, starting NTP client (UDP), 
// simple website at spezified IP adress for switching GPIO (LED pin 4)
// scanning OneWire BUS for Dallas Temperature sensor (DS18B20)
// implementing simple MQTT-Client

#include <Arduino.h>
#include <Wire.h>
#include "WiFi.h"
#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson.git
#include <NTPClient.h>          // https://github.com/taranais/NTPClient
#include <string.h>
#include <ESP32Ping.h>
#include <esp_task_wdt.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"   // Brown out detection
#include <OneWire.h>            // OneWire bus for Sensors
#include <DallasTemperature.h>  // read Dallas Temp. Sensors
#include <PubSubClient.h>       // MQTT
#include <bits/stdc++.h>
#include "hw_settings.h"

using namespace std;

// #include <ArduinoOTA.h>         // over the air update (optional)

#define Version_SW "ESP32-AZWROOM-0.9"
#define Version_HW "ESP32-AZWROOM-BR-0.9"

#define WDT_TIMEOUT 8           // Watchdog Timer 8s

String client_Id = "ESP32-";   
String client_MAC = "";
uint32_t chipId = 0;
// hardware wireing 

const int LED = 4;              // LED pin 5
const int BUTTON = 16;
const int oneWireBus = 5;       // data wire connected to GPIO 4 
const int sda1 = 2;
const int scl1 = 15;

// MQTT Broker

const char  *sensor_topic = "sensors/esp32";
#define inTopic "sensors/esp32/read"

const char *mqtt_broker = "10.4.0.106";   // Local Broker on Pi2
const char *room_topic = "home/DG/elab";
const char *mqtt_username = "esp32";
const char *mqtt_password = "esp32";
//const char *sensor_typ ="sensors/esp32";
const char *mqtt_mess = "";
const int mqtt_port = 1883;

//test using external MQTT broker
//const char *mqtt_broker = "broker.emqx.io";

// ANSI ESCAPE Sequences for coloured output
#define ESC_RED "\e[1;31m"      // red 
#define ESC_GRN "\e[1;32m"      // green
#define ESC_YEL "\e[1;33m"      // yellow
#define ESC_BLU "\e[1;34m"      // blue
#define ESC_MAG "\e[1;35m"      // magenta
#define ESC_CYA "\e[1;36m"      // cyan
#define ESC_WHT "\e[1;37m"      // white
#define ESC_RES "\e[1;0m"       // reset

OneWire OW_ds(oneWireBus);          // OneWire Datastructure
DallasTemperature ow_sensors(&OW_ds);  // Dallas Sensor Data
float ow_temp1;
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

WiFiClient espClient;  
PubSubClient client(espClient); 

// JSON Document
//StaticJsonDocument<300> odoc;  // 200 Byte RAM allocation
DynamicJsonDocument idoc(300);
DynamicJsonDocument odoc(300);
int buttonState = 0;
int lastButtonState = 0;

// non blocking Timer - millis() function returns unsigned long
unsigned long start_time;     // settings for non blocking timer
unsigned long timer1 = 30000; // time in ms
unsigned long current_time;   // millis() 

////////////////////////////  Functions  ///////////////////////////////
int status (String message)   // collecting status information
{
return 0;
}
////////////////////////// MQTT-callback ///////////////////////////////

void callback(char *topic, byte *payload, unsigned int length) {
  StaticJsonDocument<400> odoc; 
  String messageTemp;
  String output;
  char *x;
  // digitalWrite(LED, HIGH);    // LED blink for received MQTT
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
      messageTemp += (char)payload[i];
  }
  if (String(topic) == "sensors/esp32") 
    {
    if(messageTemp == "status")
      { 
        odoc["event"] = "STATUS";
        odoc["time"] = timeClient.getFormattedTime();
        odoc["sensor"] = sensor_topic;
        odoc["room"] = room_topic;
        odoc["sensor_ID"] = chipId; // cclient_Id.c_str();
        odoc["sensor_MAC"] = client_MAC.c_str();
        odoc["temp[0]"] = ow_temp1 = ow_sensors.getTempCByIndex(0);
        odoc["LED"] = digitalRead(LED);
    
        serializeJsonPretty(odoc, output);
        client.publish("sensors/esp32", output.c_str());
      } // if sensors/esp32
    
      if(messageTemp == "set_LED=1") 
      {
        digitalWrite(LED, HIGH);
        if(digitalRead(LED))
        client.publish("sensors/esp32","LED = HIGH");
      }
      else if(messageTemp == "set_LED=0")
      { 
        digitalWrite(LED, LOW);
        if(! digitalRead(LED))
        client.publish("sensors/esp32","LED = LOW");
      }
    } // if sensors/esp32
  Serial.println();
  // Serial.println(messageTemp);
  Serial.println("---------------------------------------");
  delay(10);     
} // callback

///////////////////////////   S E T U P   //////////////////////////////
void setup()
{
    //const int LED2 = 2; 
    pinMode(LED, OUTPUT);      // set the LED pin mode
    pinMode(sda1, INPUT_PULLUP);
    pinMode(scl1, INPUT_PULLUP);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //Brown out detection
    // establish serial connection (via USB Bridge)
    Serial.begin(115200);
    digitalWrite(LED, HIGH);   // Test connected LED
    delay(300);
    digitalWrite(LED, LOW);
    // retrieve chip-data
    Serial.println("");
    Serial.print(ESC_YEL);
    Serial.print("ESP MAC Address:");
    Serial.println(WiFi.macAddress());
    Serial.print(ESC_RES);
    
    // uint32_t chipId = 0;
    for(int i=0; i<17; i=i+8) 
      {
	      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	    }
    Serial.print(ESC_RES);
	  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
	  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
    Serial.print("Chip ID: "); Serial.println(chipId);
  
// Searching for One-Wire Sensors connected at GPIO "oneWireBUS" 
// Setup a oneWire instance to communicate with any OneWire devices
    // OneWire OW_ds(oneWireBus);
    Serial.print(ESC_MAG);
    Serial.print("\nScanning for OneWire devices on GPIO ");
    Serial.println(oneWireBus);

    byte addr[8]; byte i; int k=0;   // ROM address , index , number of devices
    while (OW_ds.search(addr)) 
    {
    Serial.print("ID =");           // e.g. ROM = 28 ED EE 8E 54 22 8 BE
    for (i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    } // for
    k++;                              // number of devices
  Serial.println();  
  } // while
  OW_ds.reset_search();
  Serial.print(ESC_RES);

// Pass oneWire reference to Dallas Temperature sensor (DS18B20)
  // DallasTemperature sensors(&OW_ds);  // Dallas Sensor Data
  for (i = 0; i < k; i++)             // read sensors
  {
    ow_sensors.begin();
    ow_sensors.requestTemperatures(); 
    ow_temp1 = ow_sensors.getTempCByIndex(i);  // first sensor
    
    if (ow_temp1 != -127.00)           // read error (R pullup ?)
    {
      Serial.print("Temp. Sensor(");
      Serial.print(i); Serial.print(")    : ");
      Serial.print(ow_temp1);
      Serial.println(" C");
    }
    else 
    { 
      Serial.print(ESC_RED);
      Serial.print("error reading Temp. Sensor(");
      Serial.print(i); Serial.print(") at GPIO ");
      Serial.println(oneWireBus);
      Serial.print(ESC_RES);
    }
  } // for

// Searching for i2c Devices 
  byte error, address;
  int nDevices;
  Wire.begin (sda1, scl1);   // sda= GPIO  /scl= GPIO 
  Serial.print(ESC_YEL);
  Serial.print("\nScanning for i2c Devices (sda=");
  Serial.print(sda1); Serial.print(" , scl = ");
  Serial.print(scl1); Serial.println(")");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address     0x");
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else Serial.println("done");
    Serial.print(ESC_RES);

    // Set WiFi to station mode (and disconnect from an AP)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    Serial.print("\nWIFI Staion Mode. ");
    Serial.print("Scanning for WiFi Networks ... ");
    // WiFi.scanNetworks will return the number of networks found.
    int n = WiFi.scanNetworks();
    Serial.println("done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(ESC_CYA);
        Serial.print(n);
        Serial.println(" networks found");
        Serial.println("Nr | SSID                       | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) 
        {
            // Print SSID and RSSI for each network found
            digitalWrite(LED, HIGH);           // LED blink for each network found
            Serial.printf("%2d",i + 1);
            Serial.print(" | ");
            Serial.printf("%-26.26s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4d", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2d", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i))
            {
            case WIFI_AUTH_OPEN: Serial.print("open"); break;
            case WIFI_AUTH_WEP:   Serial.print("WEP"); break;
            case WIFI_AUTH_WPA_PSK: Serial.print("WPA"); break;
            case WIFI_AUTH_WPA2_PSK: Serial.print("WPA2"); break;
            case WIFI_AUTH_WPA_WPA2_PSK: Serial.print("WPA+WPA2"); break;
            case WIFI_AUTH_WPA2_ENTERPRISE: Serial.print("WPA2-EAP"); break;
            case WIFI_AUTH_WPA3_PSK: Serial.print("WPA3"); break;
            case WIFI_AUTH_WPA2_WPA3_PSK: Serial.print("WPA2+WPA3"); break;
            case WIFI_AUTH_WAPI_PSK: Serial.print("WAPI"); break;
            default:
                Serial.print("unknown");
            }
            Serial.println();
            delay(100);
            digitalWrite(LED, LOW);
            delay(200);
        }
        Serial.print(ESC_RES);
    }
    
    // Delete the scan result to free memory for code below.
    WiFi.scanDelete();

    ///////////////////// Connecting to WiFi ///////////////////////////

    // Connecting to a WiFi network using Watchdog -Timer for Timeout!

    //Serial.print("Configuring Timeout:  "); Serial.println(WDT_TIMEOUT);
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               //add current thread to WDT watch
    esp_task_wdt_reset();                 //rest Watchdog Timer

    Serial.print(ESC_YEL);
    Serial.print("\nConnecting to:        ");
    Serial.println(wifi_ssid); 
    // WiFi.setHostname(hostname.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    esp_task_wdt_delete(NULL);            // delete WDT when connected!
    Serial.print("\nRRSI:                 ");
    Serial.println(WiFi.RSSI());
    // assign DNS-Server to IP 8.8.8.8
    WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), IPAddress(8,8,8,8));
    Serial.print("ESP MAC Address:      ");
    Serial.println(WiFi.macAddress());
    Serial.print("IP address:           ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway:              ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("DNS Server:           ");
    Serial.println(WiFi.dnsIP());
    Serial.print("HostName:             ");
    Serial.println(WiFi.getHostname());
    Serial.print(ESC_RES);

    //////////////////////  setup NTP Client   ////////////////////////
    // Initialize a NTP Client to get time
    Serial.print(ESC_YEL);
    Serial.print("\nStarting (NTP) Network Time Protocoll-Client ");
    timeClient.begin(); 
    timeClient.setTimeOffset(3600);      // GMT +1 = 3600 = CET (Vienna)
    timeClient.begin();
    timeClient.update();
    Serial.println(timeClient.getFormattedTime());
    Serial.print(ESC_RES);
    delay(10);

  ////////////// Check the Internet Connection ///////////////////
  // Check the Internet Connection and LAN Status
  if(Ping.ping("8.8.8.8", 3))
  {
    Serial.print(ESC_GRN);
    Serial.println("\nPing 8.8.8.8 successful. (Internet access)");
    Serial.print(ESC_RES);
  }
  else
  {
    Serial.print(ESC_RED);
    Serial.println("Ping 8.8.8.8 failed! (Internet access)");
    Serial.print(ESC_RES);
    // return;
  }
    // externer mqtt_broker2
    if(Ping.ping(mqtt_broker2, 3))
  {
    Serial.print(ESC_GRN);
    Serial.print("Ping ");
    Serial.print(mqtt_broker2); 
    Serial.println(" external MQTT-Broker successful.");
    Serial.print(ESC_RES);
  }
  else
  {
    Serial.print(ESC_RED);
    Serial.print("Ping ");
    Serial.print(mqtt_broker2); 
    Serial.println(" external MQTT-Broker failed!");
    Serial.print(ESC_RES);
    // return;
  }
  if(Ping.ping("www.google.com", 3))
    {
    Serial.print(ESC_GRN);    
    Serial.println("Ping \"www.google.com\" successful. (DNS)");
    Serial.print(ESC_RES);
    }
  else
  {
    Serial.print(ESC_RED);
    Serial.println("Ping \"www.google.com\" failed!  (DNS)");
    Serial.print(ESC_RES);
    // return;
  }
  
  //////////////////////  setup MQTT Client   ////////////////////////

  if(Ping.ping(mqtt_broker, 3))
  {
    Serial.print(ESC_GRN);
    Serial.print("Ping to MQTT-Broker ");
    Serial.print(mqtt_broker);
    Serial.println(" successful!");
    Serial.print(ESC_RES);
  }
  else
  {
    Serial.print(ESC_RED);
    Serial.print("Ping MQTT-Broker ");
    Serial.print(mqtt_broker);
    Serial.println(" failed!");
    Serial.print(ESC_RES);
    // return;
  }
  ////////////////  connecting to a MQTT Broker  /////////////////////
 
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (! client.connected()) {
      // clientId = "ESP32-";
      client_MAC += String(WiFi.macAddress());
      if (client.connect(client_MAC.c_str(), mqtt_username, mqtt_password))
      {
        Serial.print(ESC_MAG);
        Serial.printf("MQTT-client: %s connected\n", client_MAC.c_str());
        Serial.print(ESC_RES);
      } 
      else 
      {
        Serial.print(ESC_RED);
        Serial.print("failed with state ");
        Serial.print(client.state());
        Serial.print(ESC_RES);
        delay(2000);
      }
  }
  // publish and subscribe
// char* str_chipId = itoa(chipId,str_chipId,10);
//Serial.print(str_chipId);
  
  // sensor_topic = strcat(sensor_topic,str_chipId);
  Serial.println(sensor_topic);
  
  client.subscribe("sensors/esp32/#");       // default topic
  if(! client.subscribe(room_topic))
    { 
      Serial.print(ESC_RED);
      Serial.print(sensor_topic);
      Serial.println(" subscribe failed!");
      Serial.print(ESC_RES);
    } 
    else
    {
      Serial.print(ESC_GRN);
      Serial.print(sensor_topic);
      Serial.println(" subscribed successfully!");
      Serial.print(ESC_RES);      
    }
    if(! client.subscribe(room_topic))
    { 
      Serial.print(ESC_RED);
      Serial.print(room_topic);
      Serial.println(" subscribe failed!");
      Serial.print(ESC_RES);
    } 
    else
    {
      Serial.print(ESC_GRN);
      Serial.print(room_topic);
      Serial.println(" subscribed successfully!");
      Serial.print(ESC_RES);      
    }

  StaticJsonDocument<400> odoc;  
  String output;
  odoc["event"] = "BOOT";
  odoc["time"] = timeClient.getFormattedTime();
  odoc["sensor"] = sensor_topic;
  odoc["room"] = room_topic;
  odoc["sensor_ID"] = chipId; // cclient_Id.c_str();
  odoc["sensor_MAC"] = client_MAC.c_str();
  // odoc["version_SW"] = Version_SW;
  // odoc["version_HW"] = Version_HW;
  odoc["temp[0]"] = ow_sensors.getTempCByIndex(0);
  odoc["LED"] = digitalRead(LED);
  /* odoc["temp[1]"] = sensors.getTempCByIndex(1);
  odoc["SSID"] = WiFi.SSID();
  odoc["RSSI"] = WiFi.RSSI();
  odoc["channel"] = WiFi.channel();
  odoc["RSSI"] = WiFi.RSSI(); */

  serializeJsonPretty(odoc, Serial);   // generat & print JSON to Serial
  Serial.println("");
  
  serializeJsonPretty(odoc, output);

  if (! client.publish("sensors/esp32",output.c_str()));
    { 
      Serial.print(ESC_RED);
      Serial.println("publish failed!");
      Serial.print(ESC_RES);
    } 

  client.publish("sensors/esp32", output.c_str()); // publish to sensor_topic
odoc.clear();
// Start Timer
  current_time = millis();
	start_time = current_time; 
}  // end setup

//////////////////////////  M A I N - L O O P ////////////////////////                      
void loop() 
{
  client.loop();
  if (millis() - start_time >= timer1) //repeating after timer (ms)
  {
    String output;
    StaticJsonDocument<200> odoc;
    ow_temp1 = ow_sensors.getTempCByIndex(0);
    odoc["event"] = "LOOP";
    odoc["sensor_ID"] = chipId;
    odoc["time"] = timeClient.getFormattedTime();
    odoc["temp[0]"] = ow_temp1;       //ow_sensors.getTempCByIndex(0);
    odoc["SSID"] = WiFi.SSID();
    odoc["RSSI"] = WiFi.RSSI();
    odoc["LED"] = digitalRead(LED);

    serializeJsonPretty(odoc, output);
    client.publish(room_topic, output.c_str());
    start_time = millis();            // reset the timer
    odoc.clear();
  }
}  // end loop
//////////////////////////////  E N D  ///////////////////////////////