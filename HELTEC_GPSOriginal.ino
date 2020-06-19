#include <WiFi.h>   // ESP8266 WiFi Libary
#include <PubSubClient.h>  // MQTT Library
#include "DHT.h"           // Digital Humidity and TemperatureLibary
#include "SSD1306Wire.h"   // OLED Library
#include <TinyGPS++.h>     // GPS Library

// ==== WIFI CONFIGURATION ====
#define WIFI_SSID     "Team6"
#define WIFI_PASSWORD "passw0rd!"
WiFiClient wifiClient;

// ==== MQTT BROKER CONFIGURATION ====
#define CLIENT_ID  "ESP32Team6"
#define MQTT_SERVER "129.213.166.182"
#define MQTT_PORT    1883
PubSubClient mqttClient(wifiClient);

// ==== DHT CONFIGURATION ====
#define DHT_PIN 5
DHT dht(DHT_PIN, DHT11);

// ==== OLED DISPLAY CONFIGURATION ====
#define OLED_SDA 4
#define OLED_SCL 15
SSD1306Wire  display(0x3c, OLED_SDA, OLED_SCL);

// ==== TINY GPS ++ CONFIGURATION ====
HardwareSerial SerialGPS(2);
#define RXD2 13 //16//Gpio pins Serial2
#define TXD2 -1 //17
TinyGPSPlus gps;

// ==== LED CONFIGURATION ====
const int ledPin = LED_BUILTIN; //Blue LED

// Global variables
String greeting = "";
String myIpAddress = "NA";
long lastTime = 0;

String latString;
String lngString;
String courseString;
String mpsString;
String mphString;
String cardinalString;
String distanceString;
String courseToString;
int lastButtonPress = HIGH;

// Destination cooridnates
// Oracle Conference Center: 37.532340, -122.264024
double destinationLat = 37.532340;
double destinationLng = -122.264024;
String Firefighter = "Unit1";

boolean wifiEnabled = false;

// Subscribe to a MQTT topic. Add MQTT topics here.
void mqttSubscribe() {
  mqttClient.subscribe("room/lamp");

}

// Receive incoming MQTT topic and message
void mqttMessageReceived(String topic, String message) {
  if (topic == "greeting") {
    greeting = message;
  }
  else if (topic == "room/lamp") {
    if (message == "on") {
      digitalWrite(ledPin, LOW);
      Serial.println("LED on");
    }
    else if (message == "off") {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED off");
    }
  }
}

void setup() {
  //SERIAL MONITOR SETUP
  Serial.begin(115200);
  Serial.println("Serial monitor test");

  // Setup GPS Serial
  SerialGPS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);

  // LED SETUP
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); //(HIGH = off, LOW = on)
  // DHT SENSOR SETUP
  dht.begin();

  // OLED DISPLAY SETUP
  displaySetup();

  // WiFi SETUP
  wifiConnect();
  // MQTT SETUP
  mqttConnect();
}
bool ShrekButton = false;
void loop() {

  int button = 0;
  button = digitalRead(17);
  if (button == LOW && lastButtonPress == HIGH) {
    dualPrint("help!");
    ShrekButton = true;
    mqttPublish("shrek/button", ShrekButton);
  }   else {
    ShrekButton = false;
  }
  lastButtonPress = button;

  //Check MQTT and WiFi Connection
  mqttTestConnection();
  while (SerialGPS.available() > 0) {
    byte gpsData = SerialGPS.read();
    //Serial.write(gpsData); //Print debug
    gps.encode(gpsData);

    if (gps.location.isUpdated()) {
      // Get GPS coords as strings rounded to 6 decimals
      latString = String(gps.location.lat(), 6);
      lngString = String(gps.location.lng(), 6);
      courseString = String(gps.course.deg(), 2);
      mpsString = String(gps.speed.mps(), 2);
      mphString = String(gps.speed.mph(), 1);
      cardinalString = TinyGPSPlus::cardinal(gps.course.deg());

      calcDistanceCourse();

      display.clear();
      if (!wifiEnabled) {
        display.drawString(115, 0, "!!!");
      }
      display.drawString(0, 0, "Current Speed");
      display.drawString(0, 16,  mphString + "mph");
      display.drawString(75, 16, cardinalString);
      display.drawString(0, 32, "Destination");
      display.drawString(0, 48, distanceString);
      display.drawString(75, 48, courseToString);
      display.display();
      break;
    }
  }
  // Non-blocking timer used instead of delay. delay blocks all code from running.
  if (millis() - lastTime > 5000) {
    lastTime = millis();
    publishGPS();
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float humidity = dht.readHumidity();
    //    // Read temperature as Fahrenheit (true = Fahrenheit, false = Celcius)
    float fahrenheit = dht.readTemperature(true);
    //
    //    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(fahrenheit)) {
      Serial.println("Failed to read from DHT sensor!");
    }

    //    // Print humidity and temp in Serial Monitor and OLED display
    dualPrint("Humidity: ");
    dualPrint(64,  0, humidity);
    dualPrint( 0, 10, "Temp(*F): ");
    dualPrint(64, 10, fahrenheit);
    //
    //    // Print greeting
    //    dualPrint(0, 30, greeting);
    //
    dualPrint( 0, 50, "MY IP: ");
    dualPrint(30, 50, myIpAddress);
    //
    //    // Publishes Temperature and Humidity values
    mqttPublish("shrek/humidity", humidity);
    mqttPublish("shrek/temperature", fahrenheit);
  }
}


void calcDistanceCourse() {
  double metersToDestination  =
    TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      destinationLat,
      destinationLng);
  double milesToDestination = metersToDestination / 1609.344;
  if (milesToDestination < .1) {
    distanceString = String(milesToDestination * 5280.0, 0) + " ft";
  }
  else {
    distanceString = String(milesToDestination, 1) + " mi";
  }

  double courseToDestination =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      destinationLat,
      destinationLng);
  courseToString = TinyGPSPlus::cardinal(courseToDestination);
}

//Publish MQTT GPS messages
void publishGPS() {
  if (mqttClient.connected()) {
    String json = "{\"lat\":" + latString + ", \"lon\":" + lngString + ", \"name\":\"" + Firefighter + "\"}";
    //String s = latString + "," + lngString;
    String s = json;
    char tempString[s.length() + 1];
    s.toCharArray(tempString, s.length() + 1);
    mqttClient.publish("shrek/GPS", tempString);
  }
}

// ==== DISPLAY HELPER ======
void displaySetup() {

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high

  display.init();
  display.clear();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  dualPrint(0, 0, "SSD1306 Working");
  display.display();
  delay(1000);
}

// ==== MQTT HELPER FUNCTIONS ====

void mqttPublish(String topic, String message) {
  Serial.println("mqttPublish: " + topic + " " + message);
  mqttClient.publish((char*)topic.c_str(), (char*)message.c_str());
}

//helper to convert number values to string
void mqttPublish(String topic, float f) {
  mqttPublish(topic, String(f));
}

void mqttTestConnection() {
  // MQTT Connection Test
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  // WiFi Connection Test
  if (!mqttClient.loop())
    mqttClient.connect(CLIENT_ID);
}

// This functions mqttConnects your ESP8266 to your MQTT broker
void mqttConnect() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Display MQTT Configuration
  dualPrint("Connecting to MQTT");
  dualPrint( 0, 10, "Server:");
  dualPrint(40, 10, MQTT_SERVER);
  dualPrint( 0, 20, "Port:");
  dualPrint(40, 20, String(MQTT_PORT));

  // Loop until we're mqttConnected
  while (!mqttClient.connected()) {
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID)) {

      dualPrint(0, 50, "MQTT connected");
      mqttSubscribe();
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
      dualPrint(0, 30, "Check firewall settings.");
      delay(5000);
    }
  }
}

// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
void mqttCallback(String topic, byte * message, unsigned int length) {
  Serial.print("mqttMessage topic: ");
  Serial.print(topic);
  Serial.print("  payload: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  mqttMessageReceived(topic, messageTemp);
}


// === WiFi Connection Helper Function ====
void wifiConnect() {
  delay(10);

  // Display WiFi connection configuration
  dualPrint("Connecting to WiFi");
  dualPrint(0,  10, "SSID: ");
  dualPrint(32, 10, WIFI_SSID);

  // Attempt WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  // WiFi connection successfull
  dualPrint(0, 20, "WiFi connected");

  // Save and display ESP8266 IP Address
  myIpAddress = WiFi.localIP().toString();
  dualPrint( 0, 50, "MY IP: ");
  dualPrint(30, 50, myIpAddress);

  delay(5000);
}

// === OLED DISPLAY HELPER FUNCTIONS ====
// [dualPrint] is a helper function that will print
// text in both the Serial Monitor & the OLED Display
void dualPrint(String text) {
  display.clear();
  Serial.println(text);
  display.drawString(0, 0, text);
  display.display();
}

void dualPrint(int x, int y, String text) {
  Serial.println(text);
  display.drawString(x, y, text);
  display.display();
}

// dualPrint function used for numbers
void dualPrint(int x, int y, float f) {
  dualPrint(x, y, String(f));
}

// dualPrint function used for numbers
void dualPrint(float f) {
  dualPrint(String(f));
}
