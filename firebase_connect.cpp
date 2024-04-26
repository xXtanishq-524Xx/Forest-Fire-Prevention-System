#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <DHT.h> // Include the DHT library

// Include the TinyGPS++ library for parsing GPS data
#include <TinyGPS++.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "WiFi name"
#define WIFI_PASSWORD "WiFi password"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBkVXgv9YoxYWXiE-5zUHESo0HoTY69684"

// Insert RTDB URL
#define DATABASE_URL "https://datacommunication-14ec7-default-rtdb.asia-southeast1.firebasedatabase.app/"

#define LED1_PIN 3
#define LED2_PIN 4

const int freq = 5000;

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Define DHT sensor pin
#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

unsigned long sendDataPrevMillis = 0;
int ldrdata = 0;
float voltage = 0.0;
int pwmValue = 0;
boolean ledStatus = false;

// Create a SoftwareSerial object for communication with the GPS module
const int RXPin = 10; // Connect the GPS TX pin to this pin
const int TXPin = 11; // Connect the GPS RX pin to this pin
TinyGPSPlus gps; // Create a TinyGPS++ object

void setup() {
  pinMode(LED2_PIN, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(freq);
  analogWrite(LED1_PIN, 0);
  
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase Connected!");
  } else {
    Serial.println("Firebase Connection Failed:");
    Serial.println(config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Initialize DHT sensor
  dht.begin();

  // Initialize the SoftwareSerial object for communication with the GPS module
  Serial1.begin(9600, SWSERIAL_8N1, RXPin, TXPin);
}

void loop() {
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Read humidity and temperature from DHT sensor
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // Read latitude and longitude from GPS module
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Send latitude and longitude to Firebase
    if (Firebase.RTDB.setFloat(&fbdo, "location/latitude", latitude) &&
        Firebase.RTDB.setFloat(&fbdo, "location/longitude", longitude)) {
      Serial.println("Location data sent to Firebase!");
    } else {
      Serial.println("Failed to send location data to Firebase!");
      Serial.println(fbdo.errorReason());
    }

    // Send humidity and temperature to Firebase
    if (Firebase.RTDB.setFloat(&fbdo, "environment/humidity", humidity) &&
        Firebase.RTDB.setFloat(&fbdo, "environment/temperature", temperature)) {
      Serial.println("Humidity and temperature data sent to Firebase!");
    } else {
      Serial.println("Failed to send humidity and temperature data to Firebase!");
      Serial.println(fbdo.errorReason());
    }

    // Example: Reading data from Firebase
    if (Firebase.RTDB.getInt(&fbdo, "/LED/analog")) {
      if (fbdo.dataType() == "int") {
        pwmValue = fbdo.intData();
        Serial.print("Successful Read from ");
        Serial.print(fbdo.dataPath());
        Serial.print(": ");
        Serial.print(pwmValue);
        Serial.print(" (");
        Serial.print(fbdo.dataType());
        Serial.println(")");
        analogWrite(LED1_PIN, pwmValue);
      }
    } else {
      Serial.print("Failed: ");
      Serial.println(fbdo.errorReason());
    }

    if (Firebase.RTDB.getBool(&fbdo, "/LED/digital")) {
      if (fbdo.dataType() == "boolean") {
        ledStatus = fbdo.boolData();
        Serial.print("Successful Read from ");
        Serial.print(fbdo.dataPath());
        Serial.print(": ");
        Serial.print(ledStatus);
        Serial.print(" (");
        Serial.print(fbdo.dataType());
        Serial.println(")");
        digitalWrite(LED2_PIN, ledStatus);
      }
    } else {
      Serial.print("Failed: ");
      Serial.println(fbdo.errorReason());
    }
  }

  // Update GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      break;
    }
  }
}
