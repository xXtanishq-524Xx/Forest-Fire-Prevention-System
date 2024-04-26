Final Source Code:

// THIS EXAMPLE SHOWS HOW VVM501 ESP32 4G LTE MODULE CAN USE TO SEND AND RECEIVE SMS AND CALL
// FOR VVM501 PRODUCT DETAILS VISIT www.vv-mobility.com
#define RXD2 27    // VVM501 MODULE RXD INTERNALLY CONNECTED
#define TXD2 26    // VVM501 MODULE TXD INTERNALLY CONNECTED
#define powerPin 4 // VVM501 MODULE ESP32 PIN D4 CONNECTED TO POWER PIN OF A7670C CHIPSET, INTERNALLY CONNECTED
#define relay 2
#define SerialAT Serial1
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <DHT.h> // Include the DHT library
#include <Firebase_ESP_Client.h>

// Firebase configuration
#define FIREBASE_HOST "your_firebase_project_url"
#define FIREBASE_AUTH "your_firebase_auth_token"

// GPS RX to D1 & GPS TX to D2 and Serial Connection
const int RXPin = 4, TXPin = 5;
const uint32_t GPSBaud = 9600; 
SoftwareSerial gps_module(RXPin, TXPin);

TinyGPSPlus gps; 
#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

FirebaseData firebaseData;

int rx = -1;
String rxString;
int _timeout;
String _buffer;
String number = "+91XXXXXXXXXX"; // REPLACE WITH YOUR NUMBER
String SMS = "MESSAGE FROM VVM501 ESP32 4G LTE MODULE";

bool relayOn = false;

const int flameSensorPin = A0;  // Analog pin connected to the flame sensor
const int SmokeSensorPin = A0;  // Analog pin connected to the flame sensor
#define BuzzerPin 1 // Digital pin connected to Buzzer

unsigned long sendDataPrevMillis = 0;
int ldrdata = 0;
float voltage = 0.0;
int pwmValue = 0;
boolean ledStatus = false;

void setup() {
  InitUnit();
}

int result = 0;

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
  int flameSensorValue = analogRead(flameSensorPin);
  int SmokeSensorValue = analogRead(SmokeSensorPin);
  
  // Adjust the threshold based on your sensor and environment
  int flamethreshold = 500;
  int smokethreshold = 150;
  
  // Flame detection logic
  if (flameSensorValue < flamethreshold || SmokeSensorValue < smokethreshold) {
    // Flame or smoke detected, take action
    digitalWrite(BuzzerPin, HIGH);
    Serial.println("Fire Detected!");
    SMS = "Fire Detected!";
    result = SendMessage();
    
    // Read GPS data
    if (gps.encode(gps_module.read())) {
      displayGPSInfo();
      
      // Send GPS data to Firebase
      sendGPSDataToFirebase();
    }
  } else {
    // No flame or smoke detected
    digitalWrite(BuzzerPin , LOW);
  }

  int command = CheckForValidCallSMS();
  
  // Handle commands based on SMS or call
  if (command == 1) {
    // Handle valid call
    if (relayOn == false) {
      digitalWrite(relay, HIGH);
      relayOn = true;
      delay(1000);
    } else {
      digitalWrite(relay, LOW);
      relayOn = false;
      delay(5000);
    }
    CutTheCall();
    delay(5000);
    CallNumber();
    delay(15000);
    CutTheCall();
  } else if (command == 2) {
    // Handle invalid call
    CutTheCall();
  } else if (command == 3) {
    // Handle ON SMS command
    digitalWrite(relay, HIGH);
    relayOn = true;
    SMS = "UNIT TURNED ON SUCCESSFULLY...!";    
    result = SendMessage();
    while (result == 0) {
      SMS = "UNIT TURNED ON SUCCESSFULLY...!";    
      result = SendMessage();
    }
  } else if (command == 4) {
    // Handle OFF SMS command
    digitalWrite(relay, LOW);
    relayOn = false;
    SMS = "UNIT TURNED OFF SUCCESSFULLY...!";
    result = SendMessage();
    while (result == 0) {
      SMS = "UNIT TURNED OFF SUCCESSFULLY...!";    
      result = SendMessage();
    }
  } else if (command == 5) {
    // Handle STATUS SMS command
    if (relayOn == false)
      SMS = "UNIT IS OFF...";
    else
      SMS = "UNIT IS ON...";
    result = SendMessage();
    while (result == 0) {
      if (relayOn == false)
        SMS = "UNIT IS OFF...";
      else
        SMS = "UNIT IS ON...";  
      result = SendMessage();
    }
  } else if (command == 6) {
    // Handle invalid SMS keyword
    SMS = "INVALID KEYWORD...";
    result = SendMessage();
    while (result == 0) {
      SMS = "INVALID KEYWORD...";    
      result = SendMessage();
    }
  }
  delay(1000);
}

int CheckForValidCallSMS() {
  if (SerialAT.available() > 0) {
    rxString = "";
    rxString = SerialAT.readString();
    if (rxString != "")
      Serial.println(rxString);
    String expectedString = "+CLIP:";
    rx = rxString.indexOf(expectedString);
    if (rx != -1) {
      // Phone is ringing; Check for authorized number
      rx = rxString.indexOf(number);
      if (rx != -1) {
        // Authentic Call
        Serial.println("Authentic Call...");
        return 1;
      } else {
        Serial.println("Un-Authentic Call...");
        return 2;
      }
    } else {
      expectedString = "+CMT:";
      rx = rxString.indexOf(expectedString);
      if (rx != -1) {
        // SMS is received; Check for authorized number
        rx = rxString.indexOf(number);
        if (rx != -1) {
          // Authentic SMS
          Serial.println("SMS From Authentic Number...");
          // Check for ON / OFF Commands
          Serial.println(rxString);
          rx = rxString.indexOf("ON");
          if (rx != -1) {
            Serial.println("TURN ON");
            return 3; //ON COMMAND
          }            
          rx = rxString.indexOf("OFF");
          if (rx != -1) {
            Serial.println("TURN OFF");
            return 4; //OFF COMMAND
          }            
          rx = rxString.indexOf("STATUS");
          if (rx != -1) {
            Serial.println("GIVE STATUS");
            return 5; //STATUS COMMAND
          }
          Serial.println("INVALID KEYWORD");
          return 6; //INVALID KEYWORD
        } else {
          Serial.println("SMS From Un-Authentic Number...");
          return 0;
        }
      } else
        return 0;
    }      
  }  
}

int SendMessage() {  
  delay(5000);
  int res = 0;
  char end[2];
  end[0]=0x1a;
  end[1]='\0';
  SerialAT.println("AT+CMGS=?"); //9s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("OK");
  if (rx != -1) {
    Serial.println ("Sending Message");
    SerialAT.print("AT+CMGS=\"" + number + "\"\r"); //Mobile phone number to send message
    delay(1000);  
    Serial.println ("1");
    SerialAT.print(SMS);
    SerialAT.println(end);// ASCII code of CTRL+Z
    Serial.println ("2");
    _buffer = _readSerial();
    SerialAT.println("AT+CMGD=4");    //Delete all read messages
    Serial.println ("3");
    delay(2000);
    Serial.println ("Message Sent");
    res = 1;
  }  
  else
    Serial.println ("Can't Send Message");
  return res;
}

void sendGPSDataToFirebase() {
  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Create JSON object for GPS data
    String json = "{\"latitude\": " + String(latitude, 6) + ", \"longitude\": " + String(longitude, 6) + "}";
    
    // Push GPS data to Firebase
    if (Firebase.pushString(firebaseData, "/gps_data", json)) {
      Serial.println("GPS data sent to Firebase!");
    } else {
      Serial.println("Failed to send GPS data to Firebase!");
      Serial.println(firebaseData.errorReason());
    }
  }
}

String _readSerial() {
  _timeout = 0;
  while  (!SerialAT.available() && _timeout < 12000  ) {
    delay(13);
    _timeout++;
  }
  if (SerialAT.available()) {
    return SerialAT.readString();
  }
}

void CutTheCall() {
  Serial.println("CUT THE CALL...");
  SerialAT.print(F("AT+CHUP"));
  SerialAT.print(F(";\r\n"));
  _buffer = _readSerial();
  Serial.println(_buffer);
}

void CallNumber() {
  Serial.println("CALLING THE ADMIN...");
  SerialAT.print(F("ATD"));
  SerialAT.print(number);
  SerialAT.print(F(";\r\n"));
  _buffer = _readSerial();
  Serial.println(_buffer);
}


void InitUnit() {
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
  pinMode(LED2_PIN, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(freq);
  analogWrite(LED1_PIN, 0);

  Serial.begin(115200);
  delay(100);
  // Initialize DHT sensor
  dht.begin();

  // Initialize Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  
  SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(10000);
  Serial.println("Modem Reset, please wait");
  SerialAT.println("AT+CRESET");
  delay(1000);
  SerialAT.println("AT+CRESET");
  delay(20000);  // WAITING FOR SOME TIME TO CONFIGURE MODEM
  SerialAT.flush();
  Serial.println("Echo Off");
  SerialAT.println("ATE0");   //120s
  delay(1000);
  SerialAT.println("ATE0");   //120s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("OK");
  if (rx != -1)
    Serial.println("Modem Ready");
  delay(1000);
  Serial.println("SIM card check");
  SerialAT.println("AT+CPIN?"); //9s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("+CPIN: READY");
  if (rx != -1)
    Serial.println("SIM Card Ready");
  delay(1000);
  SerialAT.println("AT+CMGF=1");    // Sets the GSM Module in Text Mode
  delay(2000);
  SerialAT.println("AT+CMGD=4");    // Delete all read messages
  delay(2000);
  RecieveMessage(); // RECEIVE MESSAGE FROM THE MENTIONED PHONE NUMBER TO SIM
}

