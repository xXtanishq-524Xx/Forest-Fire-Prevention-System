MQ-5 Sensor:
const int mqPin = A0;  // Analog pin connected to the MQ-5 sensor
const int ledPin = D1; // Digital pin connected to an LED

void setup() {
  pinMode(mqPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // Read the analog value from the MQ-5 sensor
  int sensorValue = analogRead(mqPin);

  // Print the sensor value to the serial monitor
  Serial.print("MQ-5 Sensor Value: ");
  Serial.println(sensorValue);

  // Adjust the threshold based on your sensor and environment
  int threshold = 200;

  // Check if the sensor value is above the threshold
  if (sensorValue > threshold) {
    // Gas detected, turn on the LED or take appropriate action
    digitalWrite(ledPin, HIGH);
    Serial.println("Gas Detected!");
  } else {
    // No gas detected, turn off the LED or take appropriate action
    digitalWrite(ledPin, LOW);
    Serial.println("No Gas Detected");
  }

  delay(1000);  // Adjust delay based on your needs
}
