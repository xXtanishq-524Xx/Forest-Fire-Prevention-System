const int flameSensorPin = A0;  // Analog pin connected to the flame sensor
const int ledPin = D1;          // Digital pin connected to an LED

void setup() {
  pinMode(flameSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // Read the analog value from the flame sensor
  int sensorValue = analogRead(flameSensorPin);

  // Print the sensor value to the serial monitor
  Serial.print("Flame Sensor Value: ");
  Serial.println(sensorValue);

  // Adjust the threshold based on your sensor and environment
  int threshold = 500;

  // Check if the sensor value is above the threshold
  if (sensorValue < threshold) {
    // Flame detected, turn on the LED or take appropriate action
    digitalWrite(ledPin, HIGH);
    Serial.println("Flame Detected!");
  } else {
    // No flame detected, turn off the LED or take appropriate action
    digitalWrite(ledPin, LOW);
    Serial.println("No Flame Detected");
  }

  delay(2000);  // Adjust delay based on your needs
}
