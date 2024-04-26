// Define the pins for the ultrasonic sensor
const int trigPin = 4;  // Trigger pin
const int echoPin = 5; // Echo pin


void setup() {
  Serial.begin(9600);   // Initialize serial communication
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


void loop() {
  // Trigger the ultrasonic sensor by sending a 10us pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);


  // Measure the duration of the pulse on the echo pin
  unsigned long duration = pulseIn(echoPin, HIGH);


  // Calculate the distance based on the speed of sound (343 m/s)
  // and the time it took for the pulse to return
  float distance = (duration * 0.0343) / 2;


  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");


  // Wait for a short time before taking the next measurement
  delay(500);
}
