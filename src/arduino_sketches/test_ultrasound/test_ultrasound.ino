// Ultrasonic sensor pins
int echoPin = A4;
int trigPin = A5;

void setup()
{
  Serial.begin(9600);
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
}

long getSonarReadingMillimeters()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration_us = pulseIn(echoPin, HIGH);
  long distance_mm = (duration_us / 58.0) * 10;
  return distance_mm;
}

void loop()
{
  long us = getSonarReadingMillimeters(); //Get distance from wall with ultrasonic sensor
  Serial.println(us);
  delay(20);
}
