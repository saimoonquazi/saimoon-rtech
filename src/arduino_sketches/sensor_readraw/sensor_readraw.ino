// Ultrasonic sensor pins

#include <ros.h>
//#include <std_msgs/String.h>
#include "sensor_msgs/Range.h"

ros::NodeHandle  nh;

//std_msgs::String str_msg;
sensor_msgs::Range sensor;
ros::Publisher us_data("us_data_raw", &sensor);

int echoPin = A4;
int trigPin = A5;

void setup()
{
  nh.initNode();
  nh.advertise(us_data);
  Serial.begin(57600);
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  sensor.min_range=0.02;
  sensor.max_range=4;
  sensor.field_of_view=0.261799;
  sensor.header.frame_id="range_test";
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
  float us = (getSonarReadingMillimeters()); //Get distance from wall with ultrasonic sensor
  float sensor_distance=us/1000;
  //Serial.println(sensor_distance);
  sensor.range=sensor_distance;
  us_data.publish( &sensor );
  nh.spinOnce();
  delay(100);
}
