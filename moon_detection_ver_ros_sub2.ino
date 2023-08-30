#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#define dirPin 2
#define stepPin 3
#define dir2Pin 4
#define step2Pin 5
#define stepsPerRevolution 200

int i = 0;

const int BAUD_RATE = 57600;  // Serial communication speed

// ROS-related objects
ros::NodeHandle nh;
std_msgs::Int32MultiArray coordinates_msg;

// Callback function for receiving bounding box coordinates from ROS
void coordinatesCallback(const std_msgs::Int32MultiArray &msg) {
  if (msg.data_length >= 2) {
    int x = msg.data[0];
    int y = msg.data[1];
    
    // Print center coordinates
    Serial.print("Center X: ");
    Serial.println(x);
    Serial.print("Center Y: ");
    Serial.println(y);

    // Motor control based on X and Y coordinates
    if (x <= 500) {
      // Rotate motor in one direction
      digitalWrite(dir2Pin, HIGH);

      if (i > 0 && i < stepsPerRevolution) {
        // Step motor
        digitalWrite(step2Pin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(step2Pin, LOW);
        delayMicroseconds(1500);
        i++; // Increment i for the next step
      }
    } else if (x > 700) {
      // Rotate motor in the other direction
      digitalWrite(dir2Pin, LOW);

      // Step motor for a full revolution
      for (int i = 0; i < stepsPerRevolution; i++) {
        Serial.print("motor go right\n");
        digitalWrite(step2Pin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(step2Pin, LOW);
        delayMicroseconds(1500);
      }
    } else if (y <= 300) {
      // Move motor up
      digitalWrite(dirPin, HIGH);
      for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1500);
      }
    } else if (y > 380) {
      // Move motor down
      digitalWrite(dirPin, LOW);
      for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1500);
      }
    }
  }
} 

// ROS Subscriber for the bounding_box_coordinates topic
ros::Subscriber<std_msgs::Int32MultiArray> coordinates_sub("bounding_box_coordinates", coordinatesCallback);

void setup() {
  // Start serial communication
  Serial.begin(BAUD_RATE);

  // Declare pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(step2Pin, OUTPUT);
  pinMode(dir2Pin, OUTPUT);

  // Initialize ROS node
  nh.initNode();
  nh.subscribe(coordinates_sub);
}

void loop() {
  // Update ROS node
  nh.spinOnce();
}
