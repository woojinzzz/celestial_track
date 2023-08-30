
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

#define dirPin 2
#define stepPin 3
#define dir2Pin 4
#define step2Pin 5
#define stepsPerRevolution 200

int i=0.;

const int BAUD_RATE = 57600;  // 시리얼 통신 속도

// ROS 관련 객체
ros::NodeHandle nh;
std_msgs::Int32MultiArray coordinates_msg;

void coordinatesCallback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length >= 2) {
    int x = msg.data[0];
    int y = msg.data[1];
    
    // 중앙 좌표값 출력
    Serial.print("Center X: ");
    Serial.println(x);
    Serial.print("Center Y: ");
    Serial.println(y);

    // if (x >= 640) {
    //   Serial.print("motor go left\n");
    // }

    // if (x < 640) {
    //   Serial.print("motor go right\n");
    // }
/*      
      while (1)
      { 
        Serial.print("motor go down\n");
        digitalWrite(dirPin,HIGH);

        digitalWrite(stepPin,HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(2000);
        
        if (y>=300)
          break;
      }
*/

    if (x <= 500) {
    digitalWrite(dir2Pin, HIGH);

    if (i > 0 && i < stepsPerRevolution) {
        digitalWrite(step2Pin, HIGH);
        delayMicroseconds(1500);
        digitalWrite(step2Pin, LOW);
        delayMicroseconds(1500);
        i++; // i 변수를 증가시켜 다음 반복을 위한 준비
    }
}

    // if (x<=500){
    //   digitalWrite(dir2Pin,HIGH);
    //   Serial.print("motor go left\n");
    //   for(int i=0;i<stepsPerRevolution;i++)
    //     {
    //   digitalWrite(step2Pin,HIGH);
    //   delayMicroseconds(1500);
    //   digitalWrite(step2Pin,LOW);
    //   delayMicroseconds(1500);
    //     }
    // }
      
    else if(x>700){
    digitalWrite(dir2Pin,LOW);
    for(int i=0;i<stepsPerRevolution;i++)
      {
    Serial.print("motor go right\n");
    digitalWrite(step2Pin,HIGH);
    delayMicroseconds(1500);
    digitalWrite(step2Pin,LOW);
    delayMicroseconds(1500);
      }
    }

    else if (y<=300){
      digitalWrite(dirPin,HIGH);
      for(int i=0;i<stepsPerRevolution;i++)
        {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(1500);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(1500);
        }
      }
      
    else if (y>380){
    digitalWrite(dirPin,LOW);
    for(int i=0;i<stepsPerRevolution;i++)
      {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(1500);
      }
    }
  }
}      
      
/*        for(int i=0;i<stepsPerRevolution/12;i++)
        {
        
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(1000);
        }

      }

    else if (y > 380) {
      Serial.print("motor go up\n");
        digitalWrite(dirPin,LOW);

        digitalWrite(stepPin,HIGH);

        for(int i=0;i<stepsPerRevolution/12;i++)
        {
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(1000);
        }

      }
    } 
  }
*/
ros::Subscriber<std_msgs::Int32MultiArray> coordinates_sub("bounding_box_coordinates", coordinatesCallback);


void setup() {
  // 시리얼 통신 시작
  Serial.begin(BAUD_RATE);

  // 핀 선언 (output)
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(step2Pin,OUTPUT);
  pinMode(dir2Pin,OUTPUT);

  // ROS 노드 초기화
  nh.initNode();
  nh.subscribe(coordinates_sub);
}

void loop() {
  // ROS 노드 업데이트
  nh.spinOnce();

  // digitalWrite(dirPin,HIGH);
  //     for(int i=0;i<stepsPerRevolution;i++)
  //       {
  //     digitalWrite(stepPin,HIGH);
  //     delayMicroseconds(1500);
  //     digitalWrite(stepPin,LOW);
  //     delayMicroseconds(1500);
  //       }

  // digitalWrite(dir2Pin,HIGH);
  //     for(int i=0;i<stepsPerRevolution;i++)
  //       {
  //     digitalWrite(step2Pin,HIGH);
  //     delayMicroseconds(1500);
  //     digitalWrite(step2Pin,LOW);
  //     delayMicroseconds(1500);
  //       }

  //dir : CW

  //spin 1 rev



}
