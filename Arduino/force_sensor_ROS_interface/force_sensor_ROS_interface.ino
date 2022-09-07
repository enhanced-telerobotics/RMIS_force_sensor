/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#define ROS
//#define DEBUG

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Empty.h>

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

ros::NodeHandle  nh;

geometry_msgs::Wrench leftWrench;
geometry_msgs::Wrench rightWrench;

ros::Publisher leftWrench_pub("leftWrench", &leftWrench);
ros::Publisher rightWrench_pub("rightWrench",&rightWrench);

BLA::Matrix<9> R = {0,0,0,0,0,0,0,0,1};
BLA::Matrix<9> L = {0,0,0,0,0,0,0,0,1};
BLA::Matrix<3,9> AL = {1.49521,1.21442,0.48884,1.10765,-1.93021,-1.10197,-0.21703,-1.05919,0.04535,
0.21098,0.72545,0.01498,-0.46418,-0.01693,0.56959,-0.17388,-0.95753,0.06980,
2.56488,3.10738,3.11407,3.10949,-3.08380,-3.10661,-2.34423,-3.13610,0.01795};


BLA::Matrix<3,9> AR = {1.68307, 0.93943, 0.14401, 1.07988, -1.79573, -0.89472, -0.34823, -0.89765, 0.03359,
0.08388, 0.66539, 0.09018, -0.61163, 0.02772, 0.56665, 0.09652, -0.86233, -0.00371,
3.01506, 3.14856, 3.05043, 3.09569, -3.15789, -3.24155, -3.19147, -3.01541, 0.00615};

BLA::Matrix<3> FR = {0,0,0};
BLA::Matrix<3> FL = {0,0,0};
BLA::Matrix<3> FRinit = {0,0,0};
BLA::Matrix<3> FLinit = {0,0,0};


void setup()
{
  #ifdef ROS
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(leftWrench_pub);
  nh.advertise(rightWrench_pub);
  #endif
  #ifdef DEBUG
  Serial.begin(57600);
  delay(5);
  #endif
  
  for (int i = 0 ; i<8 ; i++) {
    R(i) = analogRead(i)*(5.0/1024.0);
  }
  for (int i = 0 ; i<8 ; i++){
    L(i) = analogRead(i+8)*(5.0/1024.0);
    }

  FRinit = AR*R;
  FLinit = AL*L;
}

void loop()
{

  //current_time = millis();
  for (int i = 0 ; i<8 ; i++) {
    R(i) = analogRead(i)*(5.0/1024.0);
  }
  for (int i = 0 ; i<8 ; i++){
    L(i) = analogRead(i+8)*(5.0/1024.0);
    }

  FR = AR*R-FRinit;
  FL = AL*L-FLinit;
  
  #ifdef DEBUG
  for (int i = 0 ; i<8;i++)
    {Serial.print(L(i));
    Serial.print(',');}
  for (int i = 0 ; i<8;i++)
    {Serial.print(R(i));
    Serial.print(',');}
  Serial.print('\n');
  //f = 1000/(current_time-prev_time); 
  //prev_time = current_time;
  #endif
  
  #ifdef ROS
  leftWrench.force.x = FL(0);
  leftWrench.force.y = FL(1);
  leftWrench.force.z = FL(2);
  rightWrench.force.x = FR(0);
  rightWrench.force.y = FR(1);
  rightWrench.force.z = FR(2);
  
  leftWrench_pub.publish( &leftWrench );
  rightWrench_pub.publish (&rightWrench);
  
  nh.spinOnce();
  //delay(500);
  #endif
}
