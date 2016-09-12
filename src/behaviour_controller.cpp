#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "bupimo_msgs/VelocityCommand.h"
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <string>

using namespace std;

// Global variables
float currentHeading = 0.;
float currentLinearVel = 0.;

float pfBearing = 0.;
float pfSpeed = 0.;

float voaBearing = 0.;
float voaSpeed = 0.;

bool commandRecieved = false;

void PatternFormationBehaviourCallBack(const bupimo_msgs::VelocityCommand::ConstPtr& msg){
  commandRecieved = true;

  pfBearing = msg->bearing;
  pfSpeed = msg->linearSpeed;
  
}

void ObstacleAvoidanceBehaviourCallBack(const bupimo_msgs::VelocityCommand::ConstPtr& msg){
  commandRecieved = true;
  
  voaBearing = msg->bearing;
  voaSpeed = msg->linearSpeed;
}


void CurrentHeadingCallback(const std_msgs::Float64::ConstPtr& msg){
  currentHeading = msg->data;
}

void CurrentLinearVelCallback(const std_msgs::Float64::ConstPtr& msg){
  currentLinearVel = msg->data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "behaviour_controller");

  ros::NodeHandle n;
 
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Subscriber PatternFormation_Sub = n.subscribe("patternFormationBehaviour", 1000, PatternFormationBehaviourCallBack);
  ros::Subscriber ObstacleAvoidance_Sub = n.subscribe("obstacleAvoidanceBehaviour", 1000, ObstacleAvoidanceBehaviourCallBack);  

  ros::Subscriber currentHeading_Sub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);
  ros::Subscriber currentLinearVel_Sub = n.subscribe("currentLinearVel", 1000, CurrentLinearVelCallback);
  
  ros::Rate loop_rate(10);

  while (ros::ok()){

    if(!commandRecieved){
      printf("Waiting for commands...\n");
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    } 

    float targetHeading;
    float targetLinearVel;

    if(voaSpeed > 0.){
      targetHeading = voaBearing;
      targetLinearVel = voaSpeed;
    }
    else{
      targetHeading = pfBearing;
      targetLinearVel = pfSpeed;
    }

    
    geometry_msgs::Twist msg;

    // Use a proportional controller 
    float headingError = -1.*(targetHeading - currentHeading) ;
      
    if(fabs(headingError) > 180.){
      headingError = 180. - headingError;
    }
    
    // Now convert to radians (per second)
    headingError = headingError * M_PI/180.;
    
    // set values of twist msg.
    msg.linear.x = 0.075*targetLinearVel;
    msg.angular.z = 2.*headingError;
    msg.angular.y = -1.;

    // Need to remove excess digits for transmission over serial.
    msg.linear.x = roundf(msg.linear.x * 100.) / 100.;
    msg.angular.z = roundf(msg.angular.z * 100.) / 100.; 
    
    // publish twist msg.
    twist_pub.publish(msg);
    

    //printf("TargetHeading = %f\t TargetSpeed = %f\n", targetHeading, targetLinearVel);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
};



