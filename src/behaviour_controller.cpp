#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "bupimo_msgs/VelocityCommand.h"
#include "geometry_msgs/Twist.h"
#include "Vector2d.h"

#include <stdio.h>
#include <string>

using namespace std;

// Global variables
float currentHeading = 0.;
float currentLinearVel = 0.;

Vector2d * patternFormationVector = new Vector2d(0., 0.);
Vector2d * obstacleAvoidanceVector = new Vector2d(0., 0.);

bool commandRecieved = false;

void PatternFormationBehaviourCallBack(const bupimo_msgs::VelocityCommand::ConstPtr& msg){
  commandRecieved = true;
  
  patternFormationVector = new Vector2d(msg->bearing*M_PI/180.);
  patternFormationVector = ScalarMultiplyVector(msg->linearSpeed, patternFormationVector);
  
}

void ObstacleAvoidanceBehaviourCallBack(const bupimo_msgs::VelocityCommand::ConstPtr& msg){
  commandRecieved = true;

  obstacleAvoidanceVector = new Vector2d(msg->bearing*M_PI/180.);
  obstacleAvoidanceVector = ScalarMultiplyVector(msg->linearSpeed, obstacleAvoidanceVector);
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
  
  ros::Rate loop_rate(50);

  while (ros::ok()){

    if(!commandRecieved){
      printf("Waiting for commands...\n");
      ros::spinOnce();
      loop_rate.sleep();
      continue;
    }

    //Vector2d * commandVector = AddVectors(patternFormationVector, obstacleAvoidanceVector);

    Vector2d * commandVector;
    if(obstacleAvoidanceVector->GetNorm() > 0.) commandVector = obstacleAvoidanceVector;
    else  commandVector = patternFormationVector;
    
    if( commandVector->GetNorm() > 1.) commandVector->Normalize();
    
    float targetHeading = commandVector->GetAngle();
    float targetLinearVel = commandVector->GetNorm();
    
    // Change to degrees
    targetHeading = targetHeading * 180. / M_PI;
    
    geometry_msgs::Twist msg;

    // Use a proportional controller to determine v and w
    /*float headingError = -1.*(targetHeading - currentHeading) ;
      
    if(fabs(headingError) > 180.){
      headingError = 180. - headingError;
    }
    
    // Now convert to radians (per second)
    headingError = headingError * M_PI/180.;*/

    float headingError = currentHeading - targetHeading;
    if( headingError > 180.){
      headingError = headingError - 360.;
    }
    else if(headingError < -180){
      headingError = headingError + 360;
    }
    else{
      headingError = headingError;
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
    
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
};
