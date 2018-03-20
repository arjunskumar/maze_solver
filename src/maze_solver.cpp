/* Maze Solver Code 

Author : Arjun S Kumar
 */

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

geometry_msgs::Twist velocity_robot;

float leftDistance = 0, rightDistance = 0;

// Distance is working.

void leftSensorCallback(const sensor_msgs::Range::ConstPtr& leftSensorValue)
{
   leftDistance = leftSensorValue->range;
   printf("\n--Left Distance-- %f\n", leftDistance);
}

void rightSensorCallback(const sensor_msgs::Range::ConstPtr& rightSensorValue)
{
   rightDistance = rightSensorValue->range;
   printf("\n--Right Distance-- %f\n", rightDistance);
}

void moveForward(){
  velocity_robot.linear.x = 0.1;
}

void turnRight(){
  velocity_robot.linear.x = 0.05;
  velocity_robot.angular.z = 3.5;
}

void turnLeft(){
  velocity_robot.linear.x = 0.05;
  velocity_robot.angular.z = -3.5;
}

int lostState(){
  if(leftDistance>=13 && rightDistance>=13)
  {
    moveForward();
  }
  else
  {
    return 1;
  }
  return 0;
}

int counterClockWise(){
  if(leftDistance<=13 || rightDistance<=13){
    turnLeft();
  }
  else
  {
    return 2;
  }
  return 1;
}

int wallFindingState(){
  if(rightDistance>13){
    moveForward();
    turnRight();
  }
  else
  {
    return 3;
  }
  return 2;
}

int stillTouchingWall(){
  if(leftDistance>13){
    if(rightDistance<=13){
      turnLeft();
      moveForward();
    }
    else
    {
      return 2;
    }
  }
  else
  {
    return 1;
  }
  return 3;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/group02/cmd_vel", 2048);
  ros::Rate loop_rate(5);

  printf("\n--- GROUP 2 ---\n");

  int state = 0;
  ros::Subscriber left_sub = n.subscribe("/group02/IR_sensor1", 2048, leftSensorCallback);
  ros::Subscriber right_sub = n.subscribe("/group02/IR_sensor2", 2048, rightSensorCallback);
  while(ros::ok())
  {
    switch (state) {
      case 0:
      {
        printf("\nLost State is happening.\n");
        velocity_pub.publish(velocity_robot);
        state = lostState();
        break;
      }
      case 1:
      {
        printf("\nCounter Clockwise State.\n");
        velocity_pub.publish(velocity_robot);
        state = counterClockWise();
        break;
      }
      case 2:
      {
        printf("\nWall Finding state.\n");
        velocity_pub.publish(velocity_robot);
        state = wallFindingState();
        break;
      }
      case 3:
      {
        printf("\nStill touching the wall.\n");
        velocity_pub.publish(velocity_robot);
        state = stillTouchingWall();
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
