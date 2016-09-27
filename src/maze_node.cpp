//copyright Arjun S Kumar: arjunskumar22@gmail.com

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <utility>
#include <string>
#include <memory>
#include <sstream>
#include <numeric>  //for using accumulate on vector
#include <algorithm>  
//ROS specific libraries
#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "ros/time.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <signal.h>
#include "sensor_msgs/LaserScan.h"
//#include <turtlebot_node/TurtlebotSensorState.h> DOES NOT EXIST ANYMORE
using namespace std;

//TO DELETE AFTER OPTIMISATION#####################
double SpeedForward(0.0), SpeedInTurns(0.0), SpeedTR_F(0.0), SpeedTR_TR(0.0),SpeedTLbeforeTR_F(0.0),SpeedTLbeforeTR_TL(0.0);
vector <int> AvWall1, AvWall2;
int wall2_count(0);  //measures how much time is spent in WALL2
//#########################################################

double sensing_threshold(0.75);          //TO MEASURE 0.75 for turtlebot 15 for Flash
bool StopOnKill(false), sensorRight(false), sensorLeft(false), away_before_turn(false);
int alternate_count(0), wall1_count(0);  //to make sure motion is decomposed and measures how much time is spent in WALL1
	
/////////////CALLBACK FUNCTIONS///////////////////////////////////////////////////
void leftDistance(const std_msgs::UInt16& distMeasuredLeft){sensorLeft=(distMeasuredLeft.data<=sensing_threshold);} //TO FILTER UNSIGNIFICANT READINGS!!!
void rightDistance(const std_msgs::UInt16& distMeasuredRight){sensorRight=(distMeasuredRight.data<=sensing_threshold);}
/////////////////////////////////////////////////////////////////////////////////

////////////////////GAZEBO CALL BACK FUNCTIONS///////////////////////////////////
void scannedYorN(const sensor_msgs::LaserScan& scanned){

	sensorRight=(scanned.ranges[0]<=sensing_threshold);
	sensorLeft=(scanned.ranges.back()<=sensing_threshold);

	cout<<"Range Right: "<<scanned.ranges[0]<<endl;
	cout<<"Range Left: "<<scanned.ranges.back()<<endl;
}
/////////////////////////////////////////////////////////////////////////////////

//////////////MOTION COMMANDS///////////////////////////////////////////////////
void moveF(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){
		velocityCtrlCmd.linear.x=SpeedForward;                        //0.3
		velocityCtrlCmd.angular.z=0.0;                        //0.0
		VEL_CMD.publish(velocityCtrlCmd);
    return;
	}
void moveF_Lost(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){
		velocityCtrlCmd.linear.x=0.3;                         //0.3
		velocityCtrlCmd.angular.z=0.0;                        //0.0
		VEL_CMD.publish(velocityCtrlCmd);
    return;
	}
void moveTR(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){   //to turn in an arc circle
		velocityCtrlCmd.linear.x=SpeedTR_F;                      //0.2               //to go around corner tightly
		velocityCtrlCmd.angular.z= -SpeedTR_TR;                    //-0.4
		VEL_CMD.publish(velocityCtrlCmd);
	return;
	}
void moveTLbeforeTR(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){   //away from the corner before
		velocityCtrlCmd.linear.x=SpeedTLbeforeTR_F;                       //0.1                       //tightening the turn
		velocityCtrlCmd.angular.z=SpeedTLbeforeTR_TL;                      //0.4
		VEL_CMD.publish(velocityCtrlCmd);
	return;
	}
void moveTR_not_F(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){ //to turn on the spot so that the trajectory
		velocityCtrlCmd.linear.x=0.0;                     //0.0                      //is as straight as possible
		velocityCtrlCmd.angular.z= -SpeedInTurns;                   //-0.4
		VEL_CMD.publish(velocityCtrlCmd);
}
void moveTL(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){
		velocityCtrlCmd.linear.x=0.0;                          //0.0
		velocityCtrlCmd.angular.z=SpeedInTurns;                         //0.4
		VEL_CMD.publish(velocityCtrlCmd); 
	return;
	}
//////////////////////////////////////////////////////////////////////////////

////////////STATE FUNCTIONS///////////////////////////////////////////////////
int LOST(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){           //OK
	if(sensorLeft==false && sensorRight==false){
		moveF_Lost(velocityCtrlCmd, VEL_CMD);
		return 0;
	}else{
		return 1;
	}	
}
int CCW(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){             //OK
	if (sensorLeft==true || sensorRight==true){
		moveTL(velocityCtrlCmd, VEL_CMD);
		return 1;
	}else{
		return 2;
	}
}
int WALL1 (geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){          //OK
	
	if(sensorRight==false){
		if (wall1_count<=6){             //ABOVE THIS COUNT ON AVERAGE ROBOT MUST BE TURNING A CORNER OR 180 DEGREES
			if (alternate_count % 2 == 0){           //THIS IS TO MAKE SURE BOTH FORWARD AND TURN COMMANDS ARE TAKEN INTO ACCOUNT
				moveF(velocityCtrlCmd, VEL_CMD);     //FOR 200ms EACH OTHERWISE ONLY TURN IS HAPPENING
			}else{
				moveTR_not_F(velocityCtrlCmd, VEL_CMD);
			}
		}else{
			if (away_before_turn){
				moveTR(velocityCtrlCmd, VEL_CMD);
			}else{
				moveTLbeforeTR(velocityCtrlCmd, VEL_CMD);
				away_before_turn=true;
				}
			}
		
		++alternate_count;
		++wall1_count;
		
		return 2;
	}else{
		AvWall1.push_back(wall1_count);   //TO DELETE EVENTUALLY
		alternate_count=0;
		wall1_count=0;
		away_before_turn=false;
		return 3;
	}
}
int WALL2(geometry_msgs::Twist& velocityCtrlCmd, ros::Publisher& VEL_CMD){          //OK
	if (sensorLeft==false){		
		if (sensorRight==true){
			if (alternate_count % 2 == 0){
				moveTL(velocityCtrlCmd, VEL_CMD); 
			}else{
				moveF(velocityCtrlCmd, VEL_CMD);
			}
			++alternate_count;
			++wall2_count;   //TO DELETE EVENTUALLY
	        return 3; 
		}else{
			AvWall2.push_back(wall2_count);   //TO DELETE EVENTUALLY
			wall2_count=0;   //TO DELETE EVENTUALLY
			alternate_count=0;
			return 2;
		}
	}else{
		AvWall2.push_back(wall2_count);   //TO DELETE EVENTUALLY
		wall2_count=0;   //TO DELETE EVENTUALLY
		alternate_count=0;
		return 1;
	}
}
/////////////////////////////////////////////////////////////////////////////
void mySigintHandler(int sig)
{
  // TO FLAG ARDUINO TO STOP BEFORE ROS MASTER SHUTS DOWN
  StopOnKill=true;
  // All the default sigint handler does is call shutdown() here it is modified so it does not 
  //shutdown() is called later explicitely
}
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	//SECTION USED TO OPTIMISE AND TUNE THE PARMETERS TO DELETE AFTER USE#####################
	
	double Duration=0.0;
	cout<<"Sensing Range Threshold: ";
	cin>>sensing_threshold;
	cout<<"Forward Speed: ";
	cin>>SpeedForward;
	cout<<"Angular Speed Only: ";
	cin>>SpeedInTurns;
	cout<<"Forward Speed at Corners: ";
	cin>>SpeedTR_F;
	cout<<"Angular Speed at Corners: ";
	cin>>SpeedTR_TR;
	cout<<"Left Forward Speed at Corners: ";
	cin>>SpeedTLbeforeTR_F;
	cout<<"Left Angular Speed at Corners: ";
	cin>>SpeedTLbeforeTR_TL;

	//#########################TO DELETE!!!!!!!!#################################################
  
  ros::init(argc, argv, "Flash_ai");
  ros::NodeHandle nh;
  
  ros::Publisher initial_pose = nh.advertise<geometry_msgs::Pose2D>("groupF/initial_pose", 1000);
  ros::Publisher vel_ctrl_cmd = nh.advertise<geometry_msgs::Twist>("groupF/velocity_cmd", 1000);
  ros::Publisher lets_rumble = nh.advertise<std_msgs::Bool>("lets_rumble", 1000);
  ros::Publisher rgb_led = nh.advertise<std_msgs::UInt8MultiArray>("groupF/rgb_led", 1000);
  
  //DEBUGGING PUBLISHER TO DELETE ONCE DONE
  /*ros::Publisher debugLeft = nh.advertise<std_msgs::UInt16>("groupFlash/debugLeft", 1000);
  ros::Publisher debugRight = nh.advertise<std_msgs::UInt16>("groupFlash/debugRight", 1000);
  ros::Publisher debugLDR = nh.advertise<std_msgs::UInt16>("groupFlash/debugLDR", 1000);*/
  //std_msgs::UInt16& distMeasuredLeft distMeasuredRight distMeasuredLDR
  
  ros::Subscriber leftDist = nh.subscribe("groupF/IR_left", 1000, leftDistance);
  ros::Subscriber rightDist = nh.subscribe("groupF/IR_right", 1000, rightDistance);
  
  //GAZEBO PUBLISHER
  ros::Publisher turtleBotVelCmd = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
  //GAZEBO SUBSCRIBER
  ros::Subscriber bumperState = nh.subscribe("/scan", 1000, scannedYorN);
  
  ros::Rate loop_rate(5); //5Hz
  int state = 0;          //INITIAL STATE ALWAYS 0
  geometry_msgs::Twist velocityCtrlCmd;
  double Timebeginning =ros::Time::now().toSec();   //time of start
  while (ros::ok())
  {
	  //DEBUG TOPICS TO DELETE ONCE DONE
	  /*std_msgs::UInt16 distMeasuredLeft1, distMeasuredRight1, distMeasuredLDR1;
	  distMeasuredLeft1.data=DistanceMeasuredLeft;
	  distMeasuredRight1.data=DistanceMeasuredRight;
	  distMeasuredLDR1.data=DistanceMeasuredLDR;
	  debugLeft.publish(distMeasuredLeft1);
	  debugRight.publish(distMeasuredRight1);
	  debugLDR.publish(distMeasuredLDR1);*/
	  //kobuki_msgs::BumperEvent fuckme;
	//cout<<DistanceMeasuredLeft<<" "<<endl;
	//cout<<DistanceMeasuredRight<<" "<<endl;
	//cout<<DistanceMeasuredLDR<<" "<<endl;
	//cout<<sensorLeft()<<" "<<endl;
	//cout<<sensorRight()<<" "<<endl;
	//std_msgs::UInt16& distMeasuredLeft distMeasuredRight distMeasuredLDR
	///////////////////////////////////
	
	// Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
	signal(SIGINT, mySigintHandler);
	if(StopOnKill){
		std_msgs::Bool kill_node;
		kill_node.data=false;
		lets_rumble.publish(kill_node);
		Duration=ros::Time::now().toSec()-Timebeginning;   //TO DELETE AFTER OPTIMISATION
		ros::shutdown();
	}

	switch(state){
		case 0 :
			ROS_INFO("LOST");  //ROS PRINT
			state=LOST(velocityCtrlCmd, turtleBotVelCmd);   //MIND THE PUBLISHER TO UPDATE!!!!
			break;                                          //turtleBotVelCmd for TURTLEBOT
		case 1 :                                            //vel_ctrl_cmd for FLASH
			ROS_INFO("CCW");
			state=CCW(velocityCtrlCmd, turtleBotVelCmd);			
			break;
		case 2 :
			ROS_INFO("WALL1");
			state=WALL1(velocityCtrlCmd, turtleBotVelCmd); 
			break;
		case 3 :
			ROS_INFO("WALL2");
			state=WALL2(velocityCtrlCmd, turtleBotVelCmd);
			break;
		}

    ros::spinOnce();  //FOR CALLBACKS
    loop_rate.sleep();

  }
  ////////TO DELETE AFTER OPTIMISATION////////////////
  cout<<endl;
  cout<<endl;
  cout<<"This run lasted: "<<Duration<<" seconds"<<endl;
  double averageWall1=std::accumulate(AvWall1.begin(),AvWall1.end(),0.0)/AvWall1.size();
  double averageWall2=std::accumulate(AvWall2.begin(),AvWall2.end(),0.0)/AvWall2.size();
  //std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
  cout<<"# of times in WALL1 on normal wall: "<<averageWall1<<" ie: "<<0.2*averageWall1<<" seconds."<<endl;
  cout<<"# of times in WALL2 on normal wall: "<<averageWall2<<" ie: "<<0.2*averageWall2<<" seconds."<<endl;
  cout<<"Sensing Range Threshold: "<<sensing_threshold<<endl;
  cout<<"Forward Speed: "<<SpeedForward<<endl;
  cout<<"Angular Speed Only: "<<SpeedInTurns<<endl;
  cout<<"Forward Speed at Corners: "<<SpeedTR_F<<endl;
  cout<<"Angular Speed at Corners: "<<SpeedTR_TR<<endl;
  cout<<"Left Forward Speed at Corners: "<<SpeedTLbeforeTR_F<<endl;
  cout<<"Left Angular Speed at Corners: "<<SpeedTLbeforeTR_TL<<endl;
///////////////////////////////////////////////////
  return 0;
}

