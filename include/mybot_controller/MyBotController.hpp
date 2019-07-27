#pragma once

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <geometry_msgs/Twist.h>

#include "sensor_msgs/LaserScan.h"

namespace mybot_controller {


// MyBot controller class

class MyBotController {
public:
	
	// Constructor
	MyBotController(ros::NodeHandle& nodeHandle);

	
	// Destructor
	virtual ~MyBotController();

private:

	bool readParameters();
	void scanCallback(const sensor_msgs::LaserScan& msg);

	ros::NodeHandle& nodeHandle;

	//variable declarations related to subscriber
	ros::Subscriber subscriber;
	std::string scanTopicName;
	int scanTopicQueueSize;

	//variable declarations realated to publisher
	ros::Publisher pubCmdVel;
	std::string pubTopicName;
	int pubTopicQueueSize;
	
	//keeps track of which side of center the robot is on
	bool side;

	//uturn function 
	geometry_msgs::Twist uturn();

	//left turn function
	geometry_msgs::Twist leftTurn(float leftTurnHyp);

	//right turn function
	geometry_msgs::Twist rightTurn();

	//hallway centering function
	geometry_msgs::Twist centerHallway(float left, float right);

	//theta for turns
	float theta = 10. * M_PI / 180.;

	//previous width
	float prevWidth;
	//count, to determine first call to centerHallway
	int count;
};

} /* namespace */
