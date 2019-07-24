#include "mybot_controller/MyBotController.hpp"

namespace mybot_controller {

MyBotController::MyBotController(ros::NodeHandle& nodeHandle) :
  nodeHandle(nodeHandle)
{
	//read parameters
	if(!readParameters()) {
		ROS_ERROR("Could not find topic parameter!");
		ros::requestShutdown();
	}

	//launch subscriber
	subscriber = nodeHandle.subscribe(scanTopicName, scanTopicQueueSize, &MyBotController::scanCallback, this);
	//launch publisher
	pubCmdVel = nodeHandle.advertise<geometry_msgs::Twist>(pubTopicName, pubTopicQueueSize);
	ROS_INFO("Node has been launched successfully");
}


MyBotController::~MyBotController()
{
}

bool MyBotController::readParameters() {
	//get scan topic name parameter
	if(!nodeHandle.getParam("scanTopicName", scanTopicName)) {
		return false;
	}

	//get scan topic queue parameter
	if(!nodeHandle.getParam("scanTopicQueueSize", scanTopicQueueSize)) {
		return false;
	}

	//get publisher topic name parameter
	if(!nodeHandle.getParam("pubTopicName", pubTopicName)) {
		return false;
	}
 
	//get publisher queue size parameter
	if(!nodeHandle.getParam("pubTopicQueueSize", pubTopicQueueSize)) {
		return false;
	}	

	return true;
}

void MyBotController::scanCallback(const sensor_msgs::LaserScan& msg) {
	//create a vector of the ranges from the Laser Scan msg
	std::vector<float> ranges = msg.ranges;
	
	int size = 0;
	for(float i : ranges) {
		size++;
	}

	float left = ranges[719];
	float right = ranges[0];
	float front = ranges[360];
	float leftTurnHyp = ranges[640];
	float rightTurnHyp = ranges[80];
	float tolerance = (left + right) / 3.5;
	float uturnDistance = (left + right) / 2; 


	//the min max and step data was printed and used to calculate the 120 and 600 indexes to fined the left and right wall distance
	ROS_INFO_STREAM("min: " << msg.angle_min << " max: " << msg.angle_max << " step: " << msg.angle_increment << " size: " << size << std::endl);

	ROS_INFO_STREAM("left: " << left << " Right: " << right << std::endl);
	
	ROS_INFO_STREAM("side: " << side << std::endl); //testing code

	//create velocity message
	geometry_msgs::Twist velMsg;
	
	ROS_INFO_STREAM("leftTurnHyp: " << leftTurnHyp << " left * cos(theta): " << left * cos(theta)); //testing code
	ROS_INFO_STREAM(" diff: " << leftTurnHyp - (left * cos(theta)) << std::endl); //testing code

	//if turn, turn otherwise move to hallway center
	if(front < uturnDistance) {
		ROS_INFO("U-TURN"); //testing code
		velMsg = uturn();
	}
	else if(leftTurnHyp > left * cos(theta) + tolerance) {
		ROS_INFO("LEFT TURN\n"); //testing code
		velMsg = leftTurn();
	}
	else if(rightTurnHyp > right * cos(theta) + tolerance) {
		ROS_INFO("RIGHT TURN\n"); //testing code
		velMsg = rightTurn();
	}
	else {
		ROS_INFO("CENTER HALLWAY\n"); //testing code
		velMsg = centerHallway(left, right);
	}
	//publish command velocity
	pubCmdVel.publish(velMsg);
}

geometry_msgs::Twist MyBotController::uturn() {
	geometry_msgs::Twist velMsg;
	velMsg.angular.z = 1.;

	return velMsg;
}

geometry_msgs::Twist MyBotController::leftTurn() {
	geometry_msgs::Twist velMsg;
	velMsg.linear.x = .1;
	velMsg.angular.z = .4;

	return velMsg;
}

geometry_msgs::Twist MyBotController::rightTurn() {
	geometry_msgs::Twist velMsg;
	velMsg.linear.x = .1;
	velMsg.angular.z = -.4;

	return velMsg;
}

geometry_msgs::Twist MyBotController::centerHallway(float left, float right) {
	
	//husky calibration constants. Must be modified experimentally.
	float linearReflectionVel = .6;
	float angularReflectionFactor = .23;
	float linearCenterVel = .6;
	float angularCenterVel = 0.;
	float linearSideVel = .4;
	float angularSideFactor = .09;
	float maxSideDistance = 4;
	float centerSpace = (left + right) / 8; 
 
	/*
	//calibration constants.  Must be modified experimentally.
	float linearReflectionVel = .15;
	float angularReflectionFactor = .25;
	float linearCenterVel = .15;
	float angularCenterVel = 0.;
	float linearSideVel = .1;
	float angularSideFactor = .005;
	float maxSideDistance = 3;
	float centerSpace = (left + right) / 8; 
	*/

	geometry_msgs::Twist velMsg;

	//if robot is in the center, go straight forward
	if (abs(left - right) < centerSpace) {
		if(side) { //right side
			if(left - right > 0) { //before center point
				velMsg.linear.x = linearReflectionVel;
				velMsg.angular.z = (right - left) * angularReflectionFactor;
				ROS_INFO("TEST 1"); //testing code
			}
			else { //after center point
				velMsg.linear.x = linearCenterVel;
				velMsg.angular.z = angularCenterVel;
				ROS_INFO("TEST 2"); //testing code
			}
		}		
		else { //left side
			if(left - right < 0) { //before center point
				velMsg.linear.x = linearReflectionVel;
				velMsg.angular.z = (right - left) * angularReflectionFactor;
				ROS_INFO("TEST 1"); //testing code
			}
			else { //after center point
				velMsg.linear.x = linearCenterVel;
				velMsg.angular.z = angularCenterVel;
				ROS_INFO("TEST 2"); //testing code
			}

		}
	}
	//if robot is not in center, move towards center
	else {
		if(left - right < 0) { //left side
			side = false;
		}
		else { //right side
			side = true;
		}
		//set linear velocity
		velMsg.linear.x = linearSideVel;

		//set angular velocity
		if(left - right > maxSideDistance) { //highest angular velocity allowed
			velMsg.angular.z = maxSideDistance * angularSideFactor;
			ROS_INFO("Max Side Distance"); //testing code
		}
		else if(right - left > maxSideDistance) {
			velMsg.angular.z = maxSideDistance * angularSideFactor;
			ROS_INFO("Max Side Distance"); //testing code
		}
		else {
			velMsg.angular.z = (left - right) * angularSideFactor;
		}
		ROS_INFO_STREAM("l-r: " << left - right << std::endl); //testing code
	}

	ROS_INFO_STREAM("angular velocity: " << velMsg.angular.z << std::endl);
	return velMsg;
}
} /* namespace */
