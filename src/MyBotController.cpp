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

	wideCount = 0;
	narrowCount = 0;
	prevWidth = INFINITY;

	//launch subscriber
	subscriber = nodeHandle.subscribe(scanTopicName, scanTopicQueueSize, &MyBotController::scanCallback, this);
	//launch publisher
	pubCmdVel = nodeHandle.advertise<geometry_msgs::Twist>(pubTopicName, pubTopicQueueSize);
	ROS_INFO("Node has been launched successfully");
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

	/* // 180 view
	float left = ranges[719];
	float right = ranges[0];
	float front = ranges[359];
	float leftTurnHyp = ranges[680]; //10 degrees
	float leftTurnHypLong = ranges[440]; //70 degrees
	//float rightTurnHyp = ranges[80]; //20 degrees
	float tolerance = (left + right) / 2.5;
	float leftUturnGuide = ranges[520]; //50 degrees
	float rightUturnGuide = ranges[200]; //50 degrees
	*/

	float left = ranges[119];
	float right = ranges[600];
	float front = ranges[359];
	float leftTurnHyp = ranges[145]; //10 degrees
	float leftTurnHypLong = ranges[306]; //70 degrees
	//float rightTurnHyp = ranges[80]; //20 degrees
	float tolerance = (left + right) / 2.5;
	float leftUturnGuide = ranges[253]; //50 degrees
	float rightUturnGuide = ranges[456]; //50 degrees

	//set uturn distance
	float uturnDistance; 
	float uturnDistanceFactor = 1./2.8;
	bool cornerUturnBool = false; //for corner condition
	if(left > 2. * right) {
		uturnDistance = 2. * right * uturnDistanceFactor;
	}
	else if(right > 2. * left) {
		uturnDistance = 2. * left * uturnDistanceFactor;
	}
	else {
		uturnDistance = (left + right) * uturnDistanceFactor;
		cornerUturnBool = true;
	}
	//minimum and miximum uturn distance
	float maximumUturnDistance = 1.5;
	float minimumUturnDistance = .8;
	if(uturnDistance > maximumUturnDistance) {
		uturnDistance = maximumUturnDistance;
	}
	if(uturnDistance < minimumUturnDistance) {
		uturnDistance = minimumUturnDistance;
	}

	ROS_INFO_STREAM("uturnDistance: " << uturnDistance);

	//the min max and step data was printed and used to calculate the 120 and 600 indexes to fined the left and right wall distance
	ROS_INFO_STREAM("min: " << msg.angle_min << " max: " << msg.angle_max << " step: " << msg.angle_increment << " size: " << size << std::endl);

	ROS_INFO_STREAM("left: " << left << " Right: " << right << " Front: " << front << std::endl);
	
	//ROS_INFO_STREAM("side: " << side << std::endl); //testing code

	//create velocity message
	geometry_msgs::Twist velMsg;
	
	ROS_INFO_STREAM("leftTurnHyp: " << leftTurnHyp << " leftTurnHypLong: " << leftTurnHypLong << " left * cos(theta): " << left * cos(theta)); //testing code
	ROS_INFO_STREAM(" diff: " << leftTurnHyp - (left * cos(theta)) << std::endl); //testing code

	//robot decision condtionals
	float skipUturnFactor = 2.; //if a turn should be made instead of uturnr
	float cornerUturnDistance = .5; //indicates corner condition
	ROS_INFO("R TEST: %d\n", front < uturnDistance && right > 2. * uturnDistance && left < 1.5 * uturnDistance);
	if((front < uturnDistance && left + right < (skipUturnFactor / uturnDistanceFactor) * front) || (uturnDistance < cornerUturnDistance && cornerUturnBool)) {
		wideCount = 0;
		narrowCount = 0;
		ROS_INFO("U-TURN"); //testing code
		velMsg = uturn(left, right, leftUturnGuide, rightUturnGuide);
	}
	else if(front < uturnDistance && left > 1.5 * uturnDistance && right < 1.5 * uturnDistance) {
		wideCount = 0;
		narrowCount = 0;
		ROS_INFO("LEFT U TURN ASSIST\n");
		velMsg = leftUturnAssist();
	}
	else if(leftTurnHyp > left * cos(theta) + tolerance || (leftTurnHyp > 1.5 * leftTurnHypLong && leftTurnHypLong > 1.1 * front)) { // && (leftTurnHyp > 1.5 * front || leftTurnHyp > 1.5 * right))) {
		wideCount = 0;
		narrowCount = 0;
		ROS_INFO("LEFT TURN\n"); //testing code
		velMsg = leftTurn(leftTurnHyp);
	}
	else if(front < uturnDistance && right > 2. * uturnDistance && left < 1.5 * uturnDistance) {
		wideCount = 0;
		narrowCount = 0;
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

geometry_msgs::Twist MyBotController::uturn(float left, float right, float leftUturnGuide, float rightUturnGuide) {
	geometry_msgs::Twist velMsg;

	if(left > 3 * right) {
		velMsg.angular.z = 3.;
	}
	else if( right > 3 * left) {
		velMsg.angular.z = -3.;
	}
	else if(leftUturnGuide > rightUturnGuide) {
		velMsg.angular.z = 3.;
	}
	else {
		velMsg.angular.z = -3.;
	}
	velMsg.linear.x = -.7;

	return velMsg;
}

geometry_msgs::Twist MyBotController::leftTurn(float leftTurnHyp) {
	geometry_msgs::Twist velMsg;

	velMsg.linear.x = .5;
	velMsg.angular.z = 0.9;
	
	return velMsg;
}

geometry_msgs::Twist MyBotController::rightTurn() {
	geometry_msgs::Twist velMsg;

	velMsg.linear.x = -.7;
	velMsg.angular.z = -3.;
	
	return velMsg;
}

geometry_msgs::Twist MyBotController::leftUturnAssist() {
	geometry_msgs::Twist velMsg;

	velMsg.linear.x = -.7;
	velMsg.angular.z = 3.;
	
	return velMsg;
}

geometry_msgs::Twist MyBotController::centerHallway(float left, float right) {

	//husky calibration constants. Must be modified experimentally.
	float linearReflectionVel = .75;
	float angularReflectionFactor = 1.;
	float linearCenterVel = .9;
	float angularCenterVel = 0.;
	float linearSideVel = .6;
	float angularSideFactor = .11;
	float maxSideDistance = 4;
	float centerSpace = (left + right) / 8; 

	/*
	//calibration constants.  Must be modified experimentally.
	float linearReflectionVel = .2;
	float angularReflectionFactor = .6;
	//float linearCenterVel = .6;
	//float angularCenterVel = 0.;
	float linearSideVel = .25;
	float angularSideFactor = .15;
	float maxSideDistance = 4;
	float centerSpace = (left + right) / 8; 
	*/

	geometry_msgs::Twist velMsg;

	//the following code creates a false wall on a right turn that should be skipped	
	ROS_INFO_STREAM("prev width: " << prevWidth << " wideCount: " << wideCount << " narrowCount: " << narrowCount << std::endl); //testing code
	if(left + right > 1.02 * prevWidth) {
		if(wideCount < 500) { 
			right = prevWidth - left;
		}
		else {
			prevWidth += .1  * (left + right - prevWidth);
			right = prevWidth - left;
		 }
		wideCount++;
		narrowCount=0;
	}
	else {
		if(narrowCount > 30) {
			prevWidth = left + right;
		}
		narrowCount++;
		wideCount = 0;
	}
	
	//if robot is in the center, go straight forward
	if (abs(left - right) < centerSpace) {
		if(side) { //right side
			if(left - right > 0) { //before center point
				velMsg.linear.x = linearSideVel;
				velMsg.angular.z = (left - right) * angularSideFactor;
				ROS_INFO("TEST 1"); //testing code
			}
			else { //after center point
				velMsg.linear.x = linearReflectionVel;
				velMsg.angular.z = (left - right) * angularReflectionFactor;
				ROS_INFO("TEST 2"); //testing code
			}
		}		
		else { //left side
			if(left - right < 0) { //before center point
				velMsg.linear.x = linearSideVel;
				velMsg.angular.z = (left - right) * angularSideFactor;
				ROS_INFO("TEST 1"); //testing code
			}
			else { //after center point
				velMsg.linear.x = linearReflectionVel;
				velMsg.angular.z = (left - right) * angularReflectionFactor;
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
			velMsg.angular.z = -maxSideDistance * angularSideFactor;
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
