#include <ros/ros.h>
#include "mybot_controller/MyBotController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mybot_controller");
  ros::NodeHandle nodeHandle("~");

  mybot_controller::MyBotController myBotController(nodeHandle);

  ros::spin();
  return 0;
}
