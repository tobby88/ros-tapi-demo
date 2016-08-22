#include "ros/ros.h"
#include "testreceiver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI_TestReceiver");
  ros::NodeHandle nh;
  TestReceiver testReceiver(&nh);
  testReceiver.Connect();
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
