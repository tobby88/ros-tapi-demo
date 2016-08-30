#include "ros/ros.h"
#include "testreceiver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestReceiver");
  ros::NodeHandle nh;
  Tapi::TestReceiver testReceiver(&nh);
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
