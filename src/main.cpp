#include "ros/ros.h"
#include "testpublisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestPublisher");
  ros::NodeHandle nh;
  Tapi::TestPublisher testPublisher(&nh);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    testPublisher.SendTest();
    ros::spinOnce();
  }

  return 0;
}
