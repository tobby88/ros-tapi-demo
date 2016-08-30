#include "ros/ros.h"
#include "testsender.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestSender");
  ros::NodeHandle nh;
  Tapi::TestSender testsender(&nh);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    testsender.SendTest();
    ros::spinOnce();
  }

  return 0;
}
