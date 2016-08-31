#include "ros/ros.h"
#include "testsubscriber.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestSubscriber");
  ros::NodeHandle nh;
  Tapi::TestSubscriber testSubscriber(&nh);
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
