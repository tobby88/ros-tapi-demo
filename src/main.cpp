#include <chrono>
#include <thread>
#include "ros/ros.h"
#include "testsender.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI_TestSender");
  ros::NodeHandle nh;
  TestSender testsender(&nh);
  while (ros::ok())
  {
    testsender.Connect();
    this_thread::sleep_for(chrono::milliseconds(1000));
  }

  return 0;
}
