#include "testoutput.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TobbyTestOutput");
  ros::NodeHandle nh;
  TestOutput testOutput(&nh);
  testOutput.Connect();
  while(ros::ok())
  {
      ros::spin();
  }

  return 0;
}
