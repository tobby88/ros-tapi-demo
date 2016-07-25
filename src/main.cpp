#include "testclient.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TobbyTestClient");
  ros::NodeHandle nh;
  TestClient testclient(&nh);
  testclient.Connect();

  return 0;
}
