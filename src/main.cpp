#include "testclient.hpp"
#include "ros/ros.h"
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TobbyTestClient");
  ros::NodeHandle nh;
  TestClient testclient(&nh);
  while(ros::ok())
  {
    testclient.Connect();
    this_thread::sleep_for(chrono::milliseconds(2000));
  }

  return 0;
}
