#include "ros/ros.h"
#include "testclient.hpp"
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyTestClient");
  ros::NodeHandle nh;
  TestClient testclient(&nh);
  while (ros::ok())
  {
    testclient.Connect();
    this_thread::sleep_for(chrono::milliseconds(1000));
  }

  return 0;
}
