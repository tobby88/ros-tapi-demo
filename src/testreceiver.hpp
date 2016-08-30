#ifndef TESTRECEIVER_H
#define TESTRECEIVER_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "tapi_clientlib/subscriber.hpp"


class TestReceiver
{
public:
  // Constructor/Destructor
  TestReceiver(ros::NodeHandle* nh);
  ~TestReceiver();

private:
  // Private member variables
  double* coefficient;
  ros::NodeHandle* nh;
  Tapi::Subscriber* tsub;

  // Private member functions
  void gotDataBool(const std_msgs::Bool::ConstPtr& msg);
  void gotDataFloat(const std_msgs::Float64::ConstPtr& msg);
};

#endif  // TESTRECEIVER_H
