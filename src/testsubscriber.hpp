#ifndef TESTSUBSCRIBER_H
#define TESTSUBSCRIBER_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "tapi_clientlib/subscriber.hpp"

namespace Tapi
{
class TestSubscriber
{
public:
  // Constructor/Destructor
  TestSubscriber(ros::NodeHandle* nh);
  ~TestSubscriber();

private:
  // Private member variables
  double* coefficient;
  ros::NodeHandle* nh;
  Tapi::Subscriber* tsub;

  // Private member functions
  void gotDataBool(const std_msgs::Bool::ConstPtr& msg);
  void gotDataFloat(const std_msgs::Float64::ConstPtr& msg);
};
}

#endif  // TESTSUBSCRIBER_H
