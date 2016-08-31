#include "testsubscriber.hpp"
#include <uuid/uuid.h>
#include "tapi_msgs/Config.h"
#include "tapi_msgs/Feature.h"

using namespace std;

namespace Tapi
{
TestSubscriber::TestSubscriber(ros::NodeHandle* nh) : nh(nh)
{
  tsub = new Tapi::Subscriber(nh, "TestSubscriber");
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(std_msgs::Bool, 5, &TestSubscriber::gotDataBool);
  tsub->AddFeature(opt, "Bool Topic");
  coefficient =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 5, &TestSubscriber::gotDataFloat), "Float Topic");
}

TestSubscriber::~TestSubscriber()
{
}

void TestSubscriber::gotDataBool(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Bool: true");
  else
    ROS_INFO("Bool: false");
}

void TestSubscriber::gotDataFloat(const std_msgs::Float64::ConstPtr& msg)
{
  double value;
  value = msg->data * *coefficient;
  ROS_INFO("Double/Float64: %f with coefficient %f", value, *coefficient);
}
}
