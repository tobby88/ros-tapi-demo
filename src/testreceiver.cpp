#include "testreceiver.hpp"
#include <uuid/uuid.h>
#include "tapi_msgs/Config.h"
#include "tapi_msgs/Feature.h"

using namespace std;

namespace Tapi
{
TestReceiver::TestReceiver(ros::NodeHandle* nh) : nh(nh)
{
  tsub = new Tapi::Subscriber(nh, "Test2");
  tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Bool, 5, &TestReceiver::gotDataBool), "Bool");
  coefficient = tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 5, &TestReceiver::gotDataFloat), "Float");
}

TestReceiver::~TestReceiver()
{
}

void TestReceiver::gotDataBool(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Bool: true");
  else
    ROS_INFO("Bool: false");
}

void TestReceiver::gotDataFloat(const std_msgs::Float64::ConstPtr& msg)
{
  double value;
  value = msg->data * *coefficient;
  ROS_INFO("Double/Float64: %f with coefficient %f", value, *coefficient);
}
}
