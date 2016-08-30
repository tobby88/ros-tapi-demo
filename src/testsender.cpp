#include "testsender.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TestSender::TestSender(ros::NodeHandle* nh) : nh(nh)
{
  tpub = new Tapi::Publisher(nh, "TestSender");
  pub[0] = tpub->AddFeature(tapi_msgs::Feature::Type_Switch, 10, "Button");
  pub[1] = tpub->AddFeature(tapi_msgs::Feature::Type_AnalogValue, 10, "AnalogValue");
  number = 0.0;
  truefalse = false;
}

TestSender::~TestSender()
{
}

// Public member functions

void TestSender::SendTest()
{
  std_msgs::Bool m1;
  std_msgs::Float64 m2;
  m1.data = truefalse;
  m2.data = number;
  pub[0]->publish(m1);
  pub[1]->publish(m2);
  if (truefalse)
    truefalse = false;
  else
    truefalse = true;
  number += 1.0001;
}
}
