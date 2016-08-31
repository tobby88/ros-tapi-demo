#include "testpublisher.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TestPublisher::TestPublisher(ros::NodeHandle* nh) : nh(nh)
{
  tpub = new Tapi::Publisher(nh, "TestPublisher");
  pub[0] = tpub->AddFeature<std_msgs::Bool>("Button", 10);
  pub[1] = tpub->AddFeature<std_msgs::Float64>("AnalogValue", 10);
  number = 0.0;
  truefalse = false;
}

TestPublisher::~TestPublisher()
{
}

// Public member functions

void TestPublisher::SendTest()
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
