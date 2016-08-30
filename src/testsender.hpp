#ifndef TESTSENDER_H
#define TESTSENDER_H

#include "ros/ros.h"
#include "tapi_clientlib/publisher.hpp"

namespace Tapi
{
class TestSender
{
public:
  // Constructor/Destructor
  TestSender(ros::NodeHandle* nh);
  ~TestSender();

  // Public member functions

private:
  // Private member variables
  ros::NodeHandle* nh;
  Tapi::Publisher* tpub;
};

#endif  // TESTSENDER_H
