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
  void SendTest();

private:
  // Private member variables
  ros::NodeHandle* nh;
  double number;
  ros::Publisher* pub[2];
  Tapi::Publisher* tpub;
  bool truefalse;
};
}

#endif  // TESTSENDER_H
