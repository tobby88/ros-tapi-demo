#ifndef TESTPUBLISHER_H
#define TESTPUBLISHER_H

#include "ros/ros.h"
#include "tapi_clientlib/publisher.hpp"

namespace Tapi
{
class TestPublisher
{
public:
  // Constructor/Destructor
  TestPublisher(ros::NodeHandle* nh);
  ~TestPublisher();

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

#endif  // TESTPUBLISHER_H
