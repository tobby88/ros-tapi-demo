#ifndef TESTSENDER_H
#define TESTSENDER_H

#include "ros/ros.h"
#include "std_msgs/Header.h"

using namespace ros;
using namespace std;

class TestSender
{
public:
  // Constructor/Destructor
  TestSender(NodeHandle* nh);
  ~TestSender();

  // Public member functions
  bool Connect();

private:
  // Private member variables
  std_msgs::Header header;
  ServiceClient helloClient;
  NodeHandle* nh;
};

#endif  // TESTSENDER_H
