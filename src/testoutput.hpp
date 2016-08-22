#ifndef TESTOUTPUT_H
#define TESTOUTPUT_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "tobbytestoutput/Config.h"
#include "tobbytestoutput/Hello.h"
#include <unordered_map>

using namespace ros;
using namespace std;

class TestOutput
{
public:
  // Constructor/Destructor
  TestOutput(NodeHandle* nh);
  ~TestOutput();

  // Public member functions
  bool Connect();

private:
  // Private member variables
  Subscriber configSub;
  bool connected;
  Subscriber featureSub;
  string featureUuid;
  std_msgs::Header header;
  ServiceClient helloClient;
  NodeHandle* nh;
  string uuid;

  // Private member functions
  void gotData(const std_msgs::Bool::ConstPtr& msg);
  void readConfigMsg(const tobbytestoutput::Config::ConstPtr& msg);
};

#endif // TESTOUTPUT_H
