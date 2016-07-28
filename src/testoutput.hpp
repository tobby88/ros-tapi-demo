#ifndef TESTOUTPUT_H
#define TESTOUTPUT_H

#include "tobbytestoutput/Config.h"
#include "tobbytestoutput/Hello.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Bool.h"
#include <unordered_map>

using namespace ros;
using namespace std;

class TestOutput
{
private:
  NodeHandle *nh;
  ServiceClient helloClient;
  Subscriber configSub, featureSub;
  std_msgs::Header header;
  bool connected;
  string uuid;
  string featureUuid;
  void readConfigMsg(const tobbytestoutput::Config::ConstPtr& msg);
  void gotData(const std_msgs::Bool::ConstPtr& msg);

public:
  TestOutput(NodeHandle *nh);
  ~TestOutput();
  bool Connect();
};

#endif // TESTOUTPUT_H
