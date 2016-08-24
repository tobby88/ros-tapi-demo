#ifndef TESTRECEIVER_H
#define TESTRECEIVER_H

#include <unordered_map>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "tobbyapi_msgs/Config.h"
#include "tobbyapi_msgs/Hello.h"

using namespace ros;
using namespace std;

class TestReceiver
{
public:
  // Constructor/Destructor
  TestReceiver(NodeHandle* nh);
  ~TestReceiver();

  // Public member functions
  bool Connect();

private:
  // Private member variables
  Subscriber configSub;
  bool connected;
  Subscriber featureSub[2];
  string featureUuid[2];
  std_msgs::Header header;
  ServiceClient helloClient;
  NodeHandle* nh;
  string uuid;

  // Private member functions
  void gotDataBool(const std_msgs::Bool::ConstPtr& msg);
  void gotDataFloat(const std_msgs::Float64::ConstPtr& msg);
  void readConfigMsg(const tobbyapi_msgs::Config::ConstPtr& msg);
};

#endif  // TESTRECEIVER_H
