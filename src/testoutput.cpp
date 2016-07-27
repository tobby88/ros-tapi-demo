#include "enums.hpp"
#include "feature.hpp"
#include "testoutput.hpp"
#include "tobbytestoutput/Config.h"
#include "tobbytestoutput/Feature.h"
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

TestOutput::TestOutput(NodeHandle *nh)
{
  connected = false;
  this->nh = nh;
  helloClient = nh->serviceClient<tobbytestoutput::Hello>("TobbyAPI/HelloServ");
  configSub = nh->subscribe("TobbyAPI/Config", 1000, &TestOutput::readConfigMsg, this);
}

TestOutput::~TestOutput()
{
}

bool TestOutput::Connect()
{
  tobbytestoutput::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.header = header;
  hello.request.name = "OutputTest";
  uuid_t uuid;
  uuid_generate_random (uuid);
  char uuid_string[37];
  uuid_unparse (uuid, uuid_string);
  //hello.request.uuid = uuid_string;
  hello.request.uuid = "TestOutputClientUUID";
  DeviceType deviceType = DeviceType::OutputDevice;
  hello.request.type = (unsigned short) deviceType;
  tobbytestoutput::Feature feature1;
  feature1.type = (unsigned short) FeatureType::Switch;
  feature1.name = "Button Test";
  uuid_generate_random (uuid);
  uuid_unparse (uuid, uuid_string);
  //feature1.uuid = uuid_string;
  feature1.uuid = "TestOutputFeatureUUID";
  vector<tobbytestoutput::Feature> features;
  features.push_back(feature1);
  hello.request.features = features;
  if (helloClient.call(hello))
  {
    ROS_INFO("Connection established, Status %u, Heartbeat %u", hello.response.status, hello.response.heartbeat);
    connected = true;
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
    connected = false;
  }
  return true;
}

void TestOutput::readConfigMsg(const tobbytestoutput::Config::ConstPtr& msg)
{
  ROS_INFO("Config Msg for: %s, should be connected to publisher %s with feature id %s to feature id %s, coefficient: %3.2f", msg->receiverUUID.c_str(), msg->publisherUUID.c_str(), msg->publisherFeatureUUID.c_str(), msg->receiverFeatureUUID.c_str(), msg->coefficient);
}
