#include "enums.hpp"
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
  hello.request.Header = header;
  hello.request.Name = "OutputTest";
  /*uuid_t uuid;
  uuid_generate_random (uuid);
  char uuid_string[37];
  uuid_unparse (uuid, uuid_string);*/

  string uuid_string;
  uuid_string = "TestOutputClientUUID";

  this->uuid = uuid_string;
  hello.request.UUID = uuid_string;
  DeviceType deviceType = DeviceType::ReceiverDevice;
  hello.request.DeviceType = (unsigned short)deviceType;
  tobbytestoutput::Feature feature1;
  feature1.FeatureType = (unsigned short)FeatureType::Switch;
  feature1.Name = "Button Test";
  /*uuid_generate_random (uuid);
  uuid_unparse (uuid, uuid_string);*/

  uuid_string = "TestOutputFeatureUUID";

  featureUuid = uuid_string;
  feature1.UUID = uuid_string;
  vector<tobbytestoutput::Feature> features;
  features.push_back(feature1);
  hello.request.Features = features;
  if (helloClient.call(hello))
  {
    ROS_INFO("Connection established, Status %u, Heartbeat %u", hello.response.Status, hello.response.Heartbeat);
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
  ROS_INFO("Config Msg for: %s, should be connected to publisher %s with feature id %s to feature id %s, coefficient: %3.2f", msg->ReceiverUUID.c_str(), msg->SenderUUID.c_str(), msg->SenderFeatureUUID.c_str(), msg->ReceiverFeatureUUID.c_str(), msg->Coefficient);
  if (msg->ReceiverUUID == uuid && msg->ReceiverFeatureUUID == featureUuid)
  {
    ROS_INFO("This message is for me! :)");
    featureSub = nh->subscribe("TobbyAPI/" + msg->SenderUUID + "/" + msg->SenderFeatureUUID, 1000, &TestOutput::gotData, this);
  }
}

void TestOutput::gotData(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Status: %d", msg->data);
}
