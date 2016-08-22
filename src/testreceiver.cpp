#include "testreceiver.hpp"
#include "enums.hpp"
#include "tobbyapi_testreceiver/Config.h"
#include "tobbyapi_testreceiver/Feature.h"
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

TestReceiver::TestReceiver(NodeHandle* nh)
{
  connected = false;
  this->nh = nh;
  helloClient =
      nh->serviceClient<tobbyapi_testreceiver::Hello>("TobbyAPI/HelloServ");
  configSub = nh->subscribe("TobbyAPI/Config", 1000,
                            &TestReceiver::readConfigMsg, this);
}

TestReceiver::~TestReceiver() {}

bool TestReceiver::Connect()
{
  tobbyapi_testreceiver::Hello hello;
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
  tobbyapi_testreceiver::Feature feature1;
  feature1.FeatureType = (unsigned short)FeatureType::Switch;
  feature1.Name = "Button Test";
  /*uuid_generate_random (uuid);
  uuid_unparse (uuid, uuid_string);*/

  uuid_string = "TestOutputFeatureUUID";

  featureUuid = uuid_string;
  feature1.UUID = uuid_string;
  vector<tobbyapi_testreceiver::Feature> features;
  features.push_back(feature1);
  hello.request.Features = features;
  if (helloClient.call(hello))
  {
    ROS_INFO("Connection established, Status %u, Heartbeat %u",
             hello.response.Status, hello.response.Heartbeat);
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

void TestReceiver::gotData(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Status: %d", msg->data);
}

void TestReceiver::readConfigMsg(
    const tobbyapi_testreceiver::Config::ConstPtr& msg)
{
  ROS_INFO("Config Msg for: %s, should be connected to publisher %s with "
           "feature id %s to feature id %s, coefficient: %3.2f",
           msg->ReceiverUUID.c_str(), msg->SenderUUID.c_str(),
           msg->SenderFeatureUUID.c_str(), msg->ReceiverFeatureUUID.c_str(),
           msg->Coefficient);
  if (msg->ReceiverUUID == uuid && msg->ReceiverFeatureUUID == featureUuid)
  {
    ROS_INFO("This message is for me! :)");
    featureSub = nh->subscribe("TobbyAPI/" + msg->SenderUUID + "/" +
                                   msg->SenderFeatureUUID,
                               1000, &TestReceiver::gotData, this);
  }
}
