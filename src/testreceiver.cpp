#include "testreceiver.hpp"
#include "tobbyapi_msgs/Config.h"
#include "tobbyapi_msgs/Feature.h"
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

TestReceiver::TestReceiver(NodeHandle* nh)
{
  connected = false;
  this->nh = nh;
  helloClient = nh->serviceClient<tobbyapi_msgs::Hello>("TobbyAPI/HelloServ");
  configSub = nh->subscribe("TobbyAPI/Config", 1000,
                            &TestReceiver::readConfigMsg, this);
}

TestReceiver::~TestReceiver() {}

bool TestReceiver::Connect()
{
  tobbyapi_msgs::Hello hello;
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
  hello.request.DeviceType = tobbyapi_msgs::HelloRequest::Type_ReceiverDevice;
  tobbyapi_msgs::Feature feature1;
  feature1.FeatureType = tobbyapi_msgs::Feature::Type_Switch;
  feature1.Name = "Button Test";
  /*uuid_generate_random (uuid);
  uuid_unparse (uuid, uuid_string);*/

  uuid_string = "TestOutputFeatureUUID";

  featureUuid[0] = uuid_string;
  feature1.UUID = uuid_string;

  tobbyapi_msgs::Feature feature2;
  feature2.FeatureType = tobbyapi_msgs::Feature::Type_AnalogValue;
  feature2.Name = "Analog Test";
  feature2.UUID = "nochneUUID";
  featureUuid[1] = "nochneUUID";

  vector<tobbyapi_msgs::Feature> features;
  features.push_back(feature1);
  features.push_back(feature2);
  hello.request.Features = features;
  if (helloClient.call(hello))
  {
    if (hello.response.Status == tobbyapi_msgs::HelloResponse::StatusOK)
    {
      ROS_INFO("Connection established, Status OK, Heartbeat %u",
               hello.response.Heartbeat);
      connected = true;
    }
    else
    {
      ROS_INFO("Connection error, Heartbeat %u", hello.response.Heartbeat);
      connected = false;
    }
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
    connected = false;
  }
  return true;
}

void TestReceiver::gotDataBool(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Status: %d", msg->data);
}

void TestReceiver::gotDataFloat(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Status: %f", msg->data);
}

void TestReceiver::readConfigMsg(const tobbyapi_msgs::Config::ConstPtr& msg)
{
  ROS_INFO("Config Msg for: %s, should be connected to publisher %s with "
           "feature id %s to feature id %s, coefficient: %3.2f",
           msg->ReceiverUUID.c_str(), msg->SenderUUID.c_str(),
           msg->SenderFeatureUUID.c_str(), msg->ReceiverFeatureUUID.c_str(),
           msg->Coefficient);
  if (msg->ReceiverUUID == uuid && msg->ReceiverFeatureUUID == featureUuid[0])
  {
    ROS_INFO("This message is for me! :)");
    featureSub[0] = nh->subscribe("TobbyAPI/" + msg->SenderUUID + "/" +
                                   msg->SenderFeatureUUID,
                               1000, &TestReceiver::gotDataBool, this);
  }
  if (msg->ReceiverUUID == uuid && msg->ReceiverFeatureUUID == featureUuid[1])
  {
    ROS_INFO("This message is for me! :)");
    featureSub[1] = nh->subscribe("TobbyAPI/" + msg->SenderUUID + "/" +
                                   msg->SenderFeatureUUID,
                               1000, &TestReceiver::gotDataFloat, this);
  }
}
