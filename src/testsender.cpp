#include "testsender.hpp"
#include <uuid/uuid.h>
#include <algorithm>
#include "tobbyapi_msgs/Feature.h"
#include "tobbyapi_msgs/Hello.h"

using namespace ros;
using namespace std;

// Constructor/Destructor

TestSender::TestSender(NodeHandle* nh)
{
  this->nh = nh;
  helloClient = nh->serviceClient<tobbyapi_msgs::Hello>("TobbyAPI/HelloServ");
}

TestSender::~TestSender()
{
}

// Public member functions

bool TestSender::Connect()
{
  tobbyapi_msgs::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.Header = header;
  hello.request.Name = "leckomio";

  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_array[37];
  uuid_unparse(uuid, uuid_array);
  string uuid_string(uuid_array);
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  hello.request.UUID = uuid_string;
  hello.request.DeviceType = tobbyapi_msgs::Hello::Request::Type_SenderDevice;

  tobbyapi_msgs::Feature feature1;
  feature1.FeatureType = tobbyapi_msgs::Feature::Type_Images;
  feature1.Name = "Kamera vorne (hinten oder so)";
  uuid_generate_random(uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature1.UUID = uuid_string;

  tobbyapi_msgs::Feature feature2;
  feature2.FeatureType = tobbyapi_msgs::Feature::Type_Images;
  feature2.Name = "Kamera hinten (wirklich hinten!)";
  uuid_generate_random(uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature2.UUID = uuid_string;

  tobbyapi_msgs::Feature feature3;
  feature3.FeatureType = tobbyapi_msgs::Feature::Type_Switch;
  feature3.Name = "irgendson Button";
  uuid_generate_random(uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature3.UUID = uuid_string;

  vector<tobbyapi_msgs::Feature> features;
  features.push_back(feature1);
  features.push_back(feature2);
  features.push_back(feature3);
  hello.request.Features = features;
  if (helloClient.call(hello))
  {
    if (hello.response.Status == tobbyapi_msgs::Hello::Response::StatusOK)
      ROS_INFO("Connection established, Status OK, Heartbeat %u", hello.response.Heartbeat);
    else
      ROS_INFO("Connection error, Heartbeat %u", hello.response.Heartbeat);
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
  }
  return true;
}
