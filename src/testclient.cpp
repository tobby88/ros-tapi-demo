#include "testclient.hpp"
#include "enums.hpp"
#include "tobbytestclient/Feature.h"
#include "tobbytestclient/Hello.h"
#include <algorithm>
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

TestClient::TestClient(NodeHandle* nh)
{
  this->nh = nh;
  helloClient = nh->serviceClient<tobbytestclient::Hello>("TobbyAPI/HelloServ");
}

TestClient::~TestClient() {}

bool TestClient::Connect()
{
  tobbytestclient::Hello hello;
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

  DeviceType deviceType = DeviceType::SenderDevice;
  hello.request.DeviceType = (unsigned short)deviceType;

  tobbytestclient::Feature feature1;
  feature1.FeatureType = (unsigned short)FeatureType::Images;
  feature1.Name = "Kamera vorne (hinten oder so)";
  uuid_generate_random(uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature1.UUID = uuid_string;

  tobbytestclient::Feature feature2;
  feature2.FeatureType = (unsigned short)FeatureType::Images;
  feature2.Name = "Kamera hinten (wirklich hinten!)";
  uuid_generate_random(uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature2.UUID = uuid_string;

  tobbytestclient::Feature feature3;
  feature3.FeatureType = (unsigned short)FeatureType::Switch;
  feature3.Name = "irgendson Button";
  uuid_generate_random(uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature3.UUID = uuid_string;

  vector<tobbytestclient::Feature> features;
  features.push_back(feature1);
  features.push_back(feature2);
  features.push_back(feature3);
  hello.request.Features = features;
  if (helloClient.call(hello))
  {
    ROS_INFO("Connection established, Status %u, Heartbeat %u",
             hello.response.Status, hello.response.Heartbeat);
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
  }
  return true;
}
