#include "enums.hpp"
#include "feature.hpp"
#include "testclient.hpp"
#include "tobbytestclient/feature.h"
//#include <uuid/uuid.h>

#define STANDARD_HEARTBEAT_INTERVAL 10000L
#define DEBUG

using namespace ros;
using namespace std;

TestClient::TestClient(NodeHandle *nh)
{
  this->nh = nh;
  helloClient = nh->serviceClient<tobbytestclient::hello>("TobbyAPI/Hello");
}

TestClient::~TestClient()
{
}

bool TestClient::Connect()
{
  tobbytestclient::hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.header = header;
  hello.request.product_name = "leckomio";
  /*uuid_t uuid;
  uuid_generate_random (uuid);
  char uuid_string[37];
  uuid_unparse (uuid, uuid_string);
  hello.request.uuid = uuid_string;*/
  hello.request.uuid = "thisisunique";
  Device_Type deviceType = Device_Type::Input_Device;
  hello.request.type = (unsigned short) deviceType;
  tobbytestclient::feature feature1;
  feature1.type = (unsigned short) Feature_Type::Camera;
  feature1.name = "Kamera vorne (hinten oder so)";
  feature1.id = 1;
  tobbytestclient::feature feature2;
  feature2.type = (unsigned short) Feature_Type::Camera;
  feature2.name = "Kamera hinten (wirklich hinten!)";
  feature2.id = 2;
  vector<tobbytestclient::feature> features;
  features.push_back(feature1);
  features.push_back(feature2);
  hello.request.features = features;
  if (helloClient.call(hello))
  {
    ROS_INFO("Connection established, Status %u, Heartbeat %u", hello.response.status, hello.response.heartbeat_interval);
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
  }
  return true;
}
