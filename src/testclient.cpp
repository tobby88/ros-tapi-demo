#include "enums.hpp"
#include "testclient.hpp"
#include "tobbytestclient/Feature.h"
#include "tobbytestclient/Hello.h"
#include <algorithm>
#include <uuid/uuid.h>

using namespace ros;
using namespace std;

TestClient::TestClient(NodeHandle *nh)
{
  this->nh = nh;
  helloClient = nh->serviceClient<tobbytestclient::Hello>("TobbyAPI/HelloServ");
}

TestClient::~TestClient()
{
}

bool TestClient::Connect()
{
  tobbytestclient::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.header = header;
  hello.request.name = "leckomio";
  uuid_t uuid;
  uuid_generate_random (uuid);
  char uuid_array[37];
  uuid_unparse(uuid, uuid_array);
  string uuid_string(uuid_array);
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  hello.request.uuid = uuid_string;
  Device_Type deviceType = Device_Type::Input_Device;
  hello.request.type = (unsigned short) deviceType;
  tobbytestclient::Feature feature1;
  feature1.type = (unsigned short) Feature_Type::Camera;
  feature1.name = "Kamera vorne (hinten oder so)";
  uuid_generate_random (uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature1.uuid = uuid_string;
  tobbytestclient::Feature feature2;
  feature2.type = (unsigned short) Feature_Type::Camera;
  feature2.name = "Kamera hinten (wirklich hinten!)";
  uuid_generate_random (uuid);
  uuid_unparse(uuid, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  feature2.uuid = uuid_string;
  vector<tobbytestclient::Feature> features;
  features.push_back(feature1);
  features.push_back(feature2);
  hello.request.features = features;
  if (helloClient.call(hello))
  {
    ROS_INFO("Connection established, Status %u, Heartbeat %u", hello.response.status, hello.response.heartbeat);
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    return false;
  }
  return true;
}
