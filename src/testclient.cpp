#include "enums.hpp"
#include "feature.hpp"
#include "testclient.hpp"
#include "tobbytestclient/feature.h"

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
  return true;
}
