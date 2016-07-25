#ifndef TESTCLIENT_H
#define TESTCLIENT_H

#include "tobbytestclient/hello.h"
#include "ros/ros.h"
#include <unordered_map>

using namespace ros;
using namespace std;

class TestClient
{
private:
  NodeHandle *nh;
  ServiceClient helloClient;

public:
  TestClient(NodeHandle *nh);
  ~TestClient();
  bool Connect();
};

#endif // TESTCLIENT_H
