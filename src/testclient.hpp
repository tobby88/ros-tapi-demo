#ifndef TESTCLIENT_H
#define TESTCLIENT_H

#include "ros/ros.h"
#include "std_msgs/Header.h"

using namespace ros;
using namespace std;

class TestClient
{
private:
  NodeHandle *nh;
  ServiceClient helloClient;
  std_msgs::Header header;

public:
  TestClient(NodeHandle *nh);
  ~TestClient();
  bool Connect();
};

#endif // TESTCLIENT_H
