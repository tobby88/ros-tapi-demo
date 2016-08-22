#ifndef TESTCLIENT_H
#define TESTCLIENT_H

#include "ros/ros.h"
#include "std_msgs/Header.h"

using namespace ros;
using namespace std;

class TestClient
{
public:
  // Constructor/Destructor
  TestClient(NodeHandle* nh);
  ~TestClient();

  // Public member functions
  bool Connect();

private:
  // Private member variables
  std_msgs::Header header;
  ServiceClient helloClient;
  NodeHandle* nh;
};

#endif // TESTCLIENT_H
