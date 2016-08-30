#include "testsender.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TestSender::TestSender(ros::NodeHandle* nh) : nh(nh)
{
  tpub = new Tapi::Publisher(nh, "TestSender");
}

TestSender::~TestSender()
{
}

// Public member functions

{
  else
}
}
