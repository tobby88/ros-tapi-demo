#include "fulldemo.hpp"
#include <string>

using namespace std;

namespace Tapi
{
// Constructor/Desctructor

Test::Test(ros::NodeHandle* nh) : nh(nh)
{
  spinner = new ros::AsyncSpinner(1);
  spinner->start();
  tpub = new Tapi::Publisher(nh, "Test");
  pub = tpub->AddFeature<std_msgs::Bool>("testbutton", 10);
  dings = false;
  tsub = new Tapi::Subscriber(nh, "Test2");
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(std_msgs::Bool, 10, &Test::gotit);
  tsub->AddFeature(opt, "test");

  tserv = new Tapi::ServiceServer(nh, "TestServiceServer");
  ros::AdvertiseServiceOptions opt2;
  opt2 = ServiceServerOptionsForTapi(tapi_lib::Hello, &Test::hello);
  tserv->AddFeature(opt2, "ServiceServer");

  tclient = new Tapi::ServiceClient(nh, "TestServiceClient");
  serviceclient = tclient->AddFeature<tapi_lib::Hello>("hallo");
  if (*serviceclient)
    ROS_INFO("There is a service client! :)");
  else
    ROS_INFO("Service client is a null pointer...");
}

Test::~Test()
{
  // TODO: Delete objects
}

void Test::Send()
{
  if (dings)
    dings = false;
  else
    dings = true;
  std_msgs::Bool blubb;
  blubb.data = dings;
  pub->publish(blubb);
  if (*serviceclient)
  {
    tapi_lib::Hello msg;
    if ((*serviceclient)->call(msg))
      ROS_INFO("Got response :)");
    else
      ROS_INFO("No response :(");
  }
}

void Test::gotit(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}

bool Test::hello(tapi_lib::Hello::Request& helloReq, tapi_lib::Hello::Response& helloResp)
{
  ROS_INFO("Service call!");
  helloResp.Status = tapi_lib::HelloResponse::StatusError;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TesterNode");
  ros::NodeHandle nh;
  Tapi::Test test(&nh);
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    test.Send();
    loop_rate.sleep();
  }

  return 0;
}
