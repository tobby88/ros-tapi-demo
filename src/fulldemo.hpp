#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tapi_lib/tapi_lib.hpp"

namespace Tapi
{
class FullDemo
{
public:
  explicit FullDemo(ros::NodeHandle *nh);
  ~FullDemo();
  void Send();

private:
  bool dings;
  ros::NodeHandle *nh;
  ros::Publisher *pub;
  ros::ServiceClient **serviceclient;
  ros::AsyncSpinner *spinner;
  Tapi::ServiceClient *tclient;
  Tapi::Publisher *tpub;
  Tapi::ServiceServer *tserv;
  Tapi::Subscriber *tsub;

  void gotit(const std_msgs::Bool::ConstPtr &msg);
  bool hello(tapi_lib::Hello::Request &helloReq, tapi_lib::Hello::Response &helloResp);
};
}
