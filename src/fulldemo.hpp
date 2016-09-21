#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tapi_lib/Hello.h"
#include "tapi_lib/publisher.hpp"
#include "tapi_lib/serviceclient.hpp"
#include "tapi_lib/serviceserver.hpp"
#include "tapi_lib/subscriber.hpp"

class Test
{
public:
  explicit Test(ros::NodeHandle *nh);
  ~Test();
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
