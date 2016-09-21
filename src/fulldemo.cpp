/******************************************************************************
 *  Copyright (C) 2016 by Tobias Holst                                        *
 *                                                                            *
 *  This file is part of tapi_demo.                                           *
 *                                                                            *
 *  tapi_demo is free software: you can redistribute it and/or modify         *
 *  it under the terms of the GNU General Public License as published by      *
 *  the Free Software Foundation, either version 3 of the License, or         *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_demo is distributed in the hope that it will be useful,              *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License         *
 *  along with tapi_demo.  If not, see <http://www.gnu.org/licenses/>.        *
 *                                                                            *
 *  Diese Datei ist Teil von tapi_demo.                                       *
 *                                                                            *
 *  tapi_demo ist Freie Software: Sie können es unter den Bedingungen         *
 *  der GNU General Public License, wie von der Free Software Foundation      *
 *  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_demo wird in der Hoffnung, dass es nützlich sein wird, aber          *
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite        *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

#include "fulldemo.hpp"
#include <string>

using namespace std;

namespace Tapi
{
// Constructor/Desctructor

FullDemo::FullDemo(ros::NodeHandle* nh) : nh(nh)
{
  spinner = new ros::AsyncSpinner(1);
  spinner->start();
  tpub = new Tapi::Publisher(nh, "Test");
  pub = tpub->AddFeature<std_msgs::Bool>("testbutton", 10);
  turn = false;
  tsub = new Tapi::Subscriber(nh, "Test2");
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(std_msgs::Bool, 10, &FullDemo::gotit);
  tsub->AddFeature(opt, "test");

  tserv = new Tapi::ServiceServer(nh, "TestServiceServer");
  ros::AdvertiseServiceOptions opt2;
  opt2 = ServiceServerOptionsForTapi(tapi_lib::Hello, &FullDemo::hello);
  tserv->AddFeature(opt2, "ServiceServer");

  tclient = new Tapi::ServiceClient(nh, "TestServiceClient");
  serviceclient = tclient->AddFeature<tapi_lib::Hello>("hallo");
  if (*serviceclient)
    ROS_INFO("There is a service client! :)");
  else
    ROS_INFO("Service client is a null pointer...");
}

FullDemo::~FullDemo()
{
  // TODO: Delete objects
}

void FullDemo::Send()
{
  if (turn)
    turn = false;
  else
    turn = true;
  std_msgs::Bool blubb;
  blubb.data = turn;
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

void FullDemo::gotit(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}

bool FullDemo::hello(tapi_lib::Hello::Request& helloReq, tapi_lib::Hello::Response& helloResp)
{
  ROS_INFO("Service call!");
  helloResp.Status = tapi_lib::HelloResponse::StatusError;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_FullDemo");
  ros::NodeHandle nh;
  Tapi::FullDemo test(&nh);
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    test.Send();
    loop_rate.sleep();
  }

  return 0;
}
