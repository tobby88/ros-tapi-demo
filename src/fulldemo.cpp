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

/*!
 * \defgroup tapi_demo_fulldemo FullDemo
 * \file fulldemo.cpp
 * \ingroup tapi_demo_fulldemo
 * \author Tobias Holst
 * \date 06 Sep 2016
 * \brief Defintion of the Tapi::FullDemo-class
 */

#include "fulldemo.hpp"
#include <string>

using namespace std;

namespace Tapi
{
// Constructor/Desctructor

FullDemo::FullDemo(ros::NodeHandle* nh) : nh(nh)
{
  // Create the asynchronous spinner which sends an receives data in the backround and calls the callback routines.
  spinner = new ros::AsyncSpinner(1);
  spinner->start();

  // Create the Publisher helper object and the Publisher itself
  tpub = new Tapi::Publisher(nh, "Test");
  pub = tpub->AddFeature<std_msgs::Bool>("testbutton", 10);

  // Initial value
  turn = false;

  // Create the Subscriber
  tsub = new Tapi::Subscriber(nh, "Test2");
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(std_msgs::Bool, 10, &FullDemo::gotit);
  tsub->AddFeature(opt, "test");

  // Create the ServiceServer
  tserv = new Tapi::ServiceServer(nh, "TestServiceServer");
  ros::AdvertiseServiceOptions opt2;
  opt2 = ServiceServerOptionsForTapi(tapi_lib::Hello, &FullDemo::hello);
  tserv->AddFeature(opt2, "ServiceServer");

  // Create the ServiceClient
  tclient = new Tapi::ServiceClient(nh, "TestServiceClient");
  serviceclient = tclient->AddFeature<tapi_lib::Hello>("hallo");
}

FullDemo::~FullDemo()
{
  delete spinner;
  delete tpub;
  delete tsub;
  delete tserv;
  delete tclient;
}

void FullDemo::Send()
{
  // Create a Bool message and publish it
  std_msgs::Bool temp;
  temp.data = turn;
  pub->publish(temp);

  // Check if the ServiceClient is connected and if yes call it
  if (*serviceclient)
  {
    tapi_lib::Hello msg;
    if ((*serviceclient)->call(msg))
      ROS_INFO("Got response :)");
    else
      ROS_INFO("No response :(");
  }

  // Change the value of turn to send a different Bool message next time
  if (turn)
    turn = false;
  else
    turn = true;
}

void FullDemo::gotit(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}

bool FullDemo::hello(tapi_lib::Hello::Request& helloReq, tapi_lib::Hello::Response& helloResp)
{
  ROS_INFO("Service call!");
  helloResp.Status = tapi_lib::HelloResponse::StatusError;
  return true;
}
}

/*!
 * \brief Main function of the FullDemo
 *
 * Main function of the FullDemo to initialize ROS, its NodeHandle and then create the FullDemo object
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return 0 when exited correctly
 * \see Tapi::FullDemo
 */
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
