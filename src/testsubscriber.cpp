/*******************************************************************************
*  This file is part of tapi_testsubscriber.                                   *
*                                                                              *
*  tapi_testsubscriber is free software: you can redistribute it and/or modify *
*  it under the terms of the GNU General Public License as published by        *
*  the Free Software Foundation, either version 3 of the License, or           *
*  (at your option) any later version.                                         *
*                                                                              *
*  tapi_testsubscriber is distributed in the hope that it will be useful,      *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of              *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
*  GNU General Public License for more details.                                *
*                                                                              *
*  You should have received a copy of the GNU General Public License           *
*  along with tapi_testsubscriber.  If not, see <http://www.gnu.org/licenses/>.*
*                                                                              *
*  Diese Datei ist Teil von tapi_testsubscriber.                               *
*                                                                              *
*  tapi_testsubscriber ist Freie Software: Sie können es unter den Bedingungen *
*  der GNU General Public License, wie von der Free Software Foundation,       *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                  *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.           *
*                                                                              *
*  tapi_testsubscriber wird in der Hoffnung, dass es nützlich sein wird, aber  *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite          *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK   *
*  Siehe die GNU General Public License für weitere Details.                   *
*                                                                              *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem   *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.  *
*******************************************************************************/

#include "testsubscriber.hpp"
#include <uuid/uuid.h>

using namespace std;

namespace Tapi
{
TestSubscriber::TestSubscriber(ros::NodeHandle* nh) : nh(nh)
{
  tsub = new Tapi::Subscriber(nh, "TestSubscriber");
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(std_msgs::Bool, 5, &TestSubscriber::gotDataBool);
  tsub->AddFeature(opt, "Bool Topic");
  coefficient =
      tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 5, &TestSubscriber::gotDataFloat), "Float Topic");
}

TestSubscriber::~TestSubscriber()
{
  delete tsub;
}

void TestSubscriber::gotDataBool(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
    ROS_INFO("Bool: true");
  else
    ROS_INFO("Bool: false");
}

void TestSubscriber::gotDataFloat(const std_msgs::Float64::ConstPtr& msg)
{
  double value;
  value = msg->data * *coefficient;
  ROS_INFO("Double/Float64: %f with coefficient %f", value, *coefficient);
}
}
