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
 * \defgroup tapi_demo_testsubscriber TestSubscriber
 * \file testsubscriber.cpp
 * \ingroup tapi_demo_testsubscriber
 * \author Tobias Holst
 * \date 28 Jul 2016
 * \brief Defintion of the Tapi::TestSubscriber-class
 */

#include "testsubscriber.hpp"
#include <uuid/uuid.h>

using namespace std;

namespace Tapi
{
TestSubscriber::TestSubscriber(ros::NodeHandle* nh) : nh(nh)
{
  // Create the Subscribers - first the "helper"
  tsub = new Tapi::Subscriber(nh, "TestSubscriber");

  // Now way one to create a topic - for the Bool topic. Throw away the coefficient when calling AddFeature since it's
  // not necessary for Bool topics anyway
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(std_msgs::Bool, 5, &TestSubscriber::gotDataBool);
  tsub->AddFeature(opt, "Bool Topic");

  // And way two to create a topic - for the Float topic. Here it's important to save the coefficient as well
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

/*!
 * \brief Main function of the TestSubscriber
 *
 * Main function of the TestSubscriber to initialize ROS, its NodeHandle and then create the TestSubscriber
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return 0 when exited correctly
 * \see Tapi::TestSubscriber
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestSubscriber");
  ros::NodeHandle nh;
  Tapi::TestSubscriber testSubscriber(&nh);
  ros::spin();

  return 0;
}
