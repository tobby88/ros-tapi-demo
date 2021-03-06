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
 * \defgroup tapi_demo_testpublisher TestPublisher
 * \file testpublisher.cpp
 * \ingroup tapi_demo_testpublisher
 * \author Tobias Holst
 * \date 26 Jul 2016
 * \brief Defintion of the Tapi::TestPublisher-class
 */

#include "testpublisher.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TestPublisher::TestPublisher(ros::NodeHandle* nh) : nh(nh)
{
  // Create the helper to create Tapi-compliant publishers and then create the two publishers
  tpub = new Tapi::Publisher(nh, "TestPublisher");
  pub[0] = tpub->AddFeature<std_msgs::Bool>("Button", 10);
  pub[1] = tpub->AddFeature<std_msgs::Float64>("AnalogValue", 10);

  // Initial values vor SendTest
  number = 0.0;
  truefalse = false;

  // Create an asynchronous spinner which will actually send the data of SendTest in background
  spinner = new ros::AsyncSpinner(1);
  spinner->start();
}

TestPublisher::~TestPublisher()
{
  spinner->stop();
  delete spinner;
  delete tpub;
}

// Public member functions

void TestPublisher::SendTest()
{
  // Create the two messages with the current data in this class
  std_msgs::Bool m1;
  std_msgs::Float64 m2;
  m1.data = truefalse;
  m2.data = number;

  // Publish the messages
  pub[0]->publish(m1);
  pub[1]->publish(m2);

  // Now change the values of the two variables used for message generation
  if (truefalse)
    truefalse = false;
  else
    truefalse = true;
  number += 1.0001;
}
}

/*!
 * \brief Main function of the TestPublisher
 *
 * Main function of the TestPublisher to initialize ROS, its NodeHandle and then create the TestPublisher
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return 0 when exited correctly
 * \see Tapi::TestPublisher
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestPublisher");
  ros::NodeHandle nh;
  Tapi::TestPublisher testPublisher(&nh);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    loop_rate.sleep();
    testPublisher.SendTest();
  }

  return 0;
}
