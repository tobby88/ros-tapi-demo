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
 * \defgroup tapi_demo_statemachine_output StateMachineOutput
 * \file statemachineout.cpp
 * \ingroup tapi_demo_statemachine_output
 * \author Tobias Holst
 * \date 22 Sep 2016
 * \brief Defintion of the Tapi::StateMachineOut-class
 */

#include "statemachineout.hpp"
#include <map>

using namespace std;

namespace Tapi
{
// Constructor/Destrurctor
StateMachineOut::StateMachineOut(ros::NodeHandle *nh) : nh(nh)
{
  // Create ServiceClient
  tclient = new Tapi::ServiceClient(nh, "StateMachine Output");
  client = tclient->AddFeature<tapi_lib::GetStateMachine>("Get StateMachine States");

  // Create the subscriber to get the current state
  tsub = new Tapi::Subscriber(nh, "StateMachine Output");
  tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::String, 10, &StateMachineOut::current), "Get Current State");
}

StateMachineOut::~StateMachineOut()
{
  delete tsub;
  delete tclient;
}

// Private member functions

void StateMachineOut::current(const std_msgs::String::ConstPtr &msg)
{
  // Check whether the service is connected, then fetch the data about the states
  if (!*client)
    return;
  tapi_lib::GetStateMachine getter;
  getter.request.Get = true;
  if ((*client)->call(getter))
  {
    // Store the states in a map
    map<string, tapi_lib::State> states;
    for (auto it = getter.response.States.begin(); it != getter.response.States.end(); ++it)
    {
      tapi_lib::State temp;
      temp.ID = it->ID;
      temp.Name = it->Name;
      temp.Transitions = it->Transitions;
      states.emplace(temp.ID, temp);
    }

    // See if the current state is in the map
    if (states.count(msg->data) > 0)
    {
      // Print information about the current state and its transitions on the console
      tapi_lib::State temp;
      temp = states.at(msg->data);
      string name, id, transitions = "";
      name = temp.Name;
      id = temp.ID;
      for (auto it = temp.Transitions.begin(); it != temp.Transitions.end(); ++it)
      {
        if (states.count(it->ToID) > 0)
        {
          string toName = states.at(it->ToID).Name;
          transitions += ", can switch to state: " + toName + ", when: " + it->Condition;
        }
        else
          ROS_ERROR("Wrong transition");
      }
      ROS_INFO("In state %s, id %s%s", name.c_str(), id.c_str(), transitions.c_str());
    }
    else
      ROS_ERROR("Wrong state");
  }
}
}

/*!
 * \brief Main function of the StateMachineOut
 *
 * Main function of the StateMachineOut to initialize ROS, its NodeHandle and then create the StateMachineOut object
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return 0 when exited correctly
 * \see Tapi::StateMachineOut
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tapi_StateMachineOutput");
  ros::NodeHandle nh;
  Tapi::StateMachineOut sm(&nh);
  ros::spin();

  return 0;
}
