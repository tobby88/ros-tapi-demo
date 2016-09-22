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
 * \defgroup tapi_demo_statemachine StateMachine
 * \file statemachine.cpp
 * \ingroup tapi_demo_statemachine
 * \author Tobias Holst
 * \date 22 Sep 2016
 * \brief Defintion of the Tapi::StateMachine-class
 */

#include "statemachine.hpp"
#include "std_msgs/String.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor
StateMachine::StateMachine(ros::NodeHandle *nh) : nh(nh)
{
  // Generate three demo states
  tapi_lib::State state1;
  state1.ID = "State1";
  state1.Name = "State 1";
  tapi_lib::Transition trans1;
  trans1.Condition = "Wait 1 second";
  trans1.ToID = "State2";
  state1.Transitions.push_back(trans1);
  states.push_back(state1);

  tapi_lib::State state2;
  state2.ID = "State2";
  state2.Name = "State 2";
  tapi_lib::Transition trans2;
  trans2.Condition = "Wait 1 second";
  trans2.ToID = "State3";
  state2.Transitions.push_back(trans2);
  states.push_back(state2);

  tapi_lib::State state3;
  state3.ID = "State3";
  state3.Name = "State 3 with two transistions";
  tapi_lib::Transition trans31;
  trans31.Condition = "Wait 1 second";
  trans31.ToID = "State1";
  tapi_lib::Transition trans32;
  trans32.Condition = "Never & Nothing";
  trans32.ToID = "State2";
  state3.Transitions.push_back(trans31);
  state3.Transitions.push_back(trans32);
  states.push_back(state3);

  // Create the current-state-publisher
  tpub = new Tapi::Publisher(nh, "StateMachine Test");
  pub = tpub->AddFeature<std_msgs::String>("Current State", 10);

  // Create the service to ask for the available states
  tserv = new Tapi::ServiceServer(nh, "StateMachine Test");
  tserv->AddFeature(ServiceServerOptionsForTapi(tapi_lib::GetStateMachine, &StateMachine::smCall), "StateMachine States");

  // Start the thread which changes the current state once a second
  changeStateThread = new thread(&StateMachine::changeState, this);
}

StateMachine::~StateMachine()
{
  delete tpub;
  delete tserv;
  changeStateThread->join();
  delete changeStateThread;
}

// Private member functions

void StateMachine::changeState()
{
  while (ros::ok())
  {
    // Wait a second, thand change the current state and publish it
    this_thread::sleep_for(chrono::milliseconds(1000));
    if (currentState == "State1")
      currentState = "State2";
    else if (currentState == "State2")
      currentState = "State3";
    else
      currentState = "State1";
    std_msgs::String msg;
    msg.data = currentState;
    pub->publish(msg);
  }
}

bool StateMachine::smCall(tapi_lib::GetStateMachine::Request &smReq, tapi_lib::GetStateMachine::Response &smResp)
{
  if (smReq.Get)
  {
    smResp.States = states;
    return true;
  }
  else
    return false;
}
}

/*!
 * \brief Main function of the StateMachine
 *
 * Main function of the StateMachine to initialize ROS, its NodeHandle and then create the StateMachine object
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return 0 when exited correctly
 * \see Tapi::StateMachine
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tapi_StateMachine");
  ros::NodeHandle nh;
  Tapi::StateMachine sm(&nh);
  while (ros::ok())
    ros::spinOnce();

  return 0;
}
