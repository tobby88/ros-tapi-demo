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
 * \file statemachineout.hpp
 * \ingroup tapi_demo_statemachine_output
 * \author Tobias Holst
 * \date 22 Sep 2016
 * \brief Declaration of the Tapi::StateMachineOut-class and definition of its member variables
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tapi_lib/tapi_lib.hpp"

namespace Tapi
{
/*!
 * \brief Simple state machine output. Just asks for all available states once and then prints information about the
 * current state
 * \author Tobias Holst
 * \version 1.2.1
 */
class StateMachineOut
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a StateMachineOut object to connect to/via \c tapi_core and get information about a Statemachine and
   * then print them on the console
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  StateMachineOut(ros::NodeHandle *nh);

  ~StateMachineOut();

private:
  // Private member functions

  /*!
   * \brief Called by a ros callback when a new current state has been got. Then it prints information about the current
   * state on the console.
   * \param msg The message waiting in the ros message queue where the current state is stored.
   */
  void current(const std_msgs::String::ConstPtr &msg);

  // Private member variables

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle *nh;

  /*!
   * \brief Doublepointer to a \c ros::ServiceClient to call the GetStateMachine service
   * \warning Has to be dereferenced once at first. Then check it whether its a null pointer. If not it can be used by
   * dereferencing it twice. If it's a null pointer when dereferencing once it means there is no connetion to a \c
   * ros::ServiceServer.
   */
  ros::ServiceClient **client;

  //! \c tapi_lib based ServiceClient object to create Tapi compliant serviceclients
  Tapi::ServiceClient *tclient;

  //! \c tapi_lib based Subscriber object to create Tapi compliant subscribers
  Tapi::Subscriber *tsub;
};
}
