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
 * \file statemachine.hpp
 * \ingroup tapi_demo_statemachine
 * \author Tobias Holst
 * \date 22 Sep 2016
 * \brief Declaration of the Tapi::StateMachine-class and definition of its member variables
 */

#include <chrono>
#include <thread>
#include "ros/ros.h"
#include "tapi_lib/tapi_lib.hpp"

namespace Tapi
{
/*!
 * \brief Simple state machine. Does nothing but publishing it's states.
 * \author Tobias Holst
 * \version 1.1.1
 */
class StateMachine
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a StateMachine object to connect to \c tapi_core and publish its states and its current state
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  StateMachine(ros::NodeHandle *nh);

  //! Delete all Tapi-objects and free the memory
  ~StateMachine();

private:
  // Private member variables

  /*!
   * \brief Thread to change the state once a second
   * \see Tapi::StateMachine::changeState
   */
  std::thread *changeStateThread;

  //! Store the id of the current state
  std::string currentState;

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle *nh;

  /*!
   * \brief Pointer to a publisher to actually publish the data. Used by \c changeState.
   * \see Tapi::FullDemo::changeState
   */
  ros::Publisher *pub;

  /*!
   * \brief Store the available states to respont to the service calls when asked for available states
   * \see Tapi::StateMachine::smCall
   */
  std::vector<tapi_lib::State> states;

  //! \c tapi_lib based Publisher object to create Tapi compliant publishers
  Tapi::Publisher *tpub;

  //! \c tapi_lib based ServiceServer object to create Tapi compliant serviceservers
  Tapi::ServiceServer *tserv;

  // Private member functions

  //! Wait a second, thand change the current state and publish it
  void changeState();

  /*!
   * \brief alled by a ros callback when a new call on its \c GetStateMachine service has been got. Responds with the
   * available states
   * \param smReq Request of the service call (should be a \c true to get a response)
   * \param smResp Response of the service call, includes all available states
   * \return \c true if the request was ok and a response was sent, \c false if not
   */
  bool smCall(tapi_lib::GetStateMachine::Request &smReq, tapi_lib::GetStateMachine::Response &smResp);
};
}
