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
 * \file fulldemo.hpp
 * \ingroup tapi_demo_fulldemo
 * \author Tobias Holst
 * \date 06 Sep 2016
 * \brief Declaration of the Tapi::FullDemo-class and definition of its member variables
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tapi_lib/tapi_lib.hpp"

namespace Tapi
{
/*!
 * \brief Connect to \c tapi_core with a \c Bool subscriber, a \c Bool publisher, a \c Hello \c ServiceServer and a \c
 * Hello \c ServiceClient. Sends/Calls twice a second.
 * \see \c Hello.srv in the \c tapi_lib package
 * \author Tobias Holst
 * \version 1.0.1
 */
class FullDemo
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a FullDemo object to connect to \c tapi_core and send test data twice a second.
   *
   * Sends out a \c Bool message and a \c Hello call. Also receives a \c Bool topic and prints its data on reception and
   * a \c Hello call which just prints if it was successfully called.
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  explicit FullDemo(ros::NodeHandle *nh);

  //! Delete all Tapi-objects and the ros::AsyncSpinner and free the memory
  ~FullDemo();

  // Public member functions

  //! Send the test data
  void Send();

private:
  // Private member variables

  /*!
   * \brief Saves true or false and changes its state every time Send is called. Its data is sent on the call of Send.
   * \see Tapi::FullDemo:Send
   */
  bool turn;

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle *nh;

  /*!
   * \brief Pointer to a publishes to actually publish the data. Used by Send.
   * \see Tapi::FullDemo::Send
   */
  ros::Publisher *pub;

  /*!
   * \brief Doublepointer to a \c ros::ServiceClient to call the Hello service testwise
   * \warning Has to be dereferenced once at first. Then check it whether its a null pointer. If not it can be used by
   * dereferencing it twice. If it's a null pointer when dereferencing once it means there is no connetion to a \c
   * ros::ServiceServer.
   */
  ros::ServiceClient **serviceclient;

  /*!
   * \brief Spinner to acutally sent the data in background after Send is called
   * \see Tapi::FullDemo::Send
   */
  ros::AsyncSpinner *spinner;

  //! \c tapi_lib based ServiceClient object to create Tapi compliant serviceclients
  Tapi::ServiceClient *tclient;

  //! \c tapi_lib based Publisher object to create Tapi compliant publishers
  Tapi::Publisher *tpub;

  //! \c tapi_lib based ServiceServer object to create Tapi compliant serviceservers
  Tapi::ServiceServer *tserv;

  //! \c tapi_lib based Subscriber object to create Tapi compliant subscribers
  Tapi::Subscriber *tsub;

  // Private member functions

  /*!
   * \brief Called by a ros callback when a new \c Bool has been got. Then it prints its state on the console.
   * \param msg The message waiting in the ros message queue where the \c Bool is stored
   */
  void gotit(const std_msgs::Bool::ConstPtr &msg);

  /*!
   * \brief Called by a ros callback when a new call on its Hello service has been got. Then it prints on the console
   * that there was a successfull call.
   * \param helloReq The request of the service call, not used.
   * \param helloResp The response of the service call, always a STATUS_ERROR since this is not a real Hello service.
   * \return Always \c true
   * \see \c Hello.srv in the \c tapi_lib package
   */
  bool hello(tapi_lib::Hello::Request &helloReq, tapi_lib::Hello::Response &helloResp);
};
}
