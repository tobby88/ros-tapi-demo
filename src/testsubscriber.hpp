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
 * \file testsubscriber.hpp
 * \ingroup tapi_demo_testsubscriber
 * \author Tobias Holst
 * \date 28 Jul 2016
 * \brief Declaration of the Tapi::TestSubscriber-class and definition of its member variables
 */

#ifndef TESTSUBSCRIBER_H
#define TESTSUBSCRIBER_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "tapi_lib/subscriber.hpp"

namespace Tapi
{
/*!
 * \brief Connect to tapi_core with two subscribers and just prints the data on the console when connected. Has a \c
 * Bool and a \c Float64 topic.
 * \author Tobias Holst
 * \version 1.0.1
 */
class TestSubscriber
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a TestSubscriber object to connect to tapi_core and print the data on the console when connected. Has
   * a \c Bool and a \c Float64 topic.
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  TestSubscriber(ros::NodeHandle* nh);

  /*!
   * \brief Delete the Tapi::Subscriber object
   * \c Tapi::Subscriber in \c tapi_lib package
   */
  ~TestSubscriber();

private:
  // Private member variables

  /*!
   * \brief coefficient to multiply all Float64 values with
   * \see Tapi::TestSubscriber::gotDataFloat
   */
  double* coefficient;

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle* nh;

  //! \c tapi_lib based Subscriber object to create Tapi compliant Subscribers
  Tapi::Subscriber* tsub;

  // Private member functions

  /*!
   * \brief Called by a ros callback when a new \c Bool has been got. Then it prints its state on the console.
   * \param msg The message waiting in the ros message queue where the \c Bool is stored
   */
  void gotDataBool(const std_msgs::Bool::ConstPtr& msg);

  /*!
   * \brief Called by a ros callback when a new \c Float64 has been got. Then it multiplies its value with the
   * configured coefficient (configured by the connection) and prints the result on the console.
   * \param msg The message waiting in the ros message queue where the \c Float64 is stored
   */
  void gotDataFloat(const std_msgs::Float64::ConstPtr& msg);
};
}

#endif  // TESTSUBSCRIBER_H
