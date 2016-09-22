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
 * \file testpublisher.hpp
 * \ingroup tapi_demo_testpublisher
 * \author Tobias Holst
 * \date 26 Jul 2016
 * \brief Declaration of the Tapi::TestPublisher-class and definition of its member variables
 */

#ifndef TESTPUBLISHER_H
#define TESTPUBLISHER_H

#include "ros/ros.h"
#include "tapi_lib/publisher.hpp"

namespace Tapi
{
/*!
 * \brief Connect to tapi_core with two publishers who just send some test data once a second. Sends out a \c Bool and a
 * \c Float64 message.
 * \author Tobias Holst
 * \version 1.0.1
 */
class TestPublisher
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a TestPublisher object to connect to tapi_core and send test data once a second. Sends out a \c Bool
   * and a \c Float64 message.
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   */
  TestPublisher(ros::NodeHandle* nh);

  /*!
   * \brief Delete the Tapi::Publisher object and stop the \c ros::AsyncSpinner
   * \see \c Tapi::Publisher in the \c tapi_lib package
   */
  ~TestPublisher();

  // Public member functions

  //! Send the test data
  void SendTest();

private:
  // Private member variables

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle* nh;

  /*!
   * \brief Store the current number which will be sent (and incremented) by SendTest
   * \see Tapi::TestPublisher::SendTest
   */
  double number;

  /*!
   * \brief Pointer to two publishers to actually publish the data. Used by SendTest.
   * \see Tapi::TestPublisher::SendTest
   */
  ros::Publisher* pub[2];

  /*!
   * \brief Spinner to acutally sent the data in background after SendTest is called
   * \see Tapi::TestPublisher::SendTest
   */
  ros::AsyncSpinner* spinner;

  //! \c tapi_lib based Subscriber object to create Tapi compliant Subscribers
  Tapi::Publisher* tpub;

  /*!
   * \brief Stores true or false, changes every time SendTest is called. Its value is also sent by SendTest
   * \see Tapi::TestPublisher::SendTest
   */
  bool truefalse;
};
}

#endif  // TESTPUBLISHER_H
