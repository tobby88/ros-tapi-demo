/******************************************************************************
*  This file is part of tapi_testpublisher.                                   *
*                                                                             *
*  tapi_testpublisher is free software: you can redistribute it and/or modify *
*  it under the terms of the GNU General Public License as published by       *
*  the Free Software Foundation, either version 3 of the License, or          *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_testpublisher is distributed in the hope that it will be useful,      *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License          *
*  along with tapi_testpublisher.  If not, see <http://www.gnu.org/licenses/>.*
*                                                                             *
*  Diese Datei ist Teil von tapi_testpublisher.                               *
*                                                                             *
*  tapi_testpublisher ist Freie Software: Sie können es unter den Bedingungen *
*  der GNU General Public License, wie von der Free Software Foundation,      *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                 *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_testpublisher wird in der Hoffnung, dass es nützlich sein wird, aber  *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite         *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

#include "testpublisher.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

TestPublisher::TestPublisher(ros::NodeHandle* nh) : nh(nh)
{
  tpub = new Tapi::Publisher(nh, "TestPublisher");
  pub[0] = tpub->AddFeature<std_msgs::Bool>("Button", 10);
  pub[1] = tpub->AddFeature<std_msgs::Float64>("AnalogValue", 10);
  number = 0.0;
  truefalse = false;
}

TestPublisher::~TestPublisher()
{
  delete tpub;
}

// Public member functions

void TestPublisher::SendTest()
{
  std_msgs::Bool m1;
  std_msgs::Float64 m2;
  m1.data = truefalse;
  m2.data = number;
  pub[0]->publish(m1);
  pub[1]->publish(m2);
  if (truefalse)
    truefalse = false;
  else
    truefalse = true;
  number += 1.0001;
}
}
