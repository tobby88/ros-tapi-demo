/******************************************************************************
*  Copyright (C) 2016 by Tobias Holst                                         *
*                                                                             *
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

#include "ros/ros.h"
#include "testpublisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_TestPublisher");
  ros::NodeHandle nh;
  Tapi::TestPublisher testPublisher(&nh);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    testPublisher.SendTest();
    ros::spinOnce();
  }

  return 0;
}
