/*
 *  start_webots.cpp
 *
 *  Starts the Webots Simulator using a system() call, so Webots can be 
 *  started from a roslaunch file.
 *
 *  Date: September 2012
 *  Authors: David Butterworth
 *  
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ros/ros.h"
#include <iostream> // stringstream

/*
Define Webots executable name. 
The full path will come from the PATH environment variable.
Prefix with 'optirun' for Linux NVidia Bumblebee graphics driver

Webots command line arguments: 
   --minimize    
   --mode=run
   --mode=stop
   (http://www.cyberbotics.com/dvd/common/doc/webots/guide/section2.2.html)
*/

//#define WEBOTS_EXECUTABLE "webots"
#define WEBOTS_EXECUTABLE "optirun webots --mode=run"

//----------------------------------------------------------------------------//

int main(int argc, char **argv)
{
    // node name: webots
    ros::init(argc, argv, "webots");

    if ( argc != 2 ) 
    {
        // no additional arguments, start Webots with previously opened world
        ROS_INFO("Starting Webots simulator... \n");
        if (system(WEBOTS_EXECUTABLE)) {}
    } 
    else 
    {
        // additional arguments:
        // argv[0] is full path to this ROS Node
        // argv[1] is full path to world file
        // argv[2] is full path to ROS log file
        ROS_INFO("Starting Webots simulator... ");
        ROS_INFO("Loading world: %s \n", argv[1]);

        std::stringstream ss;
        ss << WEBOTS_EXECUTABLE << " " << argv[1];
        if (system( ss.str().c_str() )) {}
    }

    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}


