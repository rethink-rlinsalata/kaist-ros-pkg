/*
 *  start_webots.cpp
 *
 *  Starts the Webots Simulator using a system() call, so Webots can be 
 *  started from a roslaunch file.
 *
 *  Date: September 2012
 *  Authors: David Butterworth
 *  
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#define WEBOTS_EXECUTABLE "webots --mode=run"
//#define WEBOTS_EXECUTABLE "optirun webots --mode=run"

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

    // No need to ros::spin()
    // When we quit Webots, ROS will register
    // that the webots node is shutting down.

    return 0;
}


