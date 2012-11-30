/*
 * This example shows how to call the get_fk forward kinematics service.
 *
 * The joint angles are read from the '/joint_states' topic and the
 * translation-quaternion pose is printed every 1 sec.
 *
 * Has been tested with the KDLArmKinematicsPlugin (default plugin created by wizard),
 * the IKFastKinematicsPlugin, which can be generated using the python script,
 * and the example kinematics services from the 'arm_kinematics_tools' package.
 *
 * Based on this tutorial:
 * http://www.ros.org/wiki/pr2_kinematics/Tutorials/Tutorial%203
 *
 *
 * Tested on ROS Fuerte
 *
 * Note: you should apply the patch for arm_kinematics_constraint_aware 
 * mentioned here: https://code.ros.org/trac/ros-pkg/ticket/5586
 * This fixes a bug in the FK routine, and also allows you to switch from using
 * TF by default for forward kinematics, to the FK routine from this plugin.
 *
 *
 * Author: David Butterworth, KAIST
 * Date: November 2012
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

#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>
#include "sensor_msgs/JointState.h"

// Globals, written by callback
unsigned int num_of_joints;
std::vector<double> joint_angles;
std::vector<std::string> joint_names;
bool started = false;

void callbackJointStates(const sensor_msgs::JointState::ConstPtr& msg);

int main(int argc, char **argv) {
    ros::init (argc, argv, "get_fk_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    printf("\n\nTesting the FK service... \n\n");

    // The name of the kinematics service should be passed with the launch of this node
    std::string kinematics_service_name;
    pnh.param<std::string>("kinematics_service_name", kinematics_service_name, "kinematics_service_not_specified");

    // The name of the root and tip links should be published by the kinematics service
    std::string root_link_name;
    nh.param<std::string>(kinematics_service_name + "/manipulator/root_name", root_link_name, "root_link could not be read");
    std::string tip_link_name;
    nh.param<std::string>(kinematics_service_name + "/manipulator/tip_name", tip_link_name, "tip_link could not be read");

    printf("Using: \n");
    printf("   Kinematics service path: %s \n", kinematics_service_name.c_str() );
    printf("   root_link: %s   tip_link: %s \n", root_link_name.c_str(), tip_link_name.c_str() );
    printf("\n");

    // Make sure the inverse kinematics service is running
    ros::service::waitForService(kinematics_service_name + "/get_fk_solver_info");
    ros::service::waitForService(kinematics_service_name + "/get_fk");

    // Setup service clients to get kinematic solutions
    ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(kinematics_service_name + "/get_fk_solver_info");
    ros::ServiceClient fk_client = nh.serviceClient<kinematics_msgs::GetPositionFK>(kinematics_service_name + "/get_fk");

    // Define the service messages
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    // Get the list of joint names from the kinematics solver
    if(query_client.call(request,response))
    {
        for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
        {
            ROS_DEBUG("Joint: %d %s", i, response.kinematic_solver_info.joint_names[i].c_str());
        }
    }
    else
    {
        ROS_ERROR("Could not call query service");
        ros::shutdown();
        exit(1);
    }

    // Read number of joints from solver's response
    num_of_joints = response.kinematic_solver_info.joint_names.size();

    // Subscribe to '/joint_states' topic and initialize vectors of joint angles and joint names
    ros::Subscriber subscriberJointStates;
    subscriberJointStates = nh.subscribe("joint_states", 3, callbackJointStates);
    joint_angles.resize(num_of_joints);
    joint_names.resize(num_of_joints);
    for (unsigned int i=0; i < num_of_joints; i++)
    {
        joint_angles[i] = 0.0;
    }
    joint_names = response.kinematic_solver_info.joint_names;

    // Define the kinematics service messages
    kinematics_msgs::GetPositionFK::Request  fk_request;
    kinematics_msgs::GetPositionFK::Response fk_response;

    // Reference frame for the desired forward kinematic pose
    fk_request.header.frame_id = root_link_name;

    // End effector frame
    fk_request.fk_link_names.resize(1);
    fk_request.fk_link_names[0] = tip_link_name;

    fk_request.robot_state.joint_state.position.resize(num_of_joints);
    fk_request.robot_state.joint_state.name = response.kinematic_solver_info.joint_names;

    // wait until we get joint names from callback
    while (started == false) {
        ros::spinOnce();
    } 

    ros::Rate loop_rate(10); // run loop every 1 sec
    while ( ros::ok() )
    {
        // Update service message with joint angles
        for (unsigned int i=0; i < num_of_joints; i++)
        {
            fk_request.robot_state.joint_state.position[i] = joint_angles[i];
        }

        // Call the FK service
        if (fk_client.call(fk_request, fk_response))
        {
            if (fk_response.error_code.val == fk_response.error_code.SUCCESS)
            {
                for (unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
                {
                    ROS_INFO_STREAM("ALink    : " << fk_response.fk_link_names[i].c_str() );
                    ROS_INFO_STREAM("Position: " << 
                    fk_response.pose_stamped[i].pose.position.x << "," <<  
                    fk_response.pose_stamped[i].pose.position.y << "," << 
                    fk_response.pose_stamped[i].pose.position.z);
                    ROS_INFO("Orientation:  %f i   %f j   %f k   %f   \n",
                    fk_response.pose_stamped[i].pose.orientation.x,
                    fk_response.pose_stamped[i].pose.orientation.y,
                    fk_response.pose_stamped[i].pose.orientation.z,
                    fk_response.pose_stamped[i].pose.orientation.w);
                } 
            }
            else
            {
              ROS_ERROR("Forward kinematics failed");
            }
        }
        else
        {
            ROS_ERROR("Forward kinematics service call failed");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}

// Get joint angles from '/joint_states' topic and store in vector
void callbackJointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
    started = true;
    // Need to write joint names, because 'joint_state_publisher' messes up the order
    unsigned int joint_list_index;
    for (unsigned int i=0; i < num_of_joints; i++)
    {
        joint_list_index = std::find(joint_names.begin(),joint_names.end(), std::string( msg->name[i] )) - joint_names.begin();
        joint_angles[joint_list_index] = msg->position[i];
    }
}

