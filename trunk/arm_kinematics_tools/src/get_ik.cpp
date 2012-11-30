/*
 * This example shows how to call the get_ik inverse kinematics service.
 * 
 * The joint angles are read from the '/joint_states' topic and the
 * translation-quaternion pose is printed every 1 sec.
 * 
 * Has been tested with the KDLArmKinematicsPlugin (default plugin created by wizard),
 * the IKFastKinematicsPlugin, which can be generated using the python script,
 * and the example kinematics services from the 'arm_kinematics_test' package.
 * 
 * Based on this tutorial:
 * http://www.ros.org/wiki/pr2_kinematics/Tutorials/Tutorial%204
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
#include <kinematics_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

// Globals
unsigned int num_of_joints;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";

int main(int argc, char **argv) {
    ros::init (argc, argv, "get_ik_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    printf("\n\nTesting the IK service... \n\n");

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
    ros::service::waitForService(kinematics_service_name + "/get_ik_solver_info");
    ros::service::waitForService(kinematics_service_name + "/get_ik");

    // Set the planning scene to null, else IK calls will fail     
    if (ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.5) ) )
    {
        ros::ServiceClient set_planning_scene_diff_client = nh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
        arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
        arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
        if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
            ROS_WARN("Can't set planning scene, IK service calls will fail.");
        }
    }

    // Setup service clients to get kinematic solutions
    ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(kinematics_service_name + "/get_ik_solver_info");
    ros::ServiceClient ik_client = nh.serviceClient<kinematics_msgs::GetPositionIK>(kinematics_service_name + "/get_ik");

    // Define the service messages
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    // Get the list of joint names from the kinematics solver
    if ( query_client.call(request,response) )
    {
        for (unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
        {
            ROS_DEBUG("Joint: %d %s", i, response.kinematic_solver_info.joint_names[i].c_str() );
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

    // Define the kinematics service messages
    kinematics_msgs::GetPositionIK::Request  gpik_req;
    kinematics_msgs::GetPositionIK::Response gpik_res;

    // Timeout for finding a solution 
    gpik_req.timeout = ros::Duration(10.0); // 5

    // End effector frame (must be link name of last link or tool point frame)
    gpik_req.ik_request.ik_link_name = tip_link_name;

    // Reference frame for the desired pose, eg. base_link
    gpik_req.ik_request.pose_stamped.header.frame_id = root_link_name;

    // Translation-quaternion pose of end link
    gpik_req.ik_request.pose_stamped.pose.position.x = 0.0;
    gpik_req.ik_request.pose_stamped.pose.position.y = 0.0;
    gpik_req.ik_request.pose_stamped.pose.position.z = 0.0;
    gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
    gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
    gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
    gpik_req.ik_request.pose_stamped.pose.orientation.w = 0.0;

    // Seed the solver with a guess for the joint positions, using the mid-point of the joint limits
    // Note that if using a numerical solver, it should search for solutions using multiple random seeds,
    // or seeds based on the previous known joint positions, otherwise it will often fail.
    // Alternatively, create an IKFast plugin!
    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

    for (unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
        gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
    }

    tf::TransformListener listener;  
    tf::StampedTransform transform;

    // Wait for TF to get transform from tip->root link
    try {
        printf("Waiting for frame transform from %s ... \n\n", tip_link_name.c_str() );
        listener.waitForTransform(root_link_name, tip_link_name, ros::Time(0), ros::Duration(10.0) );
    } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
    }

    ros::Rate loop_rate(10); // run loop every 1 sec
    while ( ros::ok() )
    {
        // Get transform
        try {
            listener.lookupTransform(root_link_name, tip_link_name, ros::Time(0), transform);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        // Update IK request with pose
        gpik_req.ik_request.pose_stamped.pose.position.x = (float)transform.getOrigin().x();
        gpik_req.ik_request.pose_stamped.pose.position.y = transform.getOrigin().y();
        gpik_req.ik_request.pose_stamped.pose.position.z = transform.getOrigin().z();
        gpik_req.ik_request.pose_stamped.pose.orientation.x = transform.getRotation().getX();
        gpik_req.ik_request.pose_stamped.pose.orientation.y = transform.getRotation().getY();
        gpik_req.ik_request.pose_stamped.pose.orientation.z = transform.getRotation().getZ();
        gpik_req.ik_request.pose_stamped.pose.orientation.w = transform.getRotation().getW();

        printf("Current pose of %s frame, with respect to %s: \n", tip_link_name.c_str(), root_link_name.c_str() );
        printf("Translation:  %f x   %f y   %f z  \n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z() );
        printf("Quaternion:  %f i   %f j   %f k   %f w  \n\n", transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW() );

        // Call the IK service
        if (ik_client.call(gpik_req, gpik_res))
        {
            if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
            {
                printf("Joint angles:   \n");
                for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
                {
                    printf("%s %f   ", gpik_res.solution.joint_state.name[i].c_str(), gpik_res.solution.joint_state.position[i] );
                }
                printf("\n\n");
            }
            else
            {
                ROS_ERROR("Inverse kinematics failed");
            }
        }
        else
        {
            ROS_ERROR("Inverse kinematics service call failed");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}

