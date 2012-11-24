
// This example shows how to call the get_fk forward kinematics service.
// The joint angles are read from the '/joint_states' topic and the
// translation-quaternion pose is printed every 1 sec.
//
// Has been tested with the KDLArmKinematicsPlugin (default plugin created by wizard),
// the IKFastKinematicsPlugin, which can be generated using the python script,
// and the example kinematics services from the 'arm_kinematics_test' package.

// based on this tutorial:
// http://www.ros.org/wiki/pr2_kinematics/Tutorials/Tutorial%203


// Note: there is a bug in arm_kinematics_constraint_aware.cpp in the arm_navigation stack
// version 1.1.11 from the Fuerte Deb.
// You should modify the file as per the one included with this package,
// or apply this patch (either works):
// https://code.ros.org/trac/ros-pkg/attachment/ticket/5497/patch.diff
// https://code.ros.org/trac/ros-pkg/attachment/ticket/5567/arm_kinematics_constraint_aware.patch

#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include "sensor_msgs/JointState.h"

unsigned int num_of_joints;
std::vector<float> joint_angles;

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

    // Subscribe to '/joint_states' topic and initialize vector of joint angles
    ros::Subscriber subscriberJointStates;
    subscriberJointStates = nh.subscribe("joint_states", 3, callbackJointStates);
    joint_angles.resize(num_of_joints);
    for (unsigned int i=0; i < num_of_joints; i++)
    {
        joint_angles[i] = 0.0;
    }

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

    ros::Rate loop_rate(10); // run loop every 1 sec
    while ( ros::ok() )
    {
        // Update service message with joint angles
        for (unsigned int i=0; i < num_of_joints; i++)
        {
            fk_request.robot_state.joint_state.position[i] = joint_angles[i];
        }

        // Call service
        if (fk_client.call(fk_request, fk_response))
        {
            if (fk_response.error_code.val == fk_response.error_code.SUCCESS)
            {
                for (unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
                {
                    ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str() );
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
    for (unsigned int i=0; i < num_of_joints; i++)
    {
        joint_angles[i] = (float)(msg->position[i]);
    }
}


