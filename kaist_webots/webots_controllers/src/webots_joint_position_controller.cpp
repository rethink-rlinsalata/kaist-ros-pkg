/*
 * webots_joint_position_controller.cpp
 *
 * Stack: kaist_webots
 * Package: webots_controllers
 *
 * A joint position controller for Webots, utilising the C++ API and simple P-control.
 * 
 * The controller configuration is defined in a yaml file, which includes the ROS topic for
 * subscribing to commands for each joint, the joint's servo name in Webots, and the value
 * of the P constant.
 * The 'webots_controllers_to_load' parameter in the launch file specifies which controllers
 * from the yaml file should be loaded.
 *
 * Position commands are received on a ROS topic for each joint.
 * Current position and calculated velocity is published on the /joint_states topic.
 * Force information is not checked because there is a bug with Webots 6.4.4
 *
 * 
 * Notes:
 *   More info about Webots P-control:
 *   http://www.cyberbotics.com/dvd/common/doc/webots/reference/section3.41.html
 *   
 *   With the default P value of 10, there is some instability in the waist joint controller
 *   which results in +/-0.00009 radians position variation while stationary.
 *   
 *   The calculated velocity fluctuates during the joint move, so a more robust calculation
 *   algorithm is required. Better results are obtained by using a higher value for basicTimeStep
 *   in the world file, and the controller TIME_STEP should be the same.
 *   
 *   By default, all printed output or errors from this controller will go to the Webots console
 *   so printf() is used instead of ROS_INFO and ROS_ERROR.
 *   If you want to use ROS logging, you can change this and redirect output to the shell console
 *   by setting a Webots environment variable in the launch file.
 *   
 *   This controller does not limit the joint motion.
 *   You should configure limits for revolute joints in Webots:
 *       Soft limits:            minPosition / maxPosition 
 *       Hard physical limits:   minStop / maxStop
 *   
 *   
 * Tested with ROS Fuerte, Webots 6.4.4
 *
 * Author: David Butterworth, KAIST
 * Date: October 2012
 */

#include <ros/ros.h>                // ROS
#include <std_msgs/Float64.h>       // Message type: joint position commands
#include <sensor_msgs/JointState.h> // Message type: joint states

#include <webots/Robot.hpp>         // Webots C++ API

// Controller update rate:
// This needs to be a multiple of basicTimeStep (the simulation update rate defined in the Webots world file)
// for Webots to run, however it needs to be the same value so the formula used in this controller for
// calculating joint velocity gives the correct output.
// basicTimeStep is 20 for the Puma 560 Arm, the Webots default value is 32.
#define TIME_STEP 20

// Constants
#define PI 3.141592f
#define RAD2DEG (180.0f/PI)

// ROS parameter with list of controllers to load
#define ROSPARAM_CONTROLLERS_LIST "webots_controllers_to_load"

// Controller class:
// derives from the same class as used in the Webots Scene Tree: Robot, DifferentialWheels or Supervisor
class WebotsJointPositionController : public webots::Robot
{
    public:
        // Constructor
        WebotsJointPositionController();

        // Run control loop
        void run(void);

    private:
        ros::NodeHandle nh_;

        // Stores joint/servo information:
        // (0) ROS topic for joint control command
        // (1) Servo name in Webots
        // (2) un-used
        // (3) Current position command (rads)
        // (4) Previous command (rads)
        std::vector <std::vector <std::string> > joint_list_;

        // ROS Joint State Publisher
        sensor_msgs::JointState joint_states_;
        ros::Publisher publisherJointState;

        // ROS Subscribers are defined based on the number of joints
        std::vector <ros::Subscriber> list_of_subscribers_;

        // Read the controller configuration from the ROS Parameter server,
        // and add joint information to 'joint_list_' and 'joint_states_' variables
        void getJointParams(void);

        // Access lists on the ROS Parameter Server
        bool getROSParamList(XmlRpc::XmlRpcValue param, std::vector<std::string> &output);

        // Initialize ROS Message Publishers
        void initializePublishers(void);

        // Initialize ROS Message Subscribers
        void initializeSubscribers(void);

        // Universal callback for processing joint commands
        void callbackJointCommands(const std_msgs::Float64ConstPtr& msg, const int joint_ID, const std::string joint_name);
};


// Constructor
WebotsJointPositionController::WebotsJointPositionController() : 
  webots::Robot()
{
}


// Run controller
void WebotsJointPositionController::run(void)
{
    getJointParams();

    initializePublishers();

    printf("Attempting to start %d controllers... \n", joint_list_.size() );
    initializeSubscribers();

    printf("Controller step time = %d ms  \n", TIME_STEP );

    // Main control loop: runs every TIME_STEP ms
    ros::Time now; // For ROS timestamp
    double current_time_sec = 0.0; // For calculating elapsed time
    double prev_time_sec = 0.0;
    double elapsed_time_sec = 0.0;
    while(step(TIME_STEP) != -1) 
    {
        // Get timestamp, for ROS 'joint_states' message
        now = ros::Time::now();

        // Calculate elapsed time since control loop last ran, 
        // for determining the joint velocities
        prev_time_sec = current_time_sec;

        // Convert ROS timestamp to milliseconds
        current_time_sec = (double)( now.toNSec() ) * 1e-9;
        elapsed_time_sec = current_time_sec - prev_time_sec;

        // Debug info:
        //printf("Elapsed time since last loop: %f sec \n", elapsed_time_sec );

        // Itterate over each joint
        unsigned int i = 0;
        float prev_position = 0.0;
        float current_position = 0.0;
        float position_delta;
        float velocity = 0.0;
        double goal_position = 0.0;
        //float force = 0.0;
        for(std::vector <std::vector <std::string> >::iterator joint_list_iterator = joint_list_.begin(); 
                                                          joint_list_iterator!=joint_list_.end(); joint_list_iterator++) 
        {
            // Get Servo device instance pointer
            webots::Servo* servo_device_ref = getServo( (*joint_list_iterator)[1] );
            // Get previous joint position from joint_list_
            prev_position = boost::lexical_cast<float>( (*joint_list_iterator)[4] ); 
            // Get current position of servo from Webots and store in joint_list_
            current_position = (float)( servo_device_ref->getPosition() );  
            (*joint_list_iterator)[4] = boost::lexical_cast<std::string>(current_position);
            // Calculate current velocity of joint
            position_delta = (current_position - prev_position); // radians
            velocity = position_delta / (float)elapsed_time_sec; // rad/sec

            // Debug info:
            //printf("Joint: %s  Position change: %f radians  Velocity: %f rad/sec  \n", (*joint_list_iterator)[1].c_str(), position_delta, velocity);
            //printf("Joint: %s  Position change: %f degrees  Velocity: %f deg/sec  \n", 
            //            (*joint_list_iterator)[1].c_str(), (position_delta/RAD2DEG), (velocity/RAD2DEG) );

            // Read position command from memory
            goal_position = boost::lexical_cast<double>( (*joint_list_iterator)[3] );
            // Move joint Servo using simple Webots P-control
            servo_device_ref->setPosition(goal_position); 
            // Write joint position (rad) & velocity (rad/sec) to ROS '/joint_states' topic
            joint_states_.position[i] = current_position;
            joint_states_.velocity[i] = velocity;

            // Write measured joint force
            // Force measurement is not tested. When the enable function is called, Webots 6.4.4 crashes.
            //force  = (float)( servo_device_ref->getMotorForceFeedback() );
            //printf("force: %f  \n", force );
            //joint_states.effort[i] = force;

            i++;
        }
        // Publish ROS 'joint_state' message
        publisherJointState.publish(joint_states_);

        // Check for ROS callbacks
        ros::spinOnce();

    } //end Webots control loop
}


// Read the controller configuration from the ROS Parameter server,
// and add joint information to 'joint_list_' and 'joint_states_' variables
void WebotsJointPositionController::getJointParams(void)
{
    // Read list of joint controllers from ROS Parameter '/webots_controllers_to_load'
    // This parameter name is defined by ROSPARAM_CONTROLLERS_LIST at the top of this file.
    // Each entry looks like 'MyRobot/SimController/JointName'
    printf("Reading controller configuration from ROS Parameter Server... \n\n");
    XmlRpc::XmlRpcValue xml_handle_joints;
    std::vector<std::string> xml_list_joints;
    if (!ros::param::get(ROSPARAM_CONTROLLERS_LIST, xml_handle_joints))
    {
        printf("FATAL ERROR: Could not read '%s' from ROS Parameter Server. You should define this parameter in the ROS .launch file.' \n", ROSPARAM_CONTROLLERS_LIST );
        printf("Exiting ROS... \n\n");
        exit(0);
    }
    else
    {
	// Iterate over each entry in the list...
        getROSParamList(xml_handle_joints, xml_list_joints);
        unsigned int i = 0;
        for(std::vector<std::string>::iterator it = xml_list_joints.begin(); it != xml_list_joints.end(); it++)
        {
            // Get the matching joint name from the ROS parameter server, which is the servo name in Webots.
            // This is stored in the '/MyRobot/SimController/JointName/joint' parameter, which is loaded from the .yaml
            // file in the config directory.
            std::string joint_name;
            if (ros::param::get( xml_list_joints[i]+"/joint", joint_name ))
            {
                // Check this servo name exists in Webots robot model and get pointer to the device instance
                webots::Servo* servo_device_ref;
                if ( (servo_device_ref = getServo( joint_name )) )
                {
                    // Add this joint to 'joint_list'
                    std::vector <std::string> temp;
                    temp.push_back( xml_list_joints[i].c_str() ); // Controller path eg. MyRobot/SimController/JointName
                    temp.push_back( joint_name.c_str() );        // Servo name eg. shoulder
                    temp.push_back( "" );                       // Un-used, originally used to store device ptr
                    temp.push_back("0.0");                     // Joint angle command (radians)
                    temp.push_back("0.0");                    // Previous joint angle command (radians)
                    joint_list_.push_back(temp);

                    // Add joint name to header of 'joint_states' ROS Message
                    joint_states_.name.push_back( joint_name.c_str() );
                    // Enable position measurements for servo in Webots
                    servo_device_ref->enablePosition(TIME_STEP); // TIME_STEP is period to check servo position

                    // Enable force measurements
                    // Force measurement is not tested. When this function is called, Webots 6.4.4 crashes.
                    //servo_device_ref->enableMotorForceFeedback(TIME_STEP); 

                    // Set the P-controller constant setting for this joint, from the ROS parameter server,
                    // otherwise use Webots default value ControlP = 10
                    double joint_p_setting;
                    if (ros::param::get( xml_list_joints[i]+"/pid/p", joint_p_setting ))
                    {
                        servo_device_ref->setControlP(joint_p_setting); 
                    }
                }
                else
                {
                    printf("ERROR: No servo called '%s' was found in Webots model. \n", joint_name.c_str() );
                }
            }
            else
            {
                printf("ERROR: Could not read '%s/joint' from ROS Parameter Server. Check the .yaml file in the /config directory. \n", xml_list_joints[i].c_str() );
            }
            i++;
        }
    }

}


// Access lists on the ROS Parameter Server
bool WebotsJointPositionController::getROSParamList(XmlRpc::XmlRpcValue param, std::vector<std::string> &output)
{
    XmlRpc::XmlRpcValue::Type type = param.getType();
    if (type == XmlRpc::XmlRpcValue::TypeString)
    {
        std::string find = param;
        output.push_back(find);
        return true;
    }
    else if (type == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < param.size(); ++i)
        {
            if (param[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                printf("Parameter is not a list of strings, found non-string value. XmlRpcValue: %s", param.toXml().c_str());
                output.clear();
                return false;
            }
            std::string find = param[i];
            output.push_back(find);
        }
        return true;
    }
    printf("Parameter not a list or string, unable to return values. XmlRpcValue:s %s", param.toXml().c_str());
    output.clear();
    return false;
}


// Initialize ROS Message Publishers
void WebotsJointPositionController::initializePublishers(void)
{
    // Initialize the 'joint_states' Message
    unsigned int number_of_controllers = joint_list_.size();
    joint_states_.name.resize(number_of_controllers);
    joint_states_.position.resize(number_of_controllers);
    joint_states_.velocity.resize(number_of_controllers);
    joint_states_.effort.resize(number_of_controllers);

    // Initialize Joint State Publisher
    // Publish Webots joint positions (radians) & current velocity to '/joint_states' topic
    publisherJointState = nh_.advertise<sensor_msgs::JointState>("joint_states", 10); //buffer 10 msgs
}


// Initialize ROS Message Subscribers
void WebotsJointPositionController::initializeSubscribers(void)
{
    // For each joint, subscribe to a ROS topic to receive position commands
    unsigned int i = 0;
    for(std::vector <std::vector <std::string> >::iterator joint_list_iterator = joint_list_.begin(); 
                                                          joint_list_iterator!=joint_list_.end(); joint_list_iterator++) 
    {
        // Print controller info:
        //      (number) JointName '/MyRobot/SimController/JointName/command'
        printf("(%d) %s '%s/command' \n", (i+1), (*joint_list_iterator)[1].c_str(), (*joint_list_iterator)[0].c_str() );
        // Each joint command is handled by the same callback, but we pass in the command, index number in joint_list_, and joint name
        list_of_subscribers_.push_back( nh_.subscribe<std_msgs::Float64>( 
                        (*joint_list_iterator)[0]+"/command", 1, boost::bind(&WebotsJointPositionController::callbackJointCommands, 
                        this, _1, i, (*joint_list_iterator)[1]) ));
                           // topic_name is from ROS parameter, queue_size=1
        i++;
    }
}


// Universal callback for processing joint commands
void WebotsJointPositionController::callbackJointCommands(const std_msgs::Float64ConstPtr& msg, const int joint_ID, const std::string joint_name) 
{ 
    // Save the position command in joint_list_
    joint_list_[joint_ID][3] = boost::lexical_cast<std::string>(msg->data);   

    // Debug info:
    //printf("Callback for joint: %s  joint_list_ index: %d  Position command: %f \n", joint_name.c_str(), joint_ID, msg->data );
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "webots_joint_position_controller");

    WebotsJointPositionController *webots_joint_position_controller = new WebotsJointPositionController();
    webots_joint_position_controller->run();

    delete webots_joint_position_controller;
    ros::shutdown();

    return 0;
}

