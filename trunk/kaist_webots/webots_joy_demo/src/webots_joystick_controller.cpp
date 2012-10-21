/*
 * webots_joystick_controller.cpp
 *
 * Stack: kaist_webots
 * Package: webots_joy_demo
 *
 * This is an example of a Webots Controller written in C++, running as a ROS Node.
 * It demonstrates how to build a ROS package linked against the Webots CppLibrary,
 * control a robot using ROS messages, and start the simulation from a single launch file.
 *
 * The 'joy' node from the 'joystick_drivers' stack receives input from a USB joystick
 * and publishes the state of the buttons and axes on the '/joy' topic.
 * This node uses that input to drive a simple simulated robot in Webots.
 * Speed commands are sent to the DifferentialDrive model using the Webots C++ API.
 * 
 * Tested with ROS Fuerte, Webots 6.4.4, Microsoft Xbox-360 joystick.
 *
 * Author: David Butterworth, KAIST
 * Date: October 2012
 * 
 * This code is based on 'webots_joystick', a controller using the Webots C API
 * written by Andreas Breitenmoser (ETHZ) and Olivier Michel (Cyberbotics Ltd.)
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <webots/Robot.hpp>
#include <webots/DifferentialWheels.hpp>

// Controller update rate:
// needs to be a multiple of basicTimeStep, the simulation update rate defined in the Webots world file
#define TIME_STEP 32 // Default is 32

// Note:
// printf() is used instead of ROS_INFO, ROS_WARN, etc. because the output goes to the Webots console.
// If you want to use ROS logging, you can change this and set the environment variable in the launch file.

// Controller class:
// derives from the same class as used in the Webots Scene Tree: Robot, DifferentialWheels or Supervisor
class WebotsJoystickController : public webots::DifferentialWheels
{
    public:
        // Constructor
        WebotsJoystickController();

        // Run control loop
        void run(void);

    private:
        enum{JOYPAD_LEFT = 1, JOYPAD_RIGHT, JOYPAD_UP, JOYPAD_DOWN, JOYPAD_STOP};
        unsigned int cmd_;
        double left_speed_;
        double right_speed_;
        double k_speed_;

        ros::NodeHandle nh_;
        ros::Subscriber subscriberJoy;

        // Callback for ROS messages on '/joy' topic
        void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy);
};


// Constructor
WebotsJoystickController::WebotsJoystickController() : 
  webots::DifferentialWheels(),
  cmd_(JOYPAD_STOP),
  left_speed_(0.0),
  right_speed_(0.0),
  k_speed_(0.0)
{
    // Get robot speed setting from ROS Parameter server
    nh_.param("robot_speed", k_speed_, 30.0);

    // Initialize ROS Subscriber
    subscriberJoy = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &WebotsJoystickController::callbackJoy, this);
}

// Run controller
void WebotsJoystickController::run(void)
{
    // Helpful info, will be displayed in Webots console
    printf("Joypad connected and running. \n");

    printf("Commands: \n");
    printf("  arrow up    = move forward \n");
    printf("  arrow down  = move backward \n");
    printf("  arrow left  = rotate left \n");
    printf("  arrow right = rotate right \n");
    printf("  button 1    = stop \n");
 
    printf("Robot speed: %d \n", (int)k_speed_ );

    // Main control loop: runs every TIME_STEP ms
    while(step(TIME_STEP) != -1) {

        // Process most recent joystick command   
        switch(cmd_) 
        {
            case JOYPAD_UP:
                left_speed_  = k_speed_;
                right_speed_ = k_speed_;
                break;
            case JOYPAD_DOWN:
                left_speed_  = -k_speed_;
                right_speed_ = -k_speed_;
                break;
            case JOYPAD_LEFT:
                left_speed_  = -k_speed_;
                right_speed_ = k_speed_;
                break;
            case JOYPAD_RIGHT:
                left_speed_  = k_speed_;
                right_speed_ = -k_speed_;
                break;
            default:  // case JOYPAD_STOP:
                left_speed_ = 0.0;
                right_speed_ = 0.0;
                break;
        }

        // Set speed of differential drive wheels
        setSpeed(left_speed_, right_speed_);

        // Check for callbacks
        ros::spinOnce();
    }
}

// Callback for ROS messages on '/joy' topic
void WebotsJoystickController::callbackJoy(const sensor_msgs::Joy::ConstPtr& joy) {

    // Debug: 
    // when joystick button is pressed, display values of all
    // axes/buttons in the Webots console
    /*
    printf("Axes:");
    for(size_t i=0;i<joy->axes.size();i++) printf(" %g",joy->axes[i]);
    printf(" - Buttons:");
    for(size_t i=0;i<joy->buttons.size();i++) printf(" %d",joy->buttons[i]);
    printf("\n");
    */

    // The joystick outputs both axis values at once, so lets find the
    // axis with the biggest value and set the matching command state
    float m = 0.0;
    for (size_t i = 0; i < 2; i++) 
    {
        if (fabsf(joy->axes[i]) > m) 
            m = fabsf(joy->axes[i]);
    }
    if (m > 0.1) {
        if ( joy->axes[0] == m) 
            cmd_ = JOYPAD_LEFT;
        else if (-joy->axes[0] == m) 
            cmd_ = JOYPAD_RIGHT;
        else if (joy->axes[1] == m) 
            cmd_ = JOYPAD_UP;
        else if (-joy->axes[1] == m)  
            cmd_ = JOYPAD_DOWN;
    }
    // If button 1 or button 2 is pressed, stop the robot
    if (joy->buttons[0] || joy->buttons[1]) 
        cmd_ = JOYPAD_STOP;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "webots_joy_demo");

    WebotsJoystickController *webots_joy_demo = new WebotsJoystickController();
    webots_joy_demo->run();

    delete webots_joy_demo;
    ros::shutdown();

    return 0;
}





