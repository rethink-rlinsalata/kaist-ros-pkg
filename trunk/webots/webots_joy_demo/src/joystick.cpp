/*
 * joystick.cpp
 *
 * NOTE:
 * This package 'webots_joy_demo' has been adapted from 'webots_joystick'
 * by Andreas Breitenmoser.
 * Joystick input is received by the node 'joy_node' which replaces the 
 * deprecated node 'joy' that was used in the original package.
 * Tested with ROS Fuerte.
 *
 *
 * Sample implementation of a Webots controller as a ROS node
 *
 * This Webots controller is a ROS node that subscribes to the
 * /joy topic to read input from a USB joystick. 
 * This input is used to drive a simple simulated robot in Webots.
 *
 *
 * Date:    September 2012
 * Authors: Andreas Breitenmoser, ETHZ
 *          Olivier Michel, Cyberbotics Ltd.
 *          David Butterworth, KAIST
 *
 */

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <errno.h>

#define SPEED 30

enum{JOYPAD_LEFT = 1, JOYPAD_RIGHT, JOYPAD_UP, JOYPAD_DOWN, JOYPAD_STOP};

static int cmd;

void joy_callback(const  sensor_msgs::Joy::ConstPtr& joy) {

    // When joystick button is pressed, display values of all
    // axes/buttons in the Webots console
    /*
    printf("Axes:");
    for(size_t i=0;i<joy->axes.size();i++) printf(" %g",joy->axes[i]);
    printf(" - Buttons:");
    for(size_t i=0;i<joy->buttons.size();i++) printf(" %d",joy->buttons[i]);
    printf("\n");
    */

    // depending on your joystick, you may want to set the
    // cmd variable according to different axes or buttons events.

    // find the biggest axis position
    float m=0;
    for(size_t i=0;i<2;i++) if (fabsf(joy->axes[i]) > m) m=fabsf(joy->axes[i]);
    if (m>0.1) {
           if ( joy->axes[0] == m) cmd=JOYPAD_LEFT;
      else if (-joy->axes[0] == m) cmd=JOYPAD_RIGHT;
      else if ( joy->axes[1] == m) cmd=JOYPAD_UP;
      else if (-joy->axes[1] == m) cmd=JOYPAD_DOWN;
    }
    // force stop if button 1 or button 2 is pressed
    if (joy->buttons[0] || joy->buttons[1]) cmd=JOYPAD_STOP;

    /*
    printf("cmd=%d\n",cmd);
    */
}

int main(int argc, char **argv) {

    wb_robot_init();
 
    ros::init(argc, argv, "joystick");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("joy", 10, joy_callback);

    double left_speed = 0;
    double right_speed = 0;
  
    // Helpful info, will be displayed in Webots console
    ROS_INFO("Joypad connected and running.");
    ROS_INFO("Commands:");
    ROS_INFO("arrow up: move forward, speed up");
    ROS_INFO("arrow down: move backward, slow down");
    ROS_INFO("arrow left: turn left");
    ROS_INFO("arrow right: turn right");
  
    // control loop: simulation steps of 32 ms
    while(wb_robot_step(32) != -1) {

        // get callback called
        ros::spinOnce();

        switch(cmd) {

        case JOYPAD_UP:
          left_speed=SPEED;
          right_speed=SPEED;
          break;

        case JOYPAD_DOWN:
          left_speed=-SPEED;
          right_speed=-SPEED;
          break;
        
        case JOYPAD_LEFT:
          left_speed=-SPEED;
          right_speed=SPEED;
          break;
       
        case JOYPAD_RIGHT:
          left_speed=SPEED;
          right_speed=-SPEED;
          break;
      
        case JOYPAD_STOP:
          left_speed = 0;
          right_speed = 0;
          break;
        }

        // set motor speeds
        wb_differential_wheels_set_speed(left_speed, right_speed);
    }

    ros::shutdown();
    wb_robot_cleanup();
    return 0;
}

