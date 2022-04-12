#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "simulator_mavlink.hpp"
#include "clock.hpp"
#include "_math.hpp"

using namespace std;

Simulator* simulator;

void joy_callback(const sensor_msgs::JoyConstPtr& msg) {
    float upward = msg->axes[1];
    float angular = msg->axes[0];
    float forward = msg->axes[4];
    float leftward = msg->axes[3];
    float actuator[4];
    actuator[0] = actuator[1] = actuator[2] = actuator[3] = Math::scale(upward, -1.f, 1.f, PWM_DEFAULT_MIN, PWM_DEFAULT_MAX);
    simulator->set_actuator_output(actuator, 4);
    printf("%f,%f,%f,%f -- %f,%f,%f,%f \n\r"
        , upward, angular, forward, leftward
        , actuator[0], actuator[1], actuator[2], actuator[3]);
}

int main(int _argc, char** _argv) {
    ros::init(_argc, _argv, "kugua_fcu");
    ros::NodeHandle n;

    clock_time_init(n);

    ros::Subscriber joy_sub = n.subscribe("/joy", 5, joy_callback);

    simulator = new Simulator;

    ros::spin();

    delete simulator;

    return 0;
}