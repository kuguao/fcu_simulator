#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <Eigen/Geometry>

#include "simulator_mavlink.hpp"
#include "clock.hpp"
#include "_math.hpp"
#include "fcu.hpp"

using namespace std;

shared_ptr<Simulator> simulator;
shared_ptr<Fcu> fcu;

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

    simulator = make_shared<Simulator>(n);
    fcu = make_shared<Fcu>(*simulator);
    // Eigen::Vector3d a(1.0, 2.0, 5.0), b(10.0, 20.0, 30.0);
    // cout << a.transpose() << " " << b.transpose() << " " << (a+=b).transpose() << " " << (b /= 2).transpose() << endl; 

    // ros::Publisher model_state_pub_ = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
    // sleep(2);
    // int i = 500;
    // while (ros::ok())
    // {
    //     //simulator->set_model_twist_angular(Eigen::Vector3d(0.f, 0.f, 1.f));
    //     while (!simulator->get_model_state_updated() && ros::ok()) {
    //         ros::spinOnce();
    //     }
    //     simulator->clear_model_state_updated();
    //     // cout << get_clock_time() / 1000 << " : "<< orie.x()  << " " << orie.y()  << " "  << orie.z()  << " "  << orie.w()  << " "  << endl;
    // }
    ros::spin();

    return 0;
}