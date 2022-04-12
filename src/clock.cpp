#include "clock.hpp"

#include <iostream>

#include <rosgraph_msgs/Clock.h>

using namespace std;

uint64_t clock_time;

static ros::Subscriber clock_sub;

static void clock_callback(const rosgraph_msgs::Clock& sim_clock) {
    clock_time = sim_clock.clock.sec * 1000000 + sim_clock.clock.nsec / 1000;
    // struct timeval tv;
    // gettimeofday(&tv, NULL);
    // cout << "SimTime : " << clock_time << endl;
}

int clock_time_init(ros::NodeHandle& n) {
    clock_sub = n.subscribe("/clock", 1000, clock_callback);

    return 0;
}