#include "clock.hpp"

#include <iostream>
#include <mutex>

#include <rosgraph_msgs/Clock.h>

using namespace std;

uint64_t clock_time;

mutex clock_mtx;
static ros::Subscriber clock_sub;

static void clock_callback(const rosgraph_msgs::Clock& sim_clock) {
    clock_mtx.lock();
    clock_time = sim_clock.clock.sec * 1000000 + sim_clock.clock.nsec / 1000;
    clock_mtx.unlock();
    // struct timeval tv;
    // gettimeofday(&tv, NULL);
	// struct timeval tv;
	// gettimeofday(&tv, NULL);
	// cout << "SimTime : " << get_clock_time() / 1000 << " " << tv.tv_usec / 1000 + tv.tv_sec * 1000<< endl;
}

int clock_time_init(ros::NodeHandle& n) {
    clock_sub = n.subscribe("/clock", 1000, clock_callback);
    while (clock_time == 0 && ros::ok()) {
        ros::spinOnce();
    }
    cout << "Clock init successfully!\r\n" << endl;

    return 0;
}

/**
 * 
 * @brief Get the clock time object. Time precision is 4ms, return value unit is us
 * 
 * @return uint64_t 
 */
uint64_t get_clock_time() {
    clock_mtx.lock();
    uint64_t ret = clock_time;
    clock_mtx.unlock();
    return ret;
}