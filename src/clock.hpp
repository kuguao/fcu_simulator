#ifndef _CLOCK_HPP
#define _CLOCK_HPP

#include <stdint.h>

#include <ros/ros.h>

extern uint64_t clock_time;

/**
 * 
 * @brief Get the clock time object. Time precision is 4ms, return value unit is us
 * 
 * @return uint64_t 
 */
inline uint64_t get_clock_time() {
    return clock_time;
}

int clock_time_init(ros::NodeHandle& n);

#endif