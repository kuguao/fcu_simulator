#ifndef _CLOCK_HPP
#define _CLOCK_HPP

#include <stdint.h>

#include <ros/ros.h>

int clock_time_init(ros::NodeHandle& n);
uint64_t get_clock_time();

#endif