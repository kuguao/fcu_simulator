#ifndef _IMU_HPP
#define _IMU_HPP

#include <Eigen/Core>

#include "simulator_mavlink.hpp"

class Imu{
private:
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyro_;
    Eigen::Vector3d mag_;
    Eigen::Vector3d acc_offset_;
    Eigen::Vector3d gyro_offset_;
    Eigen::Vector3d mag_offset_;
    bool is_calibrated_;
    bool is_calibrating_;
    int calibration_cnt_;

    Simulator& simulator_;

    void calibrate();
public:
    Imu(Simulator& sim) : 
        acc_(0.0, 0.0, 0.0), gyro_(0.0, 0.0, 0.0)
        , mag_(0.0, 0.0, 0.0), acc_offset_(0.0, 0.0, 0.0)
        , gyro_offset_(0.0, 0.0, 0.0), mag_offset_(0.0, 0.0, 0.0)
        , is_calibrated_(false), is_calibrating_(false)
        , calibration_cnt_(0), simulator_(sim) {}
    ~Imu() {}

    void request_calibrate();
    void imu_update();
};

inline void Imu::request_calibrate() {
    if (!is_calibrating_) {
        is_calibrating_ = true;
        calibration_cnt_ = 0;
    }
}


#endif