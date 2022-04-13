#include <iostream>

#include <Eigen/Core>

#include "imu.hpp"
#include "simulator_mavlink.hpp"

using namespace std;

void Imu::imu_update() {
    acc_ = simulator_.get_accel();
    gyro_ = simulator_.get_gyro();
    mag_ = simulator_.get_mag();

    if (true == is_calibrating_) {
        calibrate();
    } else {
        acc_ -= acc_offset_;
        gyro_ -= gyro_offset_;
        mag_ -= mag_offset_;
    }

    // cout << "acc : " << acc_.transpose() << endl;
    // cout << "gyro : " << gyro_.transpose() << endl;
    // cout << "mag : " << mag_.transpose() << endl;
}

void Imu::calibrate() {
    if (calibration_cnt_ == 0) {
        acc_offset_ << 0., 0., 0.;
        gyro_offset_ << 0., 0., 0.;
    }

    acc_offset_ += acc_ - Eigen::Vector3d(0.0, 0.0, -9.8066);
    gyro_offset_ += gyro_;

    calibration_cnt_++;
    cout << calibration_cnt_ << endl;

    if (calibration_cnt_ >= 250) {
        acc_offset_ /= calibration_cnt_;
        gyro_offset_ /= calibration_cnt_;
        is_calibrated_ = true;
        is_calibrating_ = false;
        calibration_cnt_ = 0;
        // cout << "***************" << endl;
        // cout << acc_offset_.transpose() << endl;
        // cout << gyro_offset_.transpose() << endl;
        // cout << "***************" << endl;
    }
}