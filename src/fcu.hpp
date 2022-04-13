#ifndef _FCU_HPP
#define _FCU_HPP

#include <thread>

#include "imu.hpp"
#include "simulator_mavlink.hpp"

using namespace std;

class Fcu
{
private:
    shared_ptr<thread> fcu_thread_;

    Simulator& simulator_;
    Imu imu;

public:
    Fcu(Simulator& sim) : simulator_(sim) , imu(simulator_){
        fcu_thread_ = make_shared<thread>(&Fcu::run, this);
        imu.request_calibrate();
    };
    ~Fcu() {}
    void run();
    void task_250hz();
    void task_125hz();
    void task_50hz();
    void task_25hz();
    void task_5hz();
    void task_1hz();
};

#endif