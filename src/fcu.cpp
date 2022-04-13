#include "fcu.hpp"
#include "clock.hpp"

void Fcu::run() {

    while (1) {
        uint64_t time = get_clock_time();
        static int loop_cnt = 0;
        if(loop_cnt) {
            task_250hz();
        }
        if(loop_cnt % 2 == 0) {
            task_125hz();
        }
        if(loop_cnt % 5 == 0) {
            task_50hz();
        }
        if(loop_cnt % 10 == 0) {
            task_25hz();
        }
        if(loop_cnt % 50 == 0) {
            task_5hz();
        }
        if(loop_cnt % 250 == 0) {
            task_1hz();
        }
        if(++loop_cnt >= 250)
            loop_cnt = 0;

        while (get_clock_time() - time < 3900 || get_clock_time() == 0) {
			usleep(100);
		}
    }

}

void Fcu::task_250hz() {
    double dt = 0.004;

    // cout << "SimTime : " << get_clock_time() / 1000 << endl;
    imu.imu_update();
}

void Fcu::task_125hz() {
    double dt = 0.008;

}

void Fcu::task_50hz() {
    double dt = 0.02;

}

void Fcu::task_25hz() {
    double dt = 0.04;

}

void Fcu::task_5hz() {
    double dt = 0.2;

}

void Fcu::task_1hz() {
    double dt = 1.0;

    // cout << get_clock_time() << endl;
}
