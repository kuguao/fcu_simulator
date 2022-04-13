#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <poll.h>

#include <common/mavlink.h>
#include <mavlink_types.h>
#include <common/mavlink_msg_command_long.h>
#include <common/common.h>

#include <Eigen/Core>

using namespace std;

#include "simulator_mavlink.hpp"
#include "clock.hpp"
#include "_math.hpp"

static int _fd;
static unsigned char _buf[2048];
static sockaddr_in _srcaddr;
static unsigned _addrlen = sizeof(_srcaddr);

const unsigned mode_flag_armed = 128;
const unsigned mode_flag_custom = 1;

void Simulator::run() {
	struct sockaddr_in _myaddr {};
	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(port_);

	printf("Waiting for simulator to accept connection on TCP port %u\r\n", port_);

	while (true) {
		if ((_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			printf("Creating TCP socket failed: %s\r\n", strerror(errno));
			return;
		}

		int yes = 1;
		int ret = setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int));

		if (ret != 0) {
			printf("setsockopt failed: %s\r\n", strerror(errno));
		}

		socklen_t myaddr_len = sizeof(_myaddr);
		ret = connect(_fd, (struct sockaddr *)&_myaddr, myaddr_len);

		if (ret == 0) {
			break;

		} else {
			::close(_fd);
			usleep(100);
		}
	}

	printf("Simulator connected on TCP port %u.\r\n", port_);

	thread send_thread(&Simulator::send_thread, this);

	struct pollfd fds[2] = {};
	unsigned fd_count = 1;
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	mavlink_status_t mavlink_status = {};

	request_hil_state_quaternion();

	while (1) {

		// wait for new mavlink messages to arrive
		int pret = ::poll(&fds[0], fd_count, 1000);

		if (pret == 0) {
			// Timed out.
			continue;
		}

		if (pret < 0) {
			printf("poll error %d, %d\r\n", pret, errno);
			continue;
		}

		if (fds[0].revents & POLLIN) {

			int len = ::recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, (socklen_t *)&_addrlen);

			if (len > 0) {
				mavlink_message_t msg;

				for (int i = 0; i < len; i++) {
					if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &mavlink_status)) {
						mavlink_mtx_.lock();
						connected_flag = true;
						mavlink_mtx_.unlock();
						handle_message(&msg);
					}
				}
			}
		}
	}
}

void Simulator::send_thread() {

	send_heartbeat();

	while (1) {
		uint64_t time = get_clock_time();

		mavlink_hil_actuator_controls_t hil_act_control = actuator_controls_from_outputs();

		mavlink_message_t message{};
		mavlink_msg_hil_actuator_controls_encode(param_mav_sys_id_, param_mav_comp_id_, &message, &hil_act_control);

		send_mavlink_message(message);

		// cout << "send actuator at time : " << get_clock_time() << endl;

		while (get_clock_time() - time < 3900 || get_clock_time() == 0) {
			usleep(200);
		}
	}
}

void Simulator::request_hil_state_quaternion() {
	mavlink_command_long_t cmd_long = {};
	mavlink_message_t message = {};
	cmd_long.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	cmd_long.param1 = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	cmd_long.param2 = 5e3;
	mavlink_msg_command_long_encode(param_mav_sys_id_, param_mav_comp_id_, &message, &cmd_long);
	send_mavlink_message(message);
}

void Simulator::send_mavlink_message(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t bufLen = 0;

	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	ssize_t len;

	len = ::send(_fd, buf, bufLen, 0);

	if (len <= 0) {
		printf("Failed sending mavlink message: %s\r\n", strerror(errno));
	}
}

void Simulator::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		// handle_message_optical_flow(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		// handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		// handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		// handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		// handle_message_hil_gps(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS:
		// handle_message_rc_channels(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		// handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
		// handle_message_hil_state_quaternion(msg);
		break;
	}
}

void abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

void Simulator::handle_message_hil_sensor(const mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	// gyro
	if (((SensorSource)imu.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		mavlink_mtx_.lock();
		gyro_ << imu.xgyro, imu.ygyro, imu.zgyro;
		mavlink_mtx_.unlock();
		// cout << get_clock_time() << " gyro : " << imu.xgyro << imu.ygyro << imu.zgyro << endl;
	}

	// accel
	if (((SensorSource)imu.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		mavlink_mtx_.lock();
		accel_ << imu.xacc, imu.yacc, imu.zacc;
		mavlink_mtx_.unlock();
		// cout << get_clock_time() << " accel : " << imu.xacc << imu.yacc << imu.zacc << endl;
	}

	// magnetometer
	if (((SensorSource)imu.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
		mavlink_mtx_.lock();
		mag_ << imu.xmag, imu.ymag, imu.zmag;
		mavlink_mtx_.unlock();
		// cout << get_clock_time() << " mag : " << mag_(0, 0) << " " << mag_(1, 0) << " " << mag_(2, 0) << endl;
	}

	// baro
	if (((SensorSource)imu.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		mavlink_mtx_.lock();
		abs_pressure_ = imu.abs_pressure;
		mavlink_mtx_.unlock();
	}

	// differential pressure
	if (((SensorSource)imu.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS) {

	}
}

mavlink_hil_actuator_controls_t Simulator::actuator_controls_from_outputs()
{
	mavlink_hil_actuator_controls_t msg{};

	msg.time_usec = get_clock_time();

	const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

	int _system_type = param_mav_type_;

	mavlink_mtx_.lock();
	bool armed = armed_;
	mavlink_mtx_.unlock();

	/* scale outputs depending on system type */
	if (_system_type == MAV_TYPE_QUADROTOR ||
	    _system_type == MAV_TYPE_HEXAROTOR ||
	    _system_type == MAV_TYPE_OCTOROTOR ||
	    _system_type == MAV_TYPE_VTOL_DUOROTOR ||
	    _system_type == MAV_TYPE_VTOL_QUADROTOR ||
	    _system_type == MAV_TYPE_VTOL_TILTROTOR ||
	    _system_type == MAV_TYPE_VTOL_RESERVED2) {

		/* multirotors: set number of rotor outputs depending on type */

		unsigned n;

		switch (_system_type) {
		case MAV_TYPE_VTOL_DUOROTOR:
			n = 2;
			break;

		case MAV_TYPE_QUADROTOR:
		case MAV_TYPE_VTOL_QUADROTOR:
		case MAV_TYPE_VTOL_TILTROTOR:
			n = 4;
			break;

		case MAV_TYPE_VTOL_RESERVED2:
			// this is the standard VTOL / quad plane with 5 propellers
			n = 5;
			break;

		case MAV_TYPE_HEXAROTOR:
			n = 6;
			break;

		default:
			n = 8;
			break;
		}

		mavlink_mtx_.lock();
		for (unsigned i = 0; i < 16; i++) {
			if (armed) {
				if (i < n) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for rotors */
					msg.controls[i] = (actuator_outputs_[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for other channels */
					msg.controls[i] = (actuator_outputs_[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
				}

			} else {
				/* send 0 when disarmed and for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}
		mavlink_mtx_.unlock();

	} else {
		/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */
		mavlink_mtx_.lock();
		for (unsigned i = 0; i < 16; i++) {
			if (armed) {
				if (i != 4) {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for normal channels */
					msg.controls[i] = (actuator_outputs_[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

				} else {
					/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for throttle */
					msg.controls[i] = (actuator_outputs_[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
				}

			} else {
				/* set 0 for disabled channels */
				msg.controls[i] = 0.0f;
			}
		}
		mavlink_mtx_.unlock();
	}

	msg.mode = mode_flag_custom;
	msg.mode |= (armed) ? mode_flag_armed : 0;
	msg.flags = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	msg.flags |= 1;
#endif

	return msg;
}

void Simulator::send_heartbeat()
{
	mavlink_mtx_.lock();
	bool armed = armed_;
	mavlink_mtx_.unlock();
	mavlink_heartbeat_t hb = {};
	mavlink_message_t message = {};
	hb.autopilot = 12;
	hb.base_mode |= armed ? 128 : 0;
	mavlink_msg_heartbeat_encode(param_mav_sys_id_, param_mav_comp_id_, &message, &hb);
	send_mavlink_message(message);
}

void Simulator::set_actuator_output(float * out, int num) {
	mavlink_mtx_.lock();
	for(int i = 0; i < 16; i++) {
		if(i < num)
			actuator_outputs_[i] = Math::limit(out[i], (float)PWM_DEFAULT_MIN, (float)PWM_DEFAULT_MAX);
	}
	mavlink_mtx_.unlock();
}

void Simulator::model_state_callback(const gazebo_msgs::ModelStatesConstPtr& model_state) {
	model_mtx_.lock();
	model_state_.model_name = model_state.get()->name[12];
	model_state_.pose = model_state.get()->pose[12];
	model_state_.twist = model_state.get()->twist[12];
	model_state_.reference_frame = "world";
	model_pose_position_ << model_state.get()->pose[12].position.x
		, model_state.get()->pose[12].position.y
		, model_state.get()->pose[12].position.z;
	model_pose_orientation_ = Eigen::Quaterniond(model_state.get()->pose[12].orientation.w
		, model_state.get()->pose[12].orientation.x
		, model_state.get()->pose[12].orientation.y
		, model_state.get()->pose[12].orientation.z);
	model_twist_linear_ << model_state.get()->twist[12].linear.x
		, model_state.get()->twist[12].linear.y
		, model_state.get()->twist[12].linear.z;
	model_twist_angular_ << model_state.get()->twist[12].angular.x
		, model_state.get()->twist[12].angular.y
		, model_state.get()->twist[12].angular.z;
	model_state_updated_ = true;
	model_mtx_.unlock();
	// cout << model_state.get()->pose[12] << endl;
	// cout << model_state.get()->twist[12] << endl;
}