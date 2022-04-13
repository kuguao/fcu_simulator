#ifndef _SIMULATOR_MAVLINK_HPP
#define _SIMULATOR_MAVLINK_HPP

#include <thread>
#include <type_traits>
#include <mutex>

#include <mavlink_types.h>
#include <common/mavlink.h>
#include <protocol.h>
#include <common/mavlink_msg_hil_actuator_controls.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>

using namespace std;

template<typename E>
struct enable_bitmask_operators {
	static const bool enable = false;
};

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator==(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) ==
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator~(E lhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       ~static_cast<underlying>(lhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator|(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) |
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E &>::type
operator|=(E &lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	lhs = static_cast<E>(
		      static_cast<underlying>(lhs) |
		      static_cast<underlying>(rhs)
	      );
	return lhs;
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator&(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) &
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E &>::type
operator&=(E &lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	lhs = static_cast<E>(
		      static_cast<underlying>(lhs) &
		      static_cast<underlying>(rhs)
	      );
	return lhs;
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator^(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) ^
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E &>::type
operator^=(E &lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	lhs = static_cast<E>(
		      static_cast<underlying>(lhs) ^
		      static_cast<underlying>(rhs)
	      );
	return lhs;
}

#define ENABLE_BIT_OPERATORS(E) \
    template<> \
    struct enable_bitmask_operators<E> \
    { \
        static const bool enable = true; \
    };

enum class SensorSource {
	ACCEL		= 0b111,
	GYRO		= 0b111000,
	MAG		= 0b111000000,
	BARO		= 0b1101000000000,
	DIFF_PRESS	= 0b10000000000
};
ENABLE_BIT_OPERATORS(SensorSource)

#define PWM_DEFAULT_MAX 2000
#define PWM_DEFAULT_MIN 1000

class Simulator
{
private:
	shared_ptr<thread> simulator_thread_;

	const unsigned int port_;

	const uint8_t param_mav_sys_id_;
	const uint8_t param_mav_comp_id_;
	const uint8_t param_mav_type_;

	mutex mavlink_mtx_;
	bool connected_flag;
	Eigen::Vector3d gyro_;
	Eigen::Vector3d accel_;
	Eigen::Vector3d mag_;
	float abs_pressure_;
	float actuator_outputs_[16];
	bool armed_;

	ros::Subscriber model_state_sub_;
	ros::Publisher model_state_pub_;
	mutex model_mtx_;
	bool model_state_updated_;
	gazebo_msgs::ModelState model_state_;
	Eigen::Vector3d model_pose_position_;
	Eigen::Quaterniond model_pose_orientation_;
	Eigen::Vector3d model_twist_linear_;
	Eigen::Vector3d model_twist_angular_;

public:
	Simulator(ros::NodeHandle& n)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
		: port_(4560), param_mav_sys_id_(1)                                                                                                                                                    
		, param_mav_comp_id_(1), param_mav_type_(2)
		, armed_(true), connected_flag(false)
	{
		for (int i = 0; i < 16; i++)
			actuator_outputs_[i] = PWM_DEFAULT_MIN;
		simulator_thread_ = make_shared<thread>(&Simulator::run, this);

		model_state_sub_ = n.subscribe("/gazebo/model_states", 1000, &Simulator::model_state_callback, this);
		model_state_pub_ = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);

		while (ros::ok()) {
			mavlink_mtx_.lock();
			bool check = connected_flag;
			mavlink_mtx_.unlock();
			if(check == true)
				break;
			usleep(1000 * 100);
		}
		cout << "Simulator init successfully" << endl;
	}
	~Simulator() {

	}

	void model_state_callback(const gazebo_msgs::ModelStatesConstPtr& model_state);

	void run();
	void send_thread();
	void request_hil_state_quaternion();
	void send_mavlink_message(const mavlink_message_t &aMsg);
	void send_heartbeat();
	mavlink_hil_actuator_controls_t actuator_controls_from_outputs();

	void handle_message(const mavlink_message_t *msg);
	void handle_message_hil_sensor(const mavlink_message_t *msg);

	void set_actuator_output(float * out, int num);
	Eigen::Vector3d get_accel() {
		mavlink_mtx_.lock();
		Eigen::Vector3d ret = accel_;
		mavlink_mtx_.unlock();
		return ret;
	}
	Eigen::Vector3d get_gyro() {
		mavlink_mtx_.lock();
		Eigen::Vector3d ret = gyro_;
		mavlink_mtx_.unlock();
		return ret;
	}
	Eigen::Vector3d get_mag() {
		mavlink_mtx_.lock();
		Eigen::Vector3d ret = mag_;
		mavlink_mtx_.unlock();
		return ret;
	}
	float get_abs_pressure() {
		mavlink_mtx_.lock();
		float ret = abs_pressure_;
		mavlink_mtx_.unlock();
		return ret;
	}
	Eigen::Vector3d get_model_pose_position() {
		model_mtx_.lock();
		Eigen::Vector3d ret = model_pose_position_;
		model_mtx_.unlock();
		return ret;
	}
	Eigen::Quaterniond get_model_pose_orientation() {
		model_mtx_.lock();
		Eigen::Quaterniond ret = model_pose_orientation_;
		model_mtx_.unlock();
		return ret;
	}
	Eigen::Vector3d get_model_twist_linear() {
		model_mtx_.lock();
		Eigen::Vector3d ret = model_twist_linear_;
		model_mtx_.unlock();
		return ret;
	}
	Eigen::Vector3d get_model_twist_angular() {
		model_mtx_.lock();
		Eigen::Vector3d ret = model_twist_angular_;
		model_mtx_.unlock();
		return ret;
	}
	void set_model_pose_position(const Eigen::Vector3d& model_pose_position) {
		model_mtx_.lock();
		gazebo_msgs::ModelState msg = model_state_;
        msg.pose.position.x = model_pose_position(0, 0);
        msg.pose.position.y = model_pose_position(1, 0);
        msg.pose.position.z = model_pose_position(2, 0);
		model_mtx_.unlock();
		model_state_pub_.publish(msg);
		return;
	}
	void set_model_pose_orientation(const Eigen::Quaterniond& model_pose_orientation) {
		model_mtx_.lock();
		gazebo_msgs::ModelState msg = model_state_;
        msg.pose.orientation.x = model_pose_orientation.x();
        msg.pose.orientation.y = model_pose_orientation.y();
        msg.pose.orientation.z = model_pose_orientation.z();
		msg.pose.orientation.w = model_pose_orientation.w();
		model_mtx_.unlock();
		model_state_pub_.publish(msg);
		return;
	}
	void set_model_twist_linear(const Eigen::Vector3d& model_twist_linear) {
		model_mtx_.lock();
		gazebo_msgs::ModelState msg = model_state_;
        msg.twist.linear.x = model_twist_linear(0, 0);
        msg.twist.linear.y = model_twist_linear(1, 0);
        msg.twist.linear.z = model_twist_linear(2, 0);
		model_mtx_.unlock();
		model_state_pub_.publish(msg);
		return;
	}
	void set_model_twist_angular(const Eigen::Vector3d& model_twist_angular) {
		model_mtx_.lock();
		gazebo_msgs::ModelState msg = model_state_;
        msg.twist.angular.x = model_twist_angular(0, 0);
        msg.twist.angular.y = model_twist_angular(1, 0);
        msg.twist.angular.z = model_twist_angular(2, 0);
		model_mtx_.unlock();
		cout << msg << endl;
		model_state_pub_.publish(msg);
		return;
	}
	void set_model_state(const Eigen::Vector3d& model_pose_position
		, const Eigen::Quaterniond& model_pose_orientation
		, const Eigen::Vector3d& model_twist_linear
		, const Eigen::Vector3d& model_twist_angular) {
		model_mtx_.lock();
		gazebo_msgs::ModelState msg = model_state_;
        msg.pose.position.x = model_pose_position(0, 0);
        msg.pose.position.y = model_pose_position(1, 0);
        msg.pose.position.z = model_pose_position(2, 0);
        msg.pose.orientation.x = model_pose_orientation.x();
        msg.pose.orientation.y = model_pose_orientation.y();
        msg.pose.orientation.z = model_pose_orientation.z();
		msg.pose.orientation.w = model_pose_orientation.w();
        msg.twist.linear.x = model_twist_linear(0, 0);
        msg.twist.linear.y = model_twist_linear(1, 0);
        msg.twist.linear.z = model_twist_linear(2, 0);
        msg.twist.angular.x = model_twist_angular(0, 0);
        msg.twist.angular.y = model_twist_angular(1, 0);
        msg.twist.angular.z = model_twist_angular(2, 0);
		model_mtx_.unlock();
		cout << msg << endl;
		model_state_pub_.publish(msg);
		return;
	}
	bool get_model_state_updated() {
		model_mtx_.lock();
		bool ret = model_state_updated_;
		model_mtx_.unlock();
		return ret;
	}
	void clear_model_state_updated() {
		if (model_mtx_.try_lock()) {
			model_state_updated_ = false;
			model_mtx_.unlock();
		}
	}
};

typedef uint64_t	hrt_abstime;

#endif