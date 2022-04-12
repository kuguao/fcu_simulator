#ifndef _SIMULATOR_MAVLINK_HPP
#define _SIMULATOR_MAVLINK_HPP

#include <thread>
#include<type_traits>

#include <mavlink_types.h>
#include <common/mavlink.h>
#include <protocol.h>
#include <common/mavlink_msg_hil_actuator_controls.h>

#include <Eigen/Core>

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
	shared_ptr<thread> simulator_thread;

	unsigned int port_;

	uint8_t param_mav_sys_id_;
	uint8_t param_mav_comp_id_;
	uint8_t param_mav_type_;

	Eigen::Vector3d gyro_;
	Eigen::Vector3d accel_;
	Eigen::Vector3d mag_;
	float abs_pressure_;

	float actuator_outputs_[16];
	bool armed;
public:
	Simulator() 
		: port_(4560), param_mav_sys_id_(1)
		, param_mav_comp_id_(1), param_mav_type_(2)
		, armed(true) 
	{
		for (int i = 0; i < 16; i++)
			actuator_outputs_[i] = PWM_DEFAULT_MIN;
		simulator_thread = make_shared<thread>(&Simulator::run, this);
	}
	~Simulator() {

	}

	void run();
	void send_thread();
	void request_hil_state_quaternion();
	void send_mavlink_message(const mavlink_message_t &aMsg);
	void send_heartbeat();
	mavlink_hil_actuator_controls_t actuator_controls_from_outputs();

	void handle_message(const mavlink_message_t *msg);
	void handle_message_hil_sensor(const mavlink_message_t *msg);

	void set_actuator_output(float * out, int num);
};

typedef uint64_t	hrt_abstime;

#endif