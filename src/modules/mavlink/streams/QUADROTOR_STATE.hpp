
#ifndef QUADROTOR_STATE_HPP
#define QUADROTOR_STATE_HPP

#include <uORB/topics/vehicle_odometry.h>

class MavlinkStreamQuadrotorState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamQuadrotorState(mavlink); }

	static constexpr const char *get_name_static() { return "QUADROTOR_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_QUADROTOR_STATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _odom_sub.advertised() ? MAVLINK_MSG_ID_QUADROTOR_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamQuadrotorState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _odom_sub{ORB_ID(vehicle_odometry)};

	vehicle_odometry_s _odom;
	mavlink_quadrotor_state_t _msg{};

	bool send() override
	{
		_odom_sub.copy(&_odom);
		_msg.Px = _odom.x;
		_msg.Py = _odom.y;
		_msg.Pz = _odom.z;

		_msg.Qw = _odom.q[0];
		_msg.Qx = _odom.q[1];
		_msg.Qy = _odom.q[2];
		_msg.Qz = _odom.q[3];

		_msg.Vx = _odom.vx;
		_msg.Vy = _odom.vy;
		_msg.Vz = _odom.vz;

		mavlink_msg_quadrotor_state_send_struct(_mavlink->get_channel(), &_msg);

		return true;
	}
};

#endif
