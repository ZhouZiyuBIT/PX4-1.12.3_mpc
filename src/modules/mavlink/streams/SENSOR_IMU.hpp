
#ifndef SENSOR_IMU_HPP
#define SENSOR_IMU_HPP

#include <uORB/topics/sensor_combined.h>

class MavlinkStreamSensorIMU : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) {return new MavlinkStreamSensorIMU(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_IMU"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SENSOR_IMU; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SENSOR_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}


private:
	explicit MavlinkStreamSensorIMU(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};

	bool send() override
	{
		sensor_combined_s sensor_combined{};
		_sensor_combined_sub.copy(&sensor_combined);

		mavlink_sensor_imu_t msg{};
		msg.wx = sensor_combined.gyro_rad[0];
		msg.wy = sensor_combined.gyro_rad[1];
		msg.wz = sensor_combined.gyro_rad[2];
		msg.ax = sensor_combined.accelerometer_m_s2[0];
		msg.ay = sensor_combined.accelerometer_m_s2[1];
		msg.az = sensor_combined.accelerometer_m_s2[2];

		mavlink_msg_sensor_imu_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif

