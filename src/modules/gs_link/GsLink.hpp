/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "GsLinkProtocol.hpp"
#include "GsLinkDiagnostics.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/gs_link_control.h>
#include <uORB/topics/gs_link_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_odometry.h>

#include <uORB/topics/airlink_status.h>
#include <uORB/topics/airlink_health.h>
#include <uORB/topics/airlink_actuator_command.h>
#include <uORB/topics/airlink_pair_command.h>
#include <uORB/topics/composite_attitude_estimate.h>
#include <uORB/topics/composite_estimator_status.h>
#include <uORB/topics/composite_control_reference.h>
#include <uORB/topics/composite_control_status.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/logger_status.h>

#include <stddef.h>

using namespace time_literals;

class GsLink : public ModuleBase<GsLink>, public px4::ScheduledWorkItem
{
public:
	GsLink(const char *device, uint32_t baudrate);
	~GsLink() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void loadSubscriptions();
	bool openSerial();
	void readSerial();
	void parseRxBuffer();
	void handlePacket(const gs_link::Header &header, const uint8_t *payload);
	void handleMocapAttitude(const gs_link::Header &header, const uint8_t *payload);
	void handleRc(const gs_link::Header &header, const uint8_t *payload);
	void handleCommand(const gs_link::Header &header, const uint8_t *payload);
	void handleHeartbeat(const gs_link::Header &header, const uint8_t *payload);
	void sendTelemetry(hrt_abstime now);
	void sendDiagnostics(hrt_abstime now);
	void sendHeartbeat(hrt_abstime now);
	void sendAck(uint8_t acked_class, uint8_t acked_id, uint16_t acked_seq,
		     gs_link::AckResult result, uint32_t echo_token);
	bool sendPacket(gs_link::PacketClass packet_class, uint8_t message_id, uint8_t flags,
			const void *payload, uint16_t payload_len);
	void publishControl(bool valid, hrt_abstime now);
	uint16_t buildStatusFlags(hrt_abstime now, bool attitude_valid, bool motor_valid, bool power_valid);
	bool heartbeatFresh(hrt_abstime now) const;
	bool rcDataFresh(hrt_abstime now) const;
	void resyncRxBuffer();

	static constexpr hrt_abstime kTelemetryInterval{20_ms};
	static constexpr hrt_abstime kHeartbeatInterval{50_ms};
	static constexpr hrt_abstime kHeartbeatTimeout{500_ms};
	static constexpr hrt_abstime kRcChannelsTimeout{300_ms};
	static constexpr hrt_abstime kRcTimeout{250_ms};
	static constexpr hrt_abstime kEstimatorTimeout{100_ms};

	device::Serial _serial;
	uint8_t _rx_buffer[gs_link::kMaxPacket] {};
	size_t _rx_len{0};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _gs_link_status_sub{ORB_ID(gs_link_status)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<gs_link_control_s> _control_pub{ORB_ID(gs_link_control)};

	actuator_armed_s _actuator_armed{};
	battery_status_s _battery_status{};
	esc_status_s _esc_status{};
	failsafe_flags_s _failsafe_flags{};
	gs_link_status_s _gate_status{};
	rc_channels_s _rc_channels{};
	vehicle_attitude_s _vehicle_attitude{};

	uORB::Subscription _diag_airlink_status_sub{ORB_ID(airlink_status)};
	airlink_status_s _diag_airlink_status{};
	uORB::Subscription _diag_airlink_health_sub{ORB_ID(airlink_health)};
	airlink_health_s _diag_airlink_health{};
	uORB::Subscription _diag_airlink_actuator_command_sub{ORB_ID(airlink_actuator_command)};
	airlink_actuator_command_s _diag_airlink_actuator_command{};
	uORB::Subscription _diag_airlink_pair_command_sub{ORB_ID(airlink_pair_command)};
	airlink_pair_command_s _diag_airlink_pair_command{};
	uORB::Subscription _diag_composite_attitude_estimate_sub{ORB_ID(composite_attitude_estimate)};
	composite_attitude_estimate_s _diag_composite_attitude_estimate{};
	uORB::Subscription _diag_composite_estimator_status_sub{ORB_ID(composite_estimator_status)};
	composite_estimator_status_s _diag_composite_estimator_status{};
	uORB::Subscription _diag_composite_control_reference_sub{ORB_ID(composite_control_reference)};
	composite_control_reference_s _diag_composite_control_reference{};
	uORB::Subscription _diag_composite_control_status_sub{ORB_ID(composite_control_status)};
	composite_control_status_s _diag_composite_control_status{};
	uORB::Subscription _diag_actuator_servos_sub{ORB_ID(actuator_servos)};
	actuator_servos_s _diag_actuator_servos{};
	uORB::Subscription _diag_vehicle_status_sub{ORB_ID(vehicle_status)};
	vehicle_status_s _diag_vehicle_status{};
	uORB::Subscription _diag_vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	vehicle_angular_velocity_s _diag_vehicle_angular_velocity{};
	uORB::Subscription _diag_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	vehicle_acceleration_s _diag_vehicle_acceleration{};
	uORB::Subscription _diag_vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	vehicle_local_position_s _diag_vehicle_local_position{};
	uORB::Subscription _diag_manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _diag_manual_control_setpoint{};
	uORB::Subscription _diag_logger_status_sub{ORB_ID(logger_status)};
	logger_status_s _diag_logger_status{};
	uORB::Subscription _diag_actuator_motors_sub{ORB_ID(actuator_motors)};
	actuator_motors_s _diag_actuator_motors{};
	bool _pair_control_requested{false};
	uORB::Publication<vehicle_odometry_s> _mocap_pub{ORB_ID(vehicle_visual_odometry)};
	uint32_t _mocap_session{0};
	uint32_t _mocap_sequence{0};
	hrt_abstime _mocap_sample_time{0};
	uint8_t _mocap_reset_counter{0};
	uint32_t _session_id{0};
	uint32_t _gs_session_id{0};
	uint16_t _tx_sequence{0};
	// Receive diagnostics, written only by the owning work queue.
	uint32_t _read_calls{0}, _read_bytes{0}, _read_empty{0}, _read_eagain{0}, _read_errors{0};
	int _last_read_errno{0};
	int _rx_available_last{0}, _rx_available_max{0};
	uint32_t _rx_discarded_bytes{0}, _rx_buffer_overflows{0};
	uint32_t _rx_heartbeats{0}, _rx_sync_requests{0}, _tx_sync_replies{0}, _rx_unknown_data{0};
	hrt_abstime _last_rx_byte_time{0};
	uint8_t _rx_first_bytes[32]{}, _rx_last_bytes[32]{};
	uint8_t _rx_first_len{0}, _rx_last_len{0};
	uint32_t _rx_packets{0};
	uint32_t _tx_packets{0};
	uint32_t _crc_errors{0};
	uint32_t _length_errors{0};
	uint32_t _version_errors{0};
	uint32_t _unknown_commands{0};
	uint32_t _open_failures{0};
	uint32_t _write_failures{0};
	int _last_write_errno{0};
	uint8_t _last_error{0};

	hrt_abstime _last_telemetry{0};
	hrt_abstime _last_heartbeat{0};
	hrt_abstime _last_gs_heartbeat{0};
	hrt_abstime _last_rc_arrival{0};

	bool _rc_data_valid{false};
	float _rc_roll{0.f};
	float _rc_pitch{0.f};
	float _rc_yaw{0.f};
	float _rc_throttle{0.f};
	uint8_t _rc_link_quality{255};
	uint16_t _rc_sequence{0};
};

extern "C" __EXPORT int gs_link_main(int argc, char *argv[]);
