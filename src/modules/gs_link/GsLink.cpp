/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "GsLink.hpp"
#include "GsMocap.hpp"
#if defined(CONFIG_STM32H7_USART1_RXDIAG)
# include <stm32_usart1_rxdiag.h>
#endif


#include <cmath>
#include <cstdlib>
#include <cstring>
#include <errno.h>

#include <mathlib/math/Functions.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

namespace
{

constexpr float kRadToCentiDegrees{18000.f / 3.14159265358979323846f};

int16_t clampCentiDegrees(float value)
{
	if (!PX4_ISFINITE(value)) {
		return 0;
	}

	return static_cast<int16_t>(math::constrain(lroundf(value), -18000L, 18000L));
}

} // namespace

GsLink::GsLink(const char *device, uint32_t baudrate) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_serial(device, baudrate)
{
}

GsLink::~GsLink()
{
	_serial.close();
}

bool GsLink::init()
{
	_session_id = static_cast<uint32_t>(hrt_absolute_time()) ^ 0x9E3779B9u;

	// Open and close the serial device from the work queue thread. NuttX file
	// descriptors are task-group scoped, so an fd opened by the NSH task cannot
	// be used from the hp_default worker.
	ScheduleDelayed(1);
	return true;
}

bool GsLink::openSerial()
{
	if (_serial.isOpen()) {
		return true;
	}

	if (!_serial.open()) {
		++_open_failures;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::InternalError);
		PX4_ERR("failed to open %s at %lu baud", _serial.getPort(),
			(unsigned long)_serial.getBaudrate());
		return false;
	}

	// A stop/start cycle can inherit a stale TX buffer from the previous
	// owner. Start every GS-Link session with clean UART buffers.
	_serial.flush();

	_rx_len = 0;
	_last_telemetry = 0;
	_last_heartbeat = 0;
	_last_gs_heartbeat = 0;
	_last_rc_arrival = 0;
	_rc_data_valid = false;
	_gs_session_id = 0;
	_last_error = 0;
	_last_write_errno = 0;

	PX4_INFO("GS-Link started on %s at %lu baud, session %08lx",
		 _serial.getPort(), (unsigned long)_serial.getBaudrate(),
		 (unsigned long)_session_id);
	return true;
}

void GsLink::Run()
{
	if (should_exit()) {
		ScheduleClear();
		publishControl(false, hrt_absolute_time());
		_serial.close();
		exit_and_cleanup();
		return;
	}

	if (!openSerial()) {
		ScheduleDelayed(1_s);
		return;
	}

	loadSubscriptions();
	readSerial();
	parseRxBuffer();

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_telemetry >= kTelemetryInterval) {
		sendTelemetry(now);
		sendDiagnostics(now);
		_last_telemetry = now;
		publishControl(rcDataFresh(now) && heartbeatFresh(now), now);
	}

	if (now - _last_heartbeat >= kHeartbeatInterval) {
		sendHeartbeat(now);
		_last_heartbeat = now;
	}

	ScheduleDelayed(1_ms);
}

void GsLink::loadSubscriptions()
{
	_actuator_armed_sub.update(&_actuator_armed);
	_battery_status_sub.update(&_battery_status);
	_esc_status_sub.update(&_esc_status);
	_failsafe_flags_sub.update(&_failsafe_flags);
	_gs_link_status_sub.update(&_gate_status);
	_rc_channels_sub.update(&_rc_channels);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_diag_airlink_status_sub.update(&_diag_airlink_status);
	_diag_airlink_health_sub.update(&_diag_airlink_health);
	_diag_airlink_actuator_command_sub.update(&_diag_airlink_actuator_command);
	_diag_airlink_pair_command_sub.update(&_diag_airlink_pair_command);
	_diag_composite_attitude_estimate_sub.update(&_diag_composite_attitude_estimate);
	_diag_composite_estimator_status_sub.update(&_diag_composite_estimator_status);
	_diag_composite_control_reference_sub.update(&_diag_composite_control_reference);
	_diag_composite_control_status_sub.update(&_diag_composite_control_status);
	_diag_actuator_servos_sub.update(&_diag_actuator_servos);
	_diag_vehicle_status_sub.update(&_diag_vehicle_status);
	_diag_vehicle_angular_velocity_sub.update(&_diag_vehicle_angular_velocity);
	_diag_vehicle_acceleration_sub.update(&_diag_vehicle_acceleration);
	_diag_vehicle_local_position_sub.update(&_diag_vehicle_local_position);
	_diag_manual_control_setpoint_sub.update(&_diag_manual_control_setpoint);
	_diag_logger_status_sub.update(&_diag_logger_status);
	_diag_actuator_motors_sub.update(&_diag_actuator_motors);
}

void GsLink::readSerial()
{
	uint8_t buffer[256];
	// FIONREAD is sampled in the same worker that owns this NuttX descriptor.
	errno = 0;
	_rx_available_last = static_cast<int>(_serial.bytesAvailable());
	_rx_available_max = math::max(_rx_available_max, _rx_available_last);

	while (true) {
		++_read_calls;
		errno = 0;
		const ssize_t received = _serial.read(buffer, sizeof(buffer));
		const int read_errno = errno;

		if (received <= 0) {
			if (received == 0) { ++_read_empty; }
			else if (read_errno == EAGAIN) { ++_read_eagain; }
			else { ++_read_errors; _last_read_errno = read_errno; }
			break;
		}

		_read_bytes += static_cast<uint32_t>(received);
		_last_rx_byte_time = hrt_absolute_time();
		_rx_last_len = static_cast<uint8_t>(math::min(static_cast<size_t>(received), sizeof(_rx_last_bytes)));
		memcpy(_rx_last_bytes, buffer + received - _rx_last_len, _rx_last_len);
		if (_rx_first_len == 0) {
			_rx_first_len = static_cast<uint8_t>(math::min(static_cast<size_t>(received), sizeof(_rx_first_bytes)));
			memcpy(_rx_first_bytes, buffer, _rx_first_len);
		}

		const size_t available = sizeof(_rx_buffer) - _rx_len;
		const size_t copy = math::min(static_cast<size_t>(received), available);
		memcpy(_rx_buffer + _rx_len, buffer, copy);
		_rx_len += copy;

		if (copy < static_cast<size_t>(received)) {
			++_rx_buffer_overflows;
			_rx_discarded_bytes += static_cast<uint32_t>(received - copy);
			// The parser will consume the valid prefix; drop one byte to make
			// room and resynchronize instead of permanently filling the buffer.
			resyncRxBuffer();
			break;
		}
	}
}

void GsLink::resyncRxBuffer()
{
	size_t start = 0;

	while (start + 1 < _rx_len
	       && !(_rx_buffer[start] == gs_link::kMagic0 && _rx_buffer[start + 1] == gs_link::kMagic1)) {
		++start;
	}

	if (start >= _rx_len) {
		_rx_discarded_bytes += _rx_len;
		_rx_len = 0;

	} else if (start > 0) {
		_rx_discarded_bytes += start;
		memmove(_rx_buffer, _rx_buffer + start, _rx_len - start);
		_rx_len -= start;
	}
}

void GsLink::parseRxBuffer()
{
	while (_rx_len >= gs_link::kHeaderSize + gs_link::kCrcSize) {
		if (_rx_buffer[0] != gs_link::kMagic0 || _rx_buffer[1] != gs_link::kMagic1) {
			resyncRxBuffer();
			continue;
		}

		gs_link::Header header{};
		memcpy(&header, _rx_buffer, sizeof(header));

		if (header.version != gs_link::kVersion) {
			++_version_errors;
			_last_error = static_cast<uint8_t>(gs_link::AckResult::BadVersion);
			memmove(_rx_buffer, _rx_buffer + 1, _rx_len - 1);
			--_rx_len;
			continue;
		}

		if (header.payload_len > gs_link::kMaxPayload) {
			++_length_errors;
			_last_error = static_cast<uint8_t>(gs_link::AckResult::BadLength);
			memmove(_rx_buffer, _rx_buffer + 1, _rx_len - 1);
			--_rx_len;
			continue;
		}

		const size_t total = gs_link::kHeaderSize + header.payload_len + gs_link::kCrcSize;

		if (_rx_len < total) {
			break;
		}

		uint16_t expected_crc{0};
		memcpy(&expected_crc, _rx_buffer + gs_link::kHeaderSize + header.payload_len,
		       sizeof(expected_crc));
		const uint16_t actual_crc = gs_link::crc16(_rx_buffer + 2,
					  gs_link::kHeaderSize - 2 + header.payload_len);

		if (actual_crc != expected_crc) {
			++_crc_errors;
			_last_error = 1;
			memmove(_rx_buffer, _rx_buffer + 1, _rx_len - 1);
			--_rx_len;
			continue;
		}

		handlePacket(header, _rx_buffer + gs_link::kHeaderSize);
		++_rx_packets;

		memmove(_rx_buffer, _rx_buffer + total, _rx_len - total);
		_rx_len -= total;
	}
}

void GsLink::handlePacket(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.packet_class == static_cast<uint8_t>(gs_link::PacketClass::Data)) {
		if (header.message_id == static_cast<uint8_t>(gs_link::DataId::Rc)) {
			handleRc(header, payload);
		} else if (header.message_id == static_cast<uint8_t>(gs_link::DataId::MocapAttitude)) {
			handleMocapAttitude(header, payload);
		} else if (header.message_id == static_cast<uint8_t>(gs_link::DataId::TimeSync)
			   && header.flags == 0 && header.payload_len == sizeof(uint64_t)) {
			++_rx_sync_requests;
			gs_link::TimeSyncResponse reply{};
			reply.fc_receive_us = hrt_absolute_time();
			memcpy(&reply.ground_send_ns, payload, sizeof(uint64_t));
			reply.fc_transmit_us = hrt_absolute_time();
			if (sendPacket(gs_link::PacketClass::Data, static_cast<uint8_t>(gs_link::DataId::TimeSync),
				       gs_link::Flags::Response, &reply, sizeof(reply))) { ++_tx_sync_replies; }
		} else {
			++_rx_unknown_data;
		}

		return;
	}

	if (header.packet_class == static_cast<uint8_t>(gs_link::PacketClass::Command)) {
		handleCommand(header, payload);
	}
}

void GsLink::handleMocapAttitude(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.payload_len != sizeof(gs_link::MocapAttitudePayload)) {
		++_length_errors;
		return;
	}

	gs_link::MocapAttitudePayload sample{};
	memcpy(&sample, payload, sizeof(sample));
	const hrt_abstime now = hrt_absolute_time();
	const matrix::Quatf q(sample.q[0], sample.q[1], sample.q[2], sample.q[3]);
	const bool new_session = sample.session_id != _mocap_session;

	if (header.flags != 0 || !gs_link::validMocapSample(sample, now, _gs_session_id, heartbeatFresh(now),
			_mocap_session, _mocap_sequence, _mocap_sample_time)) {
		return;
	}

	const hrt_abstime sample_time = sample.sample_time_us < now ? sample.sample_time_us : now;

	if (new_session) { ++_mocap_reset_counter; }

	_mocap_session = sample.session_id;
	_mocap_sequence = sample.sample_sequence;
	_mocap_sample_time = sample_time;
	vehicle_odometry_s odom{};
	odom.timestamp = now;
	odom.timestamp_sample = sample_time;
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
	q.normalized().copyTo(odom.q);

	for (int i = 0; i < 3; ++i) {
		odom.position[i] = NAN;
		odom.velocity[i] = NAN;
		odom.angular_velocity[i] = NAN;
		odom.position_variance[i] = NAN;
		odom.velocity_variance[i] = NAN;
		odom.orientation_variance[i] = sample.variance;
	}

	odom.reset_counter = _mocap_reset_counter;
	odom.quality = 100;
	_mocap_pub.publish(odom);
}

void GsLink::handleRc(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.payload_len != sizeof(gs_link::RcPayload)) {
		++_length_errors;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadLength);
		return;
	}

	gs_link::RcPayload rc{};
	memcpy(&rc, payload, sizeof(rc));

	const bool valid = (rc.flags & gs_link::RcFlags::RcValid)
			   && rc.roll >= -10000 && rc.roll <= 10000
			   && rc.pitch >= -10000 && rc.pitch <= 10000
			   && rc.yaw >= -10000 && rc.yaw <= 10000
			   && rc.throttle >= -10000 && rc.throttle <= 10000;

	if (!valid) {
		_rc_data_valid = false;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadArgument);
		publishControl(false, hrt_absolute_time());
		return;
	}

	_rc_roll = static_cast<float>(rc.roll) / 10000.f;
	_rc_pitch = static_cast<float>(rc.pitch) / 10000.f;
	_rc_yaw = static_cast<float>(rc.yaw) / 10000.f;
	_rc_throttle = static_cast<float>(rc.throttle) / 10000.f;
	_rc_link_quality = rc.link_quality;
	_rc_sequence = header.sequence;
	_last_rc_arrival = hrt_absolute_time();
	_rc_data_valid = true;
	_pair_control_requested = (rc.flags & gs_link::RcFlags::PairControl) != 0;
	_last_error = 0;
	publishControl(heartbeatFresh(_last_rc_arrival), _last_rc_arrival);
}

void GsLink::handleHeartbeat(const gs_link::Header &header, const uint8_t *payload)
{
	if (header.payload_len != sizeof(gs_link::HeartbeatPayload)) {
		++_length_errors;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadLength);
		return;
	}

	gs_link::HeartbeatPayload heartbeat{};
	memcpy(&heartbeat, payload, sizeof(heartbeat));

	if (heartbeat.protocol_version != gs_link::kVersion) {
		++_version_errors;
		_last_error = static_cast<uint8_t>(gs_link::AckResult::BadVersion);
		return;
	}

	++_rx_heartbeats;
	_gs_session_id = heartbeat.session_id;
	_last_gs_heartbeat = hrt_absolute_time();
	_last_error = heartbeat.error_code;
}

void GsLink::handleCommand(const gs_link::Header &header, const uint8_t *payload)
{
	const auto command = static_cast<gs_link::CommandId>(header.message_id);

	switch (command) {
	case gs_link::CommandId::Ping: {
			if (header.payload_len != sizeof(gs_link::PingPayload)) {
				++_length_errors;
				sendAck(header.packet_class, header.message_id, header.sequence,
					gs_link::AckResult::BadLength, 0);
				return;
			}

			gs_link::PingPayload ping{};
			memcpy(&ping, payload, sizeof(ping));
			sendAck(header.packet_class, header.message_id, header.sequence,
				gs_link::AckResult::Ok, ping.token);
			return;
		}

	case gs_link::CommandId::Heartbeat:
		handleHeartbeat(header, payload);
		return;

	case gs_link::CommandId::Takeoff:
	case gs_link::CommandId::Land:
	case gs_link::CommandId::Abort: {
			// Communication-only firmware: validate the command envelope but
			// never enter a takeoff/landing state or touch the actuators.
			const size_t expected = command == gs_link::CommandId::Takeoff
						? sizeof(gs_link::TakeoffPayload)
						: command == gs_link::CommandId::Land
						? sizeof(gs_link::LandPayload)
						: sizeof(gs_link::AbortPayload);

			if (header.payload_len != expected) {
				++_length_errors;
				sendAck(header.packet_class, header.message_id, header.sequence,
					gs_link::AckResult::BadLength, 0);
				return;
			}

			uint32_t session_id{0};
			uint32_t token{0};

			if (command == gs_link::CommandId::Takeoff) {
				gs_link::TakeoffPayload takeoff{};
				memcpy(&takeoff, payload, sizeof(takeoff));
				session_id = takeoff.session_id;
				token = takeoff.command_token;

			} else if (command == gs_link::CommandId::Land) {
				gs_link::LandPayload land{};
				memcpy(&land, payload, sizeof(land));
				session_id = land.session_id;
				token = land.command_token;

			} else {
				gs_link::AbortPayload abort{};
				memcpy(&abort, payload, sizeof(abort));
				session_id = abort.session_id;
				token = abort.command_token;
			}

			sendAck(header.packet_class, header.message_id, header.sequence,
				session_id == _session_id ? gs_link::AckResult::Unsupported
				: gs_link::AckResult::StaleSession, token);
			return;
		}

	case gs_link::CommandId::Ack:
		return;

	default:
		++_unknown_commands;
		sendAck(header.packet_class, header.message_id, header.sequence,
			gs_link::AckResult::Unsupported, 0);
		return;
	}
}

void GsLink::sendAck(uint8_t acked_class, uint8_t acked_id, uint16_t acked_seq,
		     gs_link::AckResult result, uint32_t echo_token)
{
	gs_link::AckPayload ack{};
	ack.acked_class = acked_class;
	ack.acked_id = acked_id;
	ack.acked_seq = acked_seq;
	ack.result = static_cast<uint8_t>(result);
	ack.echo_token = echo_token;
	sendPacket(gs_link::PacketClass::Command, static_cast<uint8_t>(gs_link::CommandId::Ack),
		   gs_link::Flags::Response, &ack, sizeof(ack));
}

bool GsLink::sendPacket(gs_link::PacketClass packet_class, uint8_t message_id, uint8_t flags,
			const void *payload, uint16_t payload_len)
{
	if (payload_len > gs_link::kMaxPayload) {
		return false;
	}

	uint8_t packet[gs_link::kMaxPacket] {};
	gs_link::Header header{};
	header.version = gs_link::kVersion;
	header.packet_class = static_cast<uint8_t>(packet_class);
	header.message_id = message_id;
	header.flags = flags;
	header.sequence = _tx_sequence++;
	header.payload_len = payload_len;
	memcpy(packet, &header, sizeof(header));

	if (payload_len > 0) {
		memcpy(packet + gs_link::kHeaderSize, payload, payload_len);
	}

	const uint16_t crc = gs_link::crc16(packet + 2, gs_link::kHeaderSize - 2 + payload_len);
	memcpy(packet + gs_link::kHeaderSize + payload_len, &crc, sizeof(crc));

	const size_t total = gs_link::kHeaderSize + payload_len + gs_link::kCrcSize;
	size_t sent = 0;
	int retries = 0;
	int write_errno = 0;

	while (sent < total && retries < 4) {
		const ssize_t written = _serial.write(packet + sent, total - sent);

		if (written > 0) {
			sent += static_cast<size_t>(written);
			retries = 0;

		} else if (written < 0 && errno == EAGAIN) {
			write_errno = errno;
			++retries;
			px4_usleep(50);

		} else {
			write_errno = written < 0 ? errno : EIO;
			break;
		}
	}

	if (sent == total) {
		++_tx_packets;
		_last_error = 0;
		_last_write_errno = 0;
		return true;
	}

	++_write_failures;
	_last_write_errno = write_errno;
	_last_error = static_cast<uint8_t>(gs_link::AckResult::InternalError);
	return false;
}

uint16_t GsLink::buildStatusFlags(hrt_abstime now, bool attitude_valid, bool motor_valid,
				  bool power_valid)
{
	uint16_t flags{0};

	if (_actuator_armed.armed) {
		flags |= gs_link::StatusFlags::Armed;
	}

	if (_gate_status.gs_mode_active) {
		flags |= gs_link::StatusFlags::GsModeActive;
	}

	if (_gate_status.rc_valid) {
		flags |= gs_link::StatusFlags::RcValidStatus;
	}

	if (_gate_status.rc_switch_selected) {
		flags |= gs_link::StatusFlags::RcSwitchSelected;
	}

	if (_gate_status.rc_switch_valid) {
		flags |= gs_link::StatusFlags::RcSwitchValid;
	}

	if (_gate_status.failsafe || _failsafe_flags.manual_control_signal_lost
	    || _failsafe_flags.battery_unhealthy || _failsafe_flags.fd_critical_failure) {
		flags |= gs_link::StatusFlags::Failsafe;
	}

	if (attitude_valid) {
		flags |= gs_link::StatusFlags::AttitudeValid;
	}

	if (motor_valid) {
		flags |= gs_link::StatusFlags::MotorRpmValid;
	}

	if (power_valid) {
		flags |= gs_link::StatusFlags::PowerValid;
	}

	if (_battery_status.warning >= battery_status_s::WARNING_LOW) {
		flags |= gs_link::StatusFlags::BatteryWarning;
	}

	if (_gate_status.command_busy) {
		flags |= gs_link::StatusFlags::CommandBusy;
	}

	if (_gate_status.takeoff_active) {
		flags |= gs_link::StatusFlags::TakeoffActive;
	}

	if (_gate_status.landing_active) {
		flags |= gs_link::StatusFlags::LandingActive;
	}

	if (heartbeatFresh(now) && rcDataFresh(now)) {
		flags |= gs_link::StatusFlags::LinkOk;
	}

	if (_rc_channels.channel_count > 0 && !_rc_channels.signal_lost
	    && now >= _rc_channels.timestamp
	    && now - _rc_channels.timestamp <= kRcChannelsTimeout) {
		flags |= gs_link::StatusFlags::RcChannelsValid;
	}

	return flags;
}

void GsLink::sendTelemetry(hrt_abstime now)
{
	gs_link::TelemetryPayload telemetry{};
	telemetry.timestamp_ms = static_cast<uint32_t>(now / 1000);

	bool attitude_valid = false;

	if (_vehicle_attitude.timestamp_sample != 0
	    && now >= _vehicle_attitude.timestamp_sample
	    && now - _vehicle_attitude.timestamp_sample <= kEstimatorTimeout) {
		const matrix::Quatf q(_vehicle_attitude.q);

		if (PX4_ISFINITE(q(0)) && PX4_ISFINITE(q(1)) && PX4_ISFINITE(q(2)) && PX4_ISFINITE(q(3))
		    && q.norm_squared() > 0.25f) {
			const matrix::Eulerf euler(q);

			if (PX4_ISFINITE(euler.phi()) && PX4_ISFINITE(euler.theta())
			    && PX4_ISFINITE(euler.psi())) {
				telemetry.roll_cdeg = clampCentiDegrees(euler.phi() * kRadToCentiDegrees);
				// Protocol telemetry uses nose-down positive, matching the manual
				// pitch-stick sign; PX4 Euler pitch is nose-up positive.
				telemetry.pitch_cdeg = clampCentiDegrees(-euler.theta() * kRadToCentiDegrees);
				telemetry.yaw_cdeg = clampCentiDegrees(euler.psi() * kRadToCentiDegrees);
				attitude_valid = true;
			}
		}
	}

	bool motor_found[2] {false, false};

	if (_esc_status.timestamp != 0 && now >= _esc_status.timestamp
	    && now - _esc_status.timestamp <= kEstimatorTimeout) {
		for (int esc_index = 0; esc_index < _esc_status.esc_count
		     && esc_index < esc_status_s::CONNECTED_ESC_MAX; ++esc_index) {
			if ((_esc_status.esc_online_flags & (1u << esc_index)) == 0) {
				continue;
			}

			const int motor_index = static_cast<int>(_esc_status.esc[esc_index].actuator_function)
						- static_cast<int>(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1);

			if (motor_index >= 0 && motor_index < 2) {
				const float rpm = static_cast<float>(_esc_status.esc[esc_index].esc_rpm);

				if (PX4_ISFINITE(rpm) && rpm >= 0.f) {
					const uint16_t value = static_cast<uint16_t>(
								       math::constrain(lroundf(rpm), 0L, 65534L));
					telemetry.motor1_rpm = motor_index == 0 ? value : telemetry.motor1_rpm;
					telemetry.motor2_rpm = motor_index == 1 ? value : telemetry.motor2_rpm;
					motor_found[motor_index] = true;
				}
			}
		}
	}

	bool power_valid = false;

	if (_battery_status.timestamp != 0 && now >= _battery_status.timestamp
	    && now - _battery_status.timestamp <= 1_s) {
		if (PX4_ISFINITE(_battery_status.voltage_v) && _battery_status.voltage_v > 0.f) {
			telemetry.battery_voltage_mv = static_cast<uint16_t>(
							       math::constrain(lroundf(_battery_status.voltage_v * 1000.f), 0L, 65535L));
			power_valid = true;
		}

		if (PX4_ISFINITE(_battery_status.current_a)) {
			telemetry.battery_current_ca = static_cast<int16_t>(
							       math::constrain(lroundf(_battery_status.current_a * 100.f), -32767L, 32767L));
			power_valid = true;
		}

		if (PX4_ISFINITE(_battery_status.remaining) && _battery_status.remaining >= 0.f) {
			telemetry.battery_remaining = static_cast<uint8_t>(
							      math::constrain(lroundf(_battery_status.remaining), 0L, 100L));
		}
	}

	uint8_t rc_count = 0;

	if (_rc_channels.timestamp != 0 && now >= _rc_channels.timestamp
	    && now - _rc_channels.timestamp <= kRcChannelsTimeout && !_rc_channels.signal_lost) {
		rc_count = math::min<uint8_t>(_rc_channels.channel_count, gs_link::kMaxRcChannels);
	}

	telemetry.rc_channel_count = rc_count;
	telemetry.status_flags = buildStatusFlags(now, attitude_valid,
				 motor_found[0] && motor_found[1], power_valid);

	uint8_t payload[gs_link::kMaxPayload] {};
	memcpy(payload, &telemetry, sizeof(telemetry));

	for (uint8_t index = 0; index < rc_count; ++index) {
		const float value = _rc_channels.channels[index];
		int8_t encoded = -128;

		if (PX4_ISFINITE(value)) {
			encoded = static_cast<int8_t>(
					  math::constrain(lroundf(math::constrain(value, -1.f, 1.f) * 127.f),
							  -127L, 127L));
		}

		payload[sizeof(telemetry) + index] = static_cast<uint8_t>(encoded);
	}

	sendPacket(gs_link::PacketClass::Data, static_cast<uint8_t>(gs_link::DataId::Telemetry),
		   0, payload, sizeof(telemetry) + rc_count);
}

void GsLink::sendDiagnostics(hrt_abstime now)
{
	const auto fresh = [now](uint64_t stamp, uint64_t timeout = 100000) {
		return stamp != 0 && now >= stamp && now - stamp <= timeout;
	};
	const auto &link = _diag_airlink_status;
	const auto &health = _diag_airlink_health;
	const auto &vehicle = _diag_vehicle_status;
	const auto &logger = _diag_logger_status;
	gs_link::NodeDiagnostics node{};
	node.role = link.role;
	node.nav_state = vehicle.nav_state;
	node.control_source = link.follower_active ? 2 : (_gate_status.gs_mode_active ? 1 : 0);
	node.timestamp_us = now;
	node.airlink_timestamp_us = link.timestamp;
	node.flags = (_actuator_armed.armed ? 1u : 0u)
		| ((_actuator_armed.manual_lockdown || _actuator_armed.lockdown) ? 2u : 0u)
		| (link.pair_mode_selected ? 4u : 0u) | (link.pair_active ? 8u : 0u)
		| (link.follower_active ? 16u : 0u) | (link.pair_lost ? 32u : 0u)
		| (link.pair_reattach_latched ? 64u : 0u) | (link.peer_heartbeat_fresh ? 128u : 0u)
		| (link.peer_sensor_fresh ? 256u : 0u) | (link.peer_control_fresh ? 512u : 0u)
		| (_gate_status.rc_valid ? 1024u : 0u) | (_gate_status.gs_mode_active ? 2048u : 0u)
		| (rcDataFresh(now) && heartbeatFresh(now) ? 4096u : 0u)
		| (fresh(link.timestamp, 10000) ? 8192u : 0u)
		| (fresh(logger.timestamp, 2000000) ? 16384u : 0u);
	node.session_id = link.session_id;
	node.peer_session_id = link.peer_session_id;
	node.airlink_state = link.link_state;
	node.airlink_version = link.protocol_version;
	node.peer_health_flags = link.peer_health_flags;
	node.peer_sensor_flags = link.peer_sensor_status_flags;
	node.airlink_rx = link.rx_frames;
	node.airlink_tx = link.tx_frames;
	node.airlink_crc_errors = link.crc_errors;
	node.airlink_session_errors = link.session_errors;
	node.airlink_retries = link.reliable_retries;
	node.airlink_write_failures = link.write_failures;
	node.gs_rx = _rx_packets;
	node.gs_tx = _tx_packets;
	node.gs_crc_errors = _crc_errors;
	node.gs_write_failures = _write_failures;
	node.logger_timestamp_us = logger.timestamp;
	node.logger_flags = (logger.is_logging ? 1u : 0u) | ((logger.backend & 1) ? 2u : 0u);
	node.logger_dropouts = logger.dropouts;
	node.logger_message_gaps = logger.message_gaps;
	node.logger_total_kb = logger.total_written_kb;
	node.logger_rate_kb_s = logger.write_rate_kb_s;
	node.health_flags = (health.health_flags & ~128u) | (fresh(health.timestamp, 10000) ? 128u : 0u);
	node.run_interval_us = health.run_interval_us;
	node.run_interval_max_us = health.run_interval_max_us;
	node.run_over_10ms = health.run_over_10ms;
	node.run_work_us = health.run_work_us;
	node.run_work_max_us = health.run_work_max_us;
	node.rx_interval_us = health.rx_interval_us;
	node.rx_interval_max_us = health.rx_interval_max_us;
	node.rx_over_10ms = health.rx_over_10ms;
	node.tx_interval_us = health.tx_interval_us;
	node.tx_interval_max_us = health.tx_interval_max_us;
	node.tx_over_10ms = health.tx_over_10ms;
	node.rx_age_us = health.rx_age_us;
	node.rx_age_max_us = health.rx_age_max_us;
	node.stream_timeouts = health.stream_timeouts;
	node.peer_sensor_age_us = health.peer_sensor_age_us;
	node.peer_control_age_us = health.peer_control_age_us;
	node.peer_link_age_us = health.peer_link_age_us;
	node.peer_imu_invalid_packets = health.peer_imu_invalid_packets;
	node.peer_imu_invalid_events = health.peer_imu_invalid_events;
	node.local_imu_age_us = health.local_imu_age_us;
	node.local_imu_age_max_us = health.local_imu_age_max_us;
	node.rtt_us = health.rtt_us;
	node.rtt_max_us = health.rtt_max_us;
	node.rtt_age_us = health.rtt_age_us;
	node.rtt_samples = health.rtt_samples;
	node.rtt_timeouts = health.rtt_timeouts;
	node.rtt_over_10ms = health.rtt_over_10ms;
	node.length_errors = health.length_errors;
	node.version_errors = health.version_errors;
	node.eof_errors = health.eof_errors;
	node.resyncs = health.resyncs;
	node.partial_timeouts = health.partial_timeouts;
	node.buffer_overflows = health.buffer_overflows;
	node.unknown_messages = health.unknown_messages;
	node.local_core_age_us = health.local_core_age_us;
	node.local_core_age_max_us = health.local_core_age_max_us;
	node.local_core_stale_events = health.local_core_stale_events;


	sendPacket(gs_link::PacketClass::Data, gs_link::kNodeDiagnosticsId, 0, &node, sizeof(node));

	const auto &state = _diag_composite_attitude_estimate;
	const auto &estimator = _diag_composite_estimator_status;
	if (link.role == airlink_status_s::ROLE_MASTER && state.timestamp != 0) {
		gs_link::ObserverDiagnostics obs{};
		obs.role = link.role;
		obs.estimator_state = estimator.state;
		obs.reset_counter = state.reset_counter;
		obs.timestamp_us = state.timestamp;
		obs.sample_us = state.timestamp_sample;
		obs.status_timestamp_us = estimator.timestamp;
		obs.flags = (state.valid ? 1u : 0u) | (estimator.enabled ? 2u : 0u)
			| (estimator.attitude_initialized ? 4u : 0u) | (estimator.peer_connected ? 8u : 0u)
			| (fresh(estimator.timestamp) ? 16u : 0u) | (fresh(state.timestamp, 10000) ? 32u : 0u)
			| (static_cast<uint32_t>(state.health_flags) << 16);
		obs.fault_reason = estimator.fault_reason;
		obs.fault_epoch = estimator.fault_epoch;
		obs.source_age_a_us = state.source_age_a_us;
		obs.source_age_b_us = state.source_age_b_us;
		obs.frame_count = estimator.frame_count;
		obs.rejected_frames = estimator.rejected_frames;
		for (int i = 0; i < 4; ++i) { obs.q[i] = state.q[i]; }
		for (int i = 0; i < 3; ++i) {
			obs.angular_velocity[i] = state.angular_velocity[i];
			obs.angular_acceleration[i] = state.angular_acceleration[i];
			obs.specific_force[i] = state.specific_force[i];
		}
		obs.weights[0] = estimator.weight_a;
		obs.weights[1] = estimator.weight_b;
		obs.yaw_sigma_rad = state.yaw_sigma_rad;
		sendPacket(gs_link::PacketClass::Data, gs_link::kObserverDiagnosticsId, 0, &obs, sizeof(obs));
	}
	const auto &status = _diag_composite_control_status;
	const auto &reference = _diag_composite_control_reference;
	if (link.role == airlink_status_s::ROLE_MASTER && (status.timestamp != 0 || reference.timestamp != 0)) {
		gs_link::ControllerDiagnostics control{};
		control.role = link.role;
		control.timestamp_us = status.timestamp;
		control.sample_us = status.timestamp_sample;
		control.reference_timestamp_us = reference.timestamp;
		control.state_age_us = status.state_age_us;
		control.saturation_flags = status.saturation_flags;
		control.enabled = (reference.enabled ? 1u : 0u) | (fresh(status.timestamp, 10000) ? 2u : 0u)
			| (fresh(reference.timestamp, 10000) ? 4u : 0u)
			| (_diag_airlink_pair_command.valid && fresh(_diag_airlink_pair_command.timestamp, 10000) ? 8u : 0u);
		for (int i = 0; i < 4; ++i) { control.requested_wrench[i] = status.requested_wrench[i]; }
		for (int i = 0; i < 4; ++i) { control.achieved_wrench[i] = status.achieved_wrench[i]; }
		for (int i = 0; i < 4; ++i) { control.residual_wrench[i] = status.residual_wrench[i]; }
		for (int i = 0; i < 2; ++i) { control.side_thrust[i] = status.side_thrust[i]; }
		for (int i = 0; i < 2; ++i) { control.achieved_side_thrust[i] = status.achieved_side_thrust[i]; }
		for (int i = 0; i < 2; ++i) { control.yaw_scale[i] = status.yaw_scale[i]; }
		for (int i = 0; i < 4; ++i) { control.attitude[i] = status.attitude[i]; }
		for (int i = 0; i < 4; ++i) { control.target_attitude[i] = status.target_attitude[i]; }
		for (int i = 0; i < 3; ++i) { control.angular_velocity[i] = status.angular_velocity[i]; }
		for (int i = 0; i < 3; ++i) { control.target_angular_velocity[i] = status.target_angular_velocity[i]; }
		control.servo_scale = status.servo_scale;
		control.vertical_acceleration = status.vertical_acceleration;
		control.target_vertical_acceleration = status.target_vertical_acceleration;
		for (int i = 0; i < 4; ++i) { control.reference_q[i] = reference.q[i]; }
		for (int i = 0; i < 3; ++i) { control.reference_rates[i] = reference.angular_velocity[i]; }
		control.reference_collective = status.collective_input;
		for (int i = 0; i < 3; ++i) { control.attitude_error[i] = status.attitude_error[i]; }
		for (int i = 0; i < 3; ++i) { control.rate_error[i] = status.rate_error[i]; }
		for (int i = 0; i < 3; ++i) { control.attitude_integral[i] = status.attitude_integral[i]; }
		for (int i = 0; i < 3; ++i) { control.moment_attitude[i] = status.moment_attitude[i]; }
		for (int i = 0; i < 3; ++i) { control.moment_rate[i] = status.moment_rate[i]; }
		for (int i = 0; i < 3; ++i) { control.moment_integral[i] = status.moment_integral[i]; }
		for (int i = 0; i < 3; ++i) { control.acceleration_b[i] = status.acceleration_b[i]; }
		control.filtered_vertical_acceleration = status.filtered_vertical_acceleration;
		control.acceleration_error = status.acceleration_error;
		control.acceleration_integral = status.acceleration_integral;
		control.acceleration_pi_p = status.acceleration_pi_p;
		control.acceleration_pi_i = status.acceleration_pi_i;
		control.corrected_vertical_acceleration = status.corrected_vertical_acceleration;
		control.bounded_vertical_acceleration = status.bounded_vertical_acceleration;
		control.raw_thrust = status.raw_thrust;
		control.allocated_roll_moment = status.allocated_roll_moment;
		for (int i = 0; i < 2; ++i) { control.installation_yaw[i] = status.installation_yaw[i]; }
		control.arm_m = status.arm_m;
		control.loop_dt_s = status.loop_dt_s;
		control.zero_throttle_threshold = status.zero_throttle_threshold;
		control.trace_flags = status.trace_flags;
		sendPacket(gs_link::PacketClass::Data, gs_link::kControllerDiagnosticsId, 0, &control, sizeof(control));
	}
	const auto &motors = _diag_actuator_motors;
	const auto &servos = _diag_actuator_servos;
	const auto &received = _diag_airlink_actuator_command;
	const auto &pair = _diag_airlink_pair_command;
	const auto &rates = _diag_vehicle_angular_velocity;
	const auto &accel = _diag_vehicle_acceleration;
	const auto &position = _diag_vehicle_local_position;
	const auto &manual = _diag_manual_control_setpoint;
	gs_link::FlightDiagnostics flight{};
	flight.role = link.role;
	flight.timestamp_us = now;
	flight.attitude_sample_us = _vehicle_attitude.timestamp_sample;
	flight.motors_timestamp_us = motors.timestamp;
	flight.servos_timestamp_us = servos.timestamp;
	flight.command_timestamp_us = link.role == airlink_status_s::ROLE_SLAVE ? received.timestamp : pair.timestamp;
	flight.flags = (_actuator_armed.armed ? 1u : 0u) | (fresh(motors.timestamp, 10000) ? 2u : 0u)
		| (fresh(servos.timestamp, 10000) ? 4u : 0u)
		| (received.valid && fresh(received.timestamp, 10000) ? 8u : 0u)
		| (pair.valid && fresh(pair.timestamp, 10000) ? 16u : 0u)
		| (position.xy_valid && position.z_valid && fresh(position.timestamp) ? 32u : 0u)
		| (fresh(_vehicle_attitude.timestamp_sample) ? 64u : 0u)
		| (fresh(rates.timestamp_sample) ? 128u : 0u) | (fresh(accel.timestamp_sample) ? 256u : 0u)
		| (manual.valid && fresh(manual.timestamp_sample, 500000) ? 512u : 0u)
		| (rcDataFresh(now) && heartbeatFresh(now) ? 1024u : 0u);
	for (int i = 0; i < 4; ++i) { flight.q[i] = _vehicle_attitude.q[i]; }
	for (int i = 0; i < 3; ++i) {
		flight.angular_velocity[i] = rates.xyz[i];
		flight.acceleration[i] = accel.xyz[i];
		flight.servos[i] = servos.control[i];
		flight.received_servos[i] = received.servos[i];
		flight.pair_master[i + 2] = pair.master_servos[i];
		flight.pair_follower[i + 2] = pair.follower_servos[i];
	}
	for (int i = 0; i < 2; ++i) {
		flight.motors[i] = motors.control[i];
		flight.received_motors[i] = received.motors[i];
		flight.pair_master[i] = pair.master_motors[i];
		flight.pair_follower[i] = pair.follower_motors[i];
	}
	flight.position[0] = position.x; flight.position[1] = position.y; flight.position[2] = position.z;
	flight.velocity[0] = position.vx; flight.velocity[1] = position.vy; flight.velocity[2] = position.vz;
	flight.rc[0] = manual.roll; flight.rc[1] = manual.pitch; flight.rc[2] = manual.yaw; flight.rc[3] = manual.throttle;
	flight.gs_command[0] = _rc_roll; flight.gs_command[1] = _rc_pitch;
	flight.gs_command[2] = _rc_yaw; flight.gs_command[3] = _rc_throttle;
	flight.voltage_v = _battery_status.voltage_v;
	flight.current_a = _battery_status.current_a;
	flight.remaining = _battery_status.remaining;
	flight.local_position_timestamp_us = position.timestamp;
	flight.rates_timestamp_us = rates.timestamp_sample;
	flight.acceleration_timestamp_us = accel.timestamp_sample;
	flight.received_timestamp_us = received.timestamp;
	flight.pair_timestamp_us = pair.timestamp;
	flight.rc_timestamp_us = manual.timestamp_sample;
	flight.gs_timestamp_us = _last_rc_arrival;
	sendPacket(gs_link::PacketClass::Data, gs_link::kFlightDiagnosticsId, 0, &flight, sizeof(flight));
}

void GsLink::sendHeartbeat(hrt_abstime now)
{
	gs_link::HeartbeatPayload heartbeat{};
	heartbeat.uptime_ms = static_cast<uint32_t>(now / 1000);
	heartbeat.session_id = _session_id;
	heartbeat.protocol_version = gs_link::kVersion;
	heartbeat.status_flags = buildStatusFlags(now, false, false, false);
	heartbeat.state = _gate_status.gs_mode_active
			  ? static_cast<uint8_t>(gs_link::LinkState::Auto)
			  : static_cast<uint8_t>(gs_link::LinkState::Ready);
	heartbeat.error_code = _last_error;
	sendPacket(gs_link::PacketClass::Command,
		   static_cast<uint8_t>(gs_link::CommandId::Heartbeat), 0,
		   &heartbeat, sizeof(heartbeat));
}

void GsLink::publishControl(bool valid, hrt_abstime now)
{
	gs_link_control_s control{};
	control.timestamp = now;
	control.timestamp_sample = _last_rc_arrival;
	control.valid = valid && _rc_data_valid;
	control.heartbeat_ok = heartbeatFresh(now);
	control.pair_control = _pair_control_requested;
	control.roll = _rc_roll;
	control.pitch = _rc_pitch;
	control.yaw = _rc_yaw;
	control.throttle = _rc_throttle;
	control.link_quality = _rc_link_quality;
	control.sequence = _rc_sequence;
	control.session_id = _session_id;
	_control_pub.publish(control);
}

bool GsLink::heartbeatFresh(hrt_abstime now) const
{
	return _last_gs_heartbeat != 0 && now >= _last_gs_heartbeat
	       && now - _last_gs_heartbeat <= kHeartbeatTimeout;
}

bool GsLink::rcDataFresh(hrt_abstime now) const
{
	return _rc_data_valid && _last_rc_arrival != 0 && now >= _last_rc_arrival
	       && now - _last_rc_arrival <= kRcTimeout;
}

int GsLink::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("port: %s, baud: %lu, open: %d",
		 _serial.getPort(), (unsigned long)_serial.getBaudrate(),
		 _serial.isOpen() ? 1 : 0);
#if defined(CONFIG_ARCH_BOARD_MICOAIR_H743_V2)
	// Read-only hardware snapshot. Never read RDR/ICR: that would consume or
	// clear receive state. Addresses match this board's STM32H743 USART1/PA10.
	if (_serial.isOpen() && strcmp(_serial.getPort(), "/dev/ttyS0") == 0) {
		PX4_INFO("UART RX diagnostic revision: ISR-QUEUE-1");
#if defined(CONFIG_STM32H7_USART1_RXDIAG)
		struct stm32_usart1_rxdiag_s driver{};
		stm32_usart1_rxdiag_get(&driver);
		PX4_INFO("USART1 IRQ/RXNE/service/RDR/queued: %lu/%lu/%lu/%lu/%lu",
			 (unsigned long)driver.irq_calls, (unsigned long)driver.rxne_seen,
			 (unsigned long)driver.rx_service_calls, (unsigned long)driver.rdr_reads,
			 (unsigned long)driver.queued_bytes);
		PX4_INFO("USART1 errors FE/NE/ORE: %lu/%lu/%lu, queue-full events: %lu",
			 (unsigned long)driver.framing_errors, (unsigned long)driver.noise_errors,
			 (unsigned long)driver.overrun_errors, (unsigned long)driver.full_events);
		PX4_INFO("USART1 driver IE/last RX ISR: %08lx/%08lx, queue head/tail/size: %lu/%lu/%lu",
			 (unsigned long)driver.driver_ie, (unsigned long)driver.last_rx_isr,
			 (unsigned long)driver.queue_head, (unsigned long)driver.queue_tail, (unsigned long)driver.queue_size);
		PX4_INFO("USART1 driver baud/kernel clock: %lu/%lu Hz",
			 (unsigned long)driver.driver_baud, (unsigned long)driver.driver_clock);
#else
		PX4_INFO("USART1 IRQ counters not enabled in this build");
#endif
		const auto reg = [](uintptr_t address) { return *reinterpret_cast<volatile const uint32_t *>(address); };
		const uint32_t cr1 = reg(0x40011000u), cr2 = reg(0x40011004u), cr3 = reg(0x40011008u);
		const uint32_t brr = reg(0x4001100cu), isr = reg(0x4001101cu);
		const uint32_t mode = reg(0x58020000u), pull = reg(0x5802000cu);
		const uint32_t input = reg(0x58020010u), af = reg(0x58020024u);
		// USART1 is external IRQ37: ISER1/ISPR1/IABR1 bit5.
		const uint32_t enabled = reg(0xe000e104u), pending = reg(0xe000e204u), active = reg(0xe000e304u);
		PX4_INFO("USART1 CR1/CR2/CR3: %08lx/%08lx/%08lx, BRR: %08lx, ISR: %08lx",
			 (unsigned long)cr1, (unsigned long)cr2, (unsigned long)cr3, (unsigned long)brr, (unsigned long)isr);
		PX4_INFO("USART1 UE/RE/RXIE: %u/%u/%u, RXNE/ORE/FE/NE: %u/%u/%u/%u",
			 (unsigned)(cr1 & 1u), (unsigned)((cr1 >> 2) & 1u), (unsigned)((cr1 >> 5) & 1u),
			 (unsigned)((isr >> 5) & 1u), (unsigned)((isr >> 3) & 1u),
			 (unsigned)((isr >> 1) & 1u), (unsigned)((isr >> 2) & 1u));
		PX4_INFO("USART1 SWAP/RXINV/HDSEL/DMAR: %u/%u/%u/%u, IRQ37 EN/PEND/ACTIVE: %u/%u/%u",
			 (unsigned)((cr2 >> 15) & 1u), (unsigned)((cr2 >> 16) & 1u),
			 (unsigned)((cr3 >> 3) & 1u), (unsigned)((cr3 >> 6) & 1u),
			 (unsigned)((enabled >> 5) & 1u), (unsigned)((pending >> 5) & 1u), (unsigned)((active >> 5) & 1u));
		PX4_INFO("PA10 mode/AF/pull/input: %u/%u/%u/%u (expected mode=2 AF=7)",
			 (unsigned)((mode >> 20) & 3u), (unsigned)((af >> 8) & 15u),
			 (unsigned)((pull >> 20) & 3u), (unsigned)((input >> 10) & 1u));
	}
#endif
	PX4_INFO("session: %08lx, rx/tx packets: %lu/%lu",
		 (unsigned long)_session_id, (unsigned long)_rx_packets,
		 (unsigned long)_tx_packets);
	PX4_INFO("RX read calls/bytes: %lu/%lu, empty/eagain/errors: %lu/%lu/%lu, errno: %d",
		 (unsigned long)_read_calls, (unsigned long)_read_bytes, (unsigned long)_read_empty,
		 (unsigned long)_read_eagain, (unsigned long)_read_errors, _last_read_errno);
	PX4_INFO("RX pending last/max: %d/%d, parser buffered: %u, discarded: %lu, overflows: %lu",
		 _rx_available_last, _rx_available_max, (unsigned)_rx_len,
		 (unsigned long)_rx_discarded_bytes, (unsigned long)_rx_buffer_overflows);
	const hrt_abstime last_rx = _last_rx_byte_time;
	PX4_INFO("RX byte age: %llu ms, heartbeats: %lu, sync requests/replies: %lu/%lu, unknown DATA: %lu",
		 (unsigned long long)(last_rx == 0 || last_rx > now ? 0 : (now - last_rx) / 1000),
		 (unsigned long)_rx_heartbeats, (unsigned long)_rx_sync_requests,
		 (unsigned long)_tx_sync_replies, (unsigned long)_rx_unknown_data);
	char hex[3 * 32 + 1]{};
	for (int which = 0; which < 2; ++which) {
		const unsigned count = math::min(static_cast<unsigned>(which == 0 ? _rx_first_len : _rx_last_len), 32u);
		const uint8_t *bytes = which == 0 ? _rx_first_bytes : _rx_last_bytes;
		for (unsigned i = 0; i < count; ++i) { snprintf(hex + 3 * i, sizeof(hex) - 3 * i, "%02x ", bytes[i]); }
		if (count == 0) { hex[0] = '\0'; }
		PX4_INFO("RX %s %u bytes: %s", which == 0 ? "first" : "last", count, hex);
	}
	PX4_INFO("crc/length/version errors: %lu/%lu/%lu",
		 (unsigned long)_crc_errors, (unsigned long)_length_errors,
		 (unsigned long)_version_errors);
	PX4_INFO("link: heartbeat %s (%llu ms), RC %s (%llu ms), GS heartbeat session %08lx",
		 heartbeatFresh(now) ? "OK" : "stale",
		 (unsigned long long)(_last_gs_heartbeat == 0 ? 0
				      : hrt_elapsed_time(&_last_gs_heartbeat) / 1000),
		 rcDataFresh(now) ? "OK" : "stale",
		 (unsigned long long)(_last_rc_arrival == 0 ? 0
				      : hrt_elapsed_time(&_last_rc_arrival) / 1000),
		 (unsigned long)_gs_session_id);
	PX4_INFO("last RC: seq %u, quality %u, axes %.3f %.3f %.3f %.3f, channels %u",
		 (unsigned)_rc_sequence, (unsigned)_rc_link_quality,
		 (double)_rc_roll, (double)_rc_pitch, (double)_rc_yaw, (double)_rc_throttle,
		 (unsigned)_rc_channels.channel_count);
	PX4_INFO("unknown commands: %lu, last error: %u, write failures: %lu, last write errno: %d",
		 (unsigned long)_unknown_commands, (unsigned)_last_error,
		 (unsigned long)_write_failures, _last_write_errno);
	PX4_INFO("mocap: session %08lx, seq %lu, sample age %llu ms",
		 (unsigned long)_mocap_session, (unsigned long)_mocap_sequence,
		 (unsigned long long)(_mocap_sample_time == 0 || _mocap_sample_time > now ? 0 : (now - _mocap_sample_time) / 1000));
	PX4_INFO("serial open failures: %lu", (unsigned long)_open_failures);
	return 0;
}

int GsLink::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS0";
	uint32_t baudrate = 4000000;

	for (int i = 1; i < argc; ++i) {
		if (!strcmp(argv[i], "-d") && i + 1 < argc) {
			device = argv[++i];

		} else if (!strcmp(argv[i], "-b") && i + 1 < argc) {
			baudrate = static_cast<uint32_t>(strtoul(argv[++i], nullptr, 10));
		}
	}

	GsLink *instance = new GsLink(device, baudrate);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int GsLink::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GsLink::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GS-Link v1 serial transport for the RC-supervised ground-station link.
This build is communication-only: takeoff, land and abort commands are
acknowledged as unsupported and never change the vehicle state.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gs_link", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gs_link_main(int argc, char *argv[])
{
	return GsLink::main(argc, argv);
}
