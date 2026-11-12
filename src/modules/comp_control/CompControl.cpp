#include "PairControlCore.hpp"
#include "PairWrench.hpp"
#include "../nn_control/LqiCoaxialCollectivePayload.hpp"
#include "../airlink/AirlinkProtocol.hpp"
#include "../airlink/PairEstimateSafety.hpp"
#include <uORB/topics/airlink_status.h>
#include <uORB/topics/actuator_armed.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/composite_attitude_estimate.h>
#include <uORB/topics/composite_control_reference.h>
#include <uORB/topics/airlink_pair_command.h>
#include <uORB/topics/composite_control_status.h>

class CompControl : public ModuleBase<CompControl>, public px4::ScheduledWorkItem
{
public:
	CompControl(): ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
		core(lqi_coaxial_collective::kModel) {}
	static int task_spawn(int, char *[])
	{
		auto *p = new CompControl();

		if (!p) {
			return PX4_ERROR;
		}

		// Gains and rotated inertia are generated for this geometry only.
		const char *names[2] = {"CEKF_MSTR_YAW", "CEKF_SLV_YAW"};

		for (int i = 0; i < 2; ++i) {
			float yaw = 0;

			if (param_get(param_find(names[i]), &yaw) != PX4_OK || !std::isfinite(yaw)
			    || fabsf(yaw - pair_gains::yaw[i] * 180.f / 3.14159265358979323846f) > .01f) {
				PX4_ERR("installation yaw differs from generated pair geometry");
				delete p;
				return PX4_ERROR;
			}
		}

		_object.store(p);
		_task_id = task_id_is_work_queue;
		p->ScheduleOnInterval(2000);
		return PX4_OK;
	}
	static int custom_command(int, char *[])
	{
		return print_usage();
	}
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_USAGE_NAME("comp_control", "controller");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}
private:
	void invalidate(uint64_t now)
	{
		core.reset();
		last_sample = 0;

		if (published) {
			airlink_pair_command_s msg{};
			msg.timestamp = now;
			pub.publish(msg);
			published = false;
		}
	}
	void Run() override
	{
		uint64_t now = hrt_absolute_time();

		if (should_exit()) {
			invalidate(now);
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		ref_sub.update(&ref);
		state_sub.update(&state);
		link_sub.update(&link);
		armed_sub.update(&armed);
		now = hrt_absolute_time();
		if (!armed.armed && link.operator_disarm_timestamp > acknowledged_disarm) {
			acknowledged_disarm = link.operator_disarm_timestamp;
			estimate_guard.acknowledge();
		}
		const bool protected_pair = link.role == airlink_status_s::ROLE_MASTER
		    && ((armed.armed && link.pair_mode_selected) || link.pair_active || link.local_arm_authorized);
		if (estimate_guard.update(protected_pair, state, now)) {
			invalidate(now);
			return;
		}
		constexpr uint64_t timeout = airlink::kSensorFreshTimeoutUs;

		if (!ref.enabled || !pair_estimate_safety::healthy(state, now) || !ref.timestamp || now < ref.timestamp || now - ref.timestamp > timeout
		    || !state.timestamp_sample || now < state.timestamp_sample || now - state.timestamp_sample > timeout
		    || !state.timestamp || now < state.timestamp || now - state.timestamp > timeout) {
			invalidate(now);
			return;
		}

		if (state.timestamp_sample == last_sample && state.reset_counter == reset_counter) {
			return;
		}

		bool restart = last_sample == 0 || state.reset_counter != reset_counter || state.timestamp_sample < last_sample;
		float dt = restart ? .002f : (state.timestamp_sample - last_sample) * 1e-6f;

		if (restart) {
			core.reset();
		}

		reset_counter = state.reset_counter;
		last_sample = state.timestamp_sample;
		PairControlCore::Input input{};
		float norm = 0, target_norm = 0;

		for (int i = 0; i < 4; ++i) {
			norm += state.q[i] * state.q[i];
			target_norm += ref.q[i] * ref.q[i];
		}

		if (!std::isfinite(norm) || !std::isfinite(target_norm) || fabsf(norm - 1.f) > .02f
		    || fabsf(target_norm - 1.f) > .02f) {
			invalidate(now);
			return;
		}

		for (int i = 0; i < 4; ++i) {
			input.attitude_q_wb[i] = state.q[i] / sqrtf(norm);
			input.target_attitude_q_wb[i] = ref.q[i] / sqrtf(target_norm);
		}

		for (int i = 0; i < 3; ++i) {
			input.angular_velocity_b[i] = state.angular_velocity[i];
			input.acceleration_b[i] = state.specific_force[i];
			input.target_angular_velocity_b[i] = ref.angular_velocity[i];
		}

		input.collective_base = ref.collective;
		PairControlCore::Output out{};

		if (!core.step(input, dt, out)) {
			invalidate(now);
			return;
		}

		airlink_pair_command_s msg{};
		msg.timestamp = now;
		msg.timestamp_sample = state.timestamp_sample;
		msg.valid = true;
		msg.collective = ref.collective;
		msg.master_motors[0] = out.a.upper;
		msg.master_motors[1] = out.a.lower;
		msg.follower_motors[0] = out.b.upper;
		msg.follower_motors[1] = out.b.lower;

		for (int i = 0; i < 3; ++i) {
			msg.master_servos[i] = out.a.servos[i];
			msg.follower_servos[i] = out.b.servos[i];
		}

		pub.publish(msg);
		published = true;
		composite_control_status_s status{};
		status.timestamp = now;
		status.timestamp_sample = state.timestamp_sample;
		status.state_age_us = now - state.timestamp_sample;

		for (int i = 0; i < 4; ++i) {
			status.requested_wrench[i] = out.requested[i];
			status.attitude[i] = state.q[i];
			status.target_attitude[i] = ref.q[i];
		}

		for (int i = 0; i < 3; ++i) {
			status.angular_velocity[i] = state.angular_velocity[i];
			status.target_angular_velocity[i] = ref.angular_velocity[i];
		}

		for (int i = 0; i < 2; ++i) {
			status.side_thrust[i] = out.side_thrust[i];
			status.yaw_scale[i] = out.yaw_scale[i];
		}

		status.servo_scale = out.servo_scale;
		const float *q = input.attitude_q_wb;
		status.vertical_acceleration = -(2 * (q[1] * q[3] - q[0] * q[2]) * input.acceleration_b[0] + 2 *
		                                 (q[2] * q[3] + q[0] * q[1]) * input.acceleration_b[1] + (1 - 2 * (q[1] * q[1] + q[2] * q[2])) * input.acceleration_b[2]
		                                 + lqi_coaxial_collective::kModel.gravity_m_s2);
		status.target_vertical_acceleration = (2 * ref.collective - 1) *
		                                      lqi_coaxial_collective::kModel.maximum_vertical_acceleration_m_s2;
		pair_plant::wrench(lqi_coaxial_collective::kModel, out.a, out.b, status.achieved_wrench, status.achieved_side_thrust);
		status.saturation_flags = out.saturation_flags;

		for (int i = 0; i < 4; ++i) {
			status.residual_wrench[i] = status.requested_wrench[i] - status.achieved_wrench[i];
		}

		for (int i = 0; i < 3; ++i) { status.attitude_error[i] = out.attitude_error[i]; }
		for (int i = 0; i < 3; ++i) { status.rate_error[i] = out.rate_error[i]; }
		for (int i = 0; i < 3; ++i) { status.attitude_integral[i] = out.attitude_integral[i]; }
		for (int i = 0; i < 3; ++i) { status.moment_attitude[i] = out.moment_attitude[i]; }
		for (int i = 0; i < 3; ++i) { status.moment_rate[i] = out.moment_rate[i]; }
		for (int i = 0; i < 3; ++i) { status.moment_integral[i] = out.moment_integral[i]; }
		for (int i = 0; i < 3; ++i) { status.acceleration_b[i] = out.acceleration_b[i]; }
		status.filtered_vertical_acceleration = out.filtered_vertical_acceleration;
		status.acceleration_error = out.acceleration_error;
		status.acceleration_integral = out.acceleration_integral;
		status.acceleration_pi_p = out.acceleration_pi_p;
		status.acceleration_pi_i = out.acceleration_pi_i;
		status.corrected_vertical_acceleration = out.corrected_vertical_acceleration;
		status.bounded_vertical_acceleration = out.bounded_vertical_acceleration;
		status.raw_thrust = out.raw_thrust;
		status.allocated_roll_moment = out.allocated_roll_moment;
		status.loop_dt_s = out.loop_dt_s;
		status.zero_throttle_threshold = out.zero_throttle_threshold;
		status.collective_input = input.collective_base;
		status.trace_flags = out.trace_flags;
		status.installation_yaw[0] = pair_gains::yaw[0];
		status.installation_yaw[1] = pair_gains::yaw[1];
		status.arm_m = pair_gains::arm;
		status_pub.publish(status);
	}
	uORB::Publication<composite_control_status_s> status_pub{ORB_ID(composite_control_status)};
	PairControlCore core;
	uORB::Subscription state_sub{ORB_ID(composite_attitude_estimate)}, ref_sub{ORB_ID(composite_control_reference)};
	uORB::Publication<airlink_pair_command_s> pub{ORB_ID(airlink_pair_command)};
	composite_attitude_estimate_s state{};
	composite_control_reference_s ref{};
	uint64_t last_sample{0};
	uint8_t reset_counter{0};
	bool published{false};
	pair_estimate_safety::Guard estimate_guard{};
	uORB::Subscription link_sub{ORB_ID(airlink_status)}, armed_sub{ORB_ID(actuator_armed)};
	airlink_status_s link{};
	actuator_armed_s armed{};
	uint64_t acknowledged_disarm{};
};
extern "C" __EXPORT int comp_control_main(int argc, char *argv[])
{
	return CompControl::main(argc, argv);
}
