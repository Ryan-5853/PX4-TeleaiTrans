#pragma once
#include "../nn_control/LqiCoaxialCollectiveCore.hpp"
#include "PairGains.hpp"


// Independent C-frame pair loop. No ownership, RC or link state machine.
class PairControlCore
{
public:
	using Input = LqiControllerCore::Input;
	struct Output {
		LqiControllerCore::ControlOutput a{}, b{};
		float requested[4] {}, side_thrust[2] {};
		float attitude_error[3] {};
		float rate_error[3] {};
		float attitude_integral[3] {};
		float moment_attitude[3] {};
		float moment_rate[3] {};
		float moment_integral[3] {};
		float acceleration_b[3] {};
		float filtered_vertical_acceleration {};
		float acceleration_error {};
		float acceleration_integral {};
		float acceleration_pi_p {};
		float acceleration_pi_i {};
		float corrected_vertical_acceleration {};
		float bounded_vertical_acceleration {};
		float raw_thrust {};
		float allocated_roll_moment {};
		float loop_dt_s {};
		float zero_throttle_threshold {};
		uint32_t trace_flags{0};
		uint8_t saturation_flags{0};
		float servo_scale{1.f}, yaw_scale[2] {1.f, 1.f};
	};
	explicit PairControlCore(const LqiCoaxialCollectiveModel &m): model(m) {}
	void reset()
	{
		for (float &v : persistent) {
			v = 0.f;
		}
		initialize_filter = true;
	}
	bool step(const Input &input, float dt, Output &output)
	{
		output = {};

		if (!std::isfinite(dt) || dt <= 0.f || dt > .01f) {
			reset();
			return false;
		}

		for (int i = 0; i < 4; ++i) {
			if (!std::isfinite(input.attitude_q_wb[i]) || !std::isfinite(input.target_attitude_q_wb[i])) {
				reset();
				return false;
			}
		}

		for (int i = 0; i < 3; ++i) {
			if (!std::isfinite(input.acceleration_b[i]) || !std::isfinite(input.angular_velocity_b[i])
			    || !std::isfinite(input.target_angular_velocity_b[i])) {
				reset();
				return false;
			}
		}

		if (!std::isfinite(input.collective_base) || input.collective_base < 0.f || input.collective_base > 1.f) {
			reset();
			return false;
		}

		float next_persistent[5] {};
		const float tw = input.target_attitude_q_wb[0];
		const float tx = -input.target_attitude_q_wb[1];
		const float ty = -input.target_attitude_q_wb[2];
		const float tz = -input.target_attitude_q_wb[3];
		const float cw = input.attitude_q_wb[0];
		const float cx = input.attitude_q_wb[1];
		const float cy = input.attitude_q_wb[2];
		const float cz = input.attitude_q_wb[3];
		float w = tw * cw - tx * cx - ty * cy - tz * cz;
		float vx = tw * cx + tx * cw + ty * cz - tz * cy;
		float vy = tw * cy - tx * cz + ty * cw + tz * cx;
		float vz = tw * cz + tx * cy - ty * cx + tz * cw;

		if (w < 0.0f) {
			w = -w;
			vx = -vx;
			vy = -vy;
			vz = -vz;
		}

		const float vnorm = std::sqrt(vx * vx + vy * vy + vz * vz);
		const float vnorm_clamped = (vnorm < 1e-12f) ? 1e-12f : vnorm;
		const float angle = 2.0f * std::atan2(vnorm, (w < 1e-12f) ? 1e-12f : w);
		const float rotation_scale = angle / vnorm_clamped;
		const float roll_error = vx * rotation_scale;
		const float pitch_error = vy * rotation_scale;
		const float rate_error[3] {
			input.angular_velocity_b[0] - input.target_angular_velocity_b[0],
			input.angular_velocity_b[1] - input.target_angular_velocity_b[1],
			input.angular_velocity_b[2] - input.target_angular_velocity_b[2],
		};

		using lqi_coaxial_collective_detail::clamp;
		using lqi_coaxial_collective_detail::inverse_lookup;
		using lqi_coaxial_collective_detail::interpolate_vector;
		const float collective = clamp(input.collective_base, 0.0f, 1.0f);
		const bool zero_throttle = collective <= model.zero_throttle_threshold;
		output.attitude_error[0] = roll_error;
		output.attitude_error[1] = pitch_error;
		output.attitude_error[2] = vz * rotation_scale;
		for (int i = 0; i < 3; ++i) {
			output.rate_error[i] = rate_error[i];
			output.acceleration_b[i] = input.acceleration_b[i];
		}
		output.loop_dt_s = dt;
		output.zero_throttle_threshold = model.zero_throttle_threshold;
		output.trace_flags = zero_throttle ? 3u : 1u;


		if (zero_throttle) {
			reset();
			return true;
		}

		const float qw = input.attitude_q_wb[0];
		const float qx = input.attitude_q_wb[1];
		const float qy = input.attitude_q_wb[2];
		const float qz = input.attitude_q_wb[3];
		const float rotation_31 = 2.0f * (qx * qz - qw * qy);
		const float rotation_32 = 2.0f * (qy * qz + qw * qx);
		const float rotation_33 = 1.0f - 2.0f * (qx * qx + qy * qy);
		const float specific_force_z_ned =
		        rotation_31 * input.acceleration_b[0]
		        + rotation_32 * input.acceleration_b[1]
		        + rotation_33 * input.acceleration_b[2];
		const float measured_up_acceleration =
		        -(specific_force_z_ned + model.gravity_m_s2);
		if (initialize_filter) {
			persistent[0] = measured_up_acceleration;
			initialize_filter = false;
		}
		const float filter_alpha =
		        model.acceleration_filter_time_constant_s > 0.0f
		        ? dt
		        / (model.acceleration_filter_time_constant_s + dt)
		        : 1.0f;
		const float filtered_up_acceleration =
		        persistent[0]
		        + filter_alpha * (measured_up_acceleration - persistent[0]);
		const float normalized_collective =
		        2.0f * clamp(
		                (collective - model.pilot_minimum)
		                / (model.pilot_maximum - model.pilot_minimum),
		                0.0f, 1.0f)
		        - 1.0f;
		const float target_up_acceleration =
		        normalized_collective * model.maximum_vertical_acceleration_m_s2;
		const float acceleration_error =
		        target_up_acceleration - filtered_up_acceleration;
		const float candidate_acceleration_integral =
		        clamp(
		                persistent[1] + acceleration_error * dt,
		                -model.acceleration_integral_limit_m_s2,
		                model.acceleration_integral_limit_m_s2);
		const float corrected_up_acceleration =
		        target_up_acceleration
		        + model.acceleration_kp * acceleration_error
		        + model.acceleration_ki * candidate_acceleration_integral;
		const float acceleration_limit =
		        1.5f * model.maximum_vertical_acceleration_m_s2
		        + model.acceleration_integral_limit_m_s2;
		const float bounded_up_acceleration =
		        clamp(
		                corrected_up_acceleration,
		                -model.gravity_m_s2 - acceleration_limit,
		                acceleration_limit);
		const float tilt_cosine =
		        fmaxf(1.0f - 2.0f * (qx * qx + qy * qy), model.tilt_cosine_minimum);
		const float desired_thrust =
		        clamp(
		                2.0f * model.mass_kg * (model.gravity_m_s2 + bounded_up_acceleration)
		                / tilt_cosine,
		                2.0f * model.minimum_thrust_fraction * model.maximum_thrust_n,
		                2.0f * model.maximum_thrust_fraction * model.maximum_thrust_n);

		const float errors[3] {roll_error, pitch_error, rate_error[2]};
		float integral[3], state[8];

		for (int i = 0; i < 3; ++i) {
			integral[i] = clamp(persistent[i + 2] + errors[i] * dt, -pair_gains::integral_limit[i], pair_gains::integral_limit[i]);
		}

		state[0] = roll_error;
		state[1] = pitch_error;

		for (int i = 0; i < 3; ++i) {
			state[i + 2] = rate_error[i];
			state[i + 5] = integral[i];
		}

		float moment[3] {};

		for (int i = 0; i < 3; ++i) for (int j = 0; j < 8; ++j) {
				moment[i] -= pair_gains::gain[i * 8 + j] * state[j];
			}

		for (int i = 0; i < 3; ++i) {
			output.attitude_integral[i] = integral[i];
			for (int j = 0; j < 2; ++j) output.moment_attitude[i] -= pair_gains::gain[i * 8 + j] * state[j];
			for (int j = 2; j < 5; ++j) output.moment_rate[i] -= pair_gains::gain[i * 8 + j] * state[j];
			for (int j = 5; j < 8; ++j) output.moment_integral[i] -= pair_gains::gain[i * 8 + j] * state[j];
		}
		output.filtered_vertical_acceleration = filtered_up_acceleration;
		output.acceleration_error = acceleration_error;
		output.acceleration_integral = candidate_acceleration_integral;
		output.acceleration_pi_p = model.acceleration_kp * acceleration_error;
		output.acceleration_pi_i = model.acceleration_ki * candidate_acceleration_integral;
		output.corrected_vertical_acceleration = corrected_up_acceleration;
		output.bounded_vertical_acceleration = bounded_up_acceleration;
		output.raw_thrust = 2 * model.mass_kg * (model.gravity_m_s2 + bounded_up_acceleration) / tilt_cosine;
		output.requested[0] = desired_thrust;

		for (int i = 0; i < 3; ++i) {
			output.requested[i + 1] = moment[i];
		}

		const float tmin = model.minimum_thrust_fraction * model.maximum_thrust_n;
		const float tmax = model.maximum_thrust_fraction * model.maximum_thrust_n;
		const float roll_limit = pair_gains::arm * fminf(desired_thrust - 2 * tmin, 2 * tmax - desired_thrust);
		float mx = clamp(moment[0], -roll_limit, roll_limit);
		output.allocated_roll_moment = mx;
		output.side_thrust[0] = desired_thrust * .5f - mx / (2 * pair_gains::arm);
		output.side_thrust[1] = desired_thrust * .5f + mx / (2 * pair_gains::arm);
		float servo_raw[2][3] {};
		float servo_peak = 1.f;

		for (int side = 0; side < 2; ++side) {
			auto &cmd = side == 0 ? output.a : output.b;
			float t = output.side_thrust[side];
			float ratio = sqrtf(model.motor_torque_coefficients[0] / model.motor_torque_coefficients[1]);
			float denom = model.thrust_coefficients[0] + model.thrust_coefficients[1] * ratio * ratio + model.thrust_coefficients[2]
			              * ratio;
			float speed = sqrtf(t / denom);
			float base[2] {inverse_lookup(speed, model.upper_command_to_rad_s, model.upper_motor_table_points), inverse_lookup(speed * ratio, model.lower_command_to_rad_s, model.lower_motor_table_points)};
			float eff[4] {};
			interpolate_vector(model.motor_effectiveness_points, model.motor_effectiveness, model.motor_effectiveness_points_count,
			                   4, .5f * (base[0] + base[1]), eff);
			float det = eff[0] * eff[3] - eff[1] * eff[2];

			if (fabsf(det) < 1e-12f) {
				reset();
				return false;
			}

			float delta[2] {-eff[1] *moment[2] *.5f / det, eff[0] *moment[2] *.5f / det};
			float scale = 1.f;

			for (int i = 0; i < 2; ++i) if (fabsf(delta[i]) > 1e-12f) {
					scale = fminf(scale, delta[i] > 0 ? (1 - base[i]) / delta[i] : -base[i] / delta[i]);
				}

			scale = clamp(scale, 0.f, 1.f);
			output.yaw_scale[side] = scale;
			cmd.upper = clamp(base[0] + scale * delta[0], 0.f, 1.f);
			cmd.lower = clamp(base[1] + scale * delta[1], 0.f, 1.f);
			// R_Ci^T [0,My/2,0]: local X = sin(yaw)*My/2.
			float local_x = sinf(pair_gains::yaw[side]) * moment[1] * .5f;
			float local_y = cosf(pair_gains::yaw[side]) * moment[1] * .5f;
			float authority = clamp(t / (model.mass_kg * model.gravity_m_s2) * model.servo_effectiveness_scale,
			                        model.servo_effectiveness_scale_minimum, model.servo_effectiveness_scale_maximum);

			for (int i = 0; i < 3; ++i) {
				servo_raw[side][i] = (model.servo_direction[2 * i] * local_x + model.servo_direction[2 * i + 1] * local_y) / authority;
				servo_peak = fmaxf(servo_peak, fabsf(servo_raw[side][i]));
			}
		}

		output.servo_scale = 1.f / servo_peak;

		for (int i = 0; i < 3; ++i) {
			output.a.servos[i] = servo_raw[0][i] / servo_peak;
			output.b.servos[i] = servo_raw[1][i] / servo_peak;
		}

		next_persistent[0] = filtered_up_acceleration;
		float raw_thrust = 2 * model.mass_kg * (model.gravity_m_s2 + corrected_up_acceleration) / tilt_cosine;
		if (fabsf(raw_thrust - desired_thrust) > 1e-5f) output.saturation_flags |= 1;
		if (fabsf(moment[0] - mx) > 1e-6f) output.saturation_flags |= 2;
		if (output.servo_scale < 1.f) output.saturation_flags |= 4;
		if (output.yaw_scale[0] < 1.f) output.saturation_flags |= 8;
		if (output.yaw_scale[1] < 1.f) output.saturation_flags |= 16;
		next_persistent[1] = ((raw_thrust - desired_thrust) * acceleration_error > 0) ? persistent[1] :
		                     candidate_acceleration_integral;
		float achieved[3] {mx, moment[1] *output.servo_scale, moment[2] *fminf(output.yaw_scale[0], output.yaw_scale[1])};

		for (int i = 0; i < 3; ++i) {
			// Integrating positive error requests negative moment.
			next_persistent[i + 2] = ((moment[i] - achieved[i]) * (-errors[i]) > 0) ? persistent[i + 2] : integral[i];
		}

		for (int i = 0; i < 5; ++i) {
			persistent[i] = next_persistent[i];
		}

		const LqiControllerCore::ControlOutput *commands[2] {&output.a, &output.b};

		for (const auto *cmd : commands) {
			if (!std::isfinite(cmd->upper) || !std::isfinite(cmd->lower)) {
				reset();
				return false;
			}

			for (float v : cmd->servos) if (!std::isfinite(v)) {
					reset();
					return false;
				}
		}

		return true;
	}
private:
	const LqiCoaxialCollectiveModel &model;
	float persistent[5] {};
	bool initialize_filter{true};
};
