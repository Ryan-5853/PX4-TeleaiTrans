#pragma once
#include "PairPlant.hpp"
#include "PairGains.hpp"
#include "../nn_control/LqiCoaxialCollectiveCore.hpp"

namespace pair_plant
{
inline float lookup(float x, const float *table, int n)
{
	float points[10] {}, values[10] {};

	for (int i = 0; i < n; ++i) {
		points[i] = table[2 * i];
		values[i] = table[2 * i + 1];
	}

	float out{};
	lqi_coaxial_collective_detail::interpolate_vector(points, values, n, 1, x, &out);
	return out;
}
inline void cross(const float *a, const float *b, float *out)
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}
// Exact steady unit equations for the baseline's neutral -Z, zero CG and
// direct-thrust arm. Includes grid attenuation and r_i x R_Ci F_i.
inline void wrench(const LqiCoaxialCollectiveModel &model,
                   const LqiControllerCore::ControlOutput &a, const LqiControllerCore::ControlOutput &b, float out[4],
                   float *side_thrust = nullptr)
{
	for (int i = 0; i < 4; ++i) {
		out[i] = 0;
	}

	const LqiControllerCore::ControlOutput *commands[2] {&a, &b};

	for (int side = 0; side < 2; ++side) {
		const auto &cmd = *commands[side];
		float upper = lookup(cmd.upper, model.upper_command_to_rad_s, model.upper_motor_table_points);
		float lower = lookup(cmd.lower, model.lower_command_to_rad_s, model.lower_motor_table_points);
		float t = model.thrust_coefficients[0] * upper * upper + model.thrust_coefficients[1] * lower * lower +
		          model.thrust_coefficients[2] * upper * lower;
		float f[3] {0, 0, -t *partition[0]}, m[3] {0, 0, model.motor_torque_coefficients[0] *upper *upper - model.motor_torque_coefficients[1] *lower * lower};
		float angle[3] {}, attenuation[3] {};

		for (int i = 0; i < 3; ++i) {
			angle[i] = lookup(cmd.servos[i], servo_table, 3);
			attenuation[i] = lookup(fabsf(angle[i]), attenuation_table, 2);
		}

		for (int i = 0; i < 3; ++i) {
			float effective = attenuation[i];

			for (int j = 0; j < 3; ++j) {
				effective -= coupling[3 * i + j] * (1 - attenuation[j]);
			}

			effective = lqi_coaxial_collective_detail::clamp(effective, 0.f, 1.f);
			float deflection = angle[i] * vector_gain[i];
			float grid[3] {-axis[3 * i + 1] *sinf(deflection), axis[3 * i] *sinf(deflection), -cosf(deflection)};

			for (int j = 0; j < 3; ++j) {
				grid[j] *= t * partition[i + 1] * effective;
				f[j] += grid[j];
			}

			float gm[3] {};
			cross(&grid_arm[3 * i], grid, gm);

			for (int j = 0; j < 3; ++j) {
				m[j] += gm[j];
			}
		}

		float c = cosf(pair_gains::yaw[side]), s = sinf(pair_gains::yaw[side]);
		float fc[3] {c *f[0] - s *f[1], s *f[0] + c *f[1], f[2]};
		float mc[3] {c *m[0] - s *m[1], s *m[0] + c *m[1], m[2]};
		float arm[3] {0, side == 0 ? pair_gains::arm : -pair_gains::arm, 0}, lever[3] {};
		cross(arm, fc, lever);
		out[0] -= fc[2];
		if (side_thrust) side_thrust[side] = -fc[2];

		for (int j = 0; j < 3; ++j) {
			out[j + 1] += mc[j] + lever[j];
		}
	}
}
}
