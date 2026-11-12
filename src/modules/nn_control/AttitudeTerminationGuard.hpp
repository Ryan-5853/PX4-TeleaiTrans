#pragma once

#include <cmath>

// A local collective-controller stop, independent of Commander hysteresis.
// It must run before the thrust/tilt compensation and remain latched until
// disarm. Missing/invalid attitude is handled by the existing input gates.
class AttitudeTerminationGuard
{
public:
	static constexpr float kLimitRad{0.5235987755982988f}; // 30 degrees per axis

	bool update(bool armed, bool enabled, const float q_wb[4])
	{
		if (!armed) {
			_latched = false;
			return false;
		}

		if (_latched || !enabled) {
			return _latched;
		}

		float norm_squared = 0.f;

		for (int i = 0; i < 4; ++i) {
			if (!std::isfinite(q_wb[i])) {
				return false;
			}

			norm_squared += q_wb[i] * q_wb[i];
		}

		if (!std::isfinite(norm_squared) || norm_squared < 1e-12f) {
			return false;
		}

		const float inverse_norm = 1.f / std::sqrt(norm_squared);
		const float w = q_wb[0] * inverse_norm;
		const float x = q_wb[1] * inverse_norm;
		const float y = q_wb[2] * inverse_norm;
		const float z = q_wb[3] * inverse_norm;
		const float roll = std::atan2(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y));
		const float sin_pitch = 2.f * (w * y - z * x);
		const float pitch = std::asin(sin_pitch < -1.f ? -1.f : (sin_pitch > 1.f ? 1.f : sin_pitch));
		_latched = std::fabs(roll) >= kLimitRad || std::fabs(pitch) >= kLimitRad;
		return _latched;
	}

private:
	bool _latched{false};
};
