#pragma once

#include "ControllerTypes.hpp"
#include <cmath>

namespace actuator_output_validation
{
// Finite servo commands saturate independently. Servo saturation must not
// revoke valid motor commands, including at rounded AIRLINK endpoints.
// Invalid motors and non-finite commands remain hard failures.
inline bool validateAndLimit(ActuatorSetpoint &output)
{
	for (float value : output.motors) {
		if (!std::isfinite(value) || value < 0.f || value > 1.f) {
			return false;
		}
	}

	for (float value : output.servos) {
		if (!std::isfinite(value)) {
			return false;
		}
	}

	for (float &value : output.servos) {
		value = value < -1.f ? -1.f : (value > 1.f ? 1.f : value);
	}

	return true;
}
} // namespace actuator_output_validation
