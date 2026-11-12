#pragma once

#include "GsLinkProtocol.hpp"
#include <cmath>

namespace gs_link
{
// Kept independent of uORB so wire validation can be checked on the host.
inline bool validMocapSample(const MocapAttitudePayload &sample, uint64_t now,
			    uint32_t gs_session, bool heartbeat_fresh,
			    uint32_t last_session, uint32_t last_sequence, uint64_t last_sample_time)
{
	if (!heartbeat_fresh || sample.session_id == 0 || sample.session_id != gs_session
	    || sample.sample_time_us == 0
	    || (sample.sample_time_us > now && sample.sample_time_us - now > 10000)
	    || (now > sample.sample_time_us && now - sample.sample_time_us > 100000)
	    || !std::isfinite(sample.variance) || sample.variance < 1e-6f || sample.variance > 1.f
	    || (sample.session_id == last_session && static_cast<int32_t>(sample.sample_sequence - last_sequence) <= 0)) {
		return false;
	}

	float norm_squared = 0.f;

	for (int i = 0; i < 4; ++i) {
		if (!std::isfinite(sample.q[i])) { return false; }
		norm_squared += sample.q[i] * sample.q[i];
	}

	const uint64_t sample_time = sample.sample_time_us < now ? sample.sample_time_us : now;
	return std::fabs(std::sqrt(norm_squared) - 1.f) <= 1e-3f && sample_time > last_sample_time;
}
}
