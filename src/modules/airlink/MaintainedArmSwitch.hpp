#pragma once
#include <cstdint>

namespace airlink
{
// A maintained OFF/ON switch, independent of ManualControl's button setting.
// Only a new OFF->ON edge can request arm. Startup/reconnect while ON cannot.
// OFF acknowledges disarm once; holding OFF does not continuously reset sessions.
class MaintainedArmSwitch
{
public:
	struct Result {
		bool request_arm{false};
		bool request_disarm{false};
		bool off{false};
		bool arm_allowed{false};
	};
	enum class PendingArm { None, Issue, Reject };
	void deferArm(uint64_t deadline) { _arm_deadline = deadline; }
	PendingArm pollArm(uint64_t now, bool transaction_pending, bool disarm_requested)
	{
		if (_arm_deadline == 0) { return PendingArm::None; }
		if (!transaction_pending || disarm_requested || now >= _arm_deadline) {
			_arm_deadline = 0;
			return PendingArm::Reject;
		}
		if (_arm_allowed) {
			_arm_deadline = 0;
			return PendingArm::Issue;
		}
		return PendingArm::None;
	}
	Result update(uint64_t sample, bool fresh, bool on, bool off)
	{
		if (!fresh || sample == 0 || on == off) {
			_off_seen = false;
			_arm_allowed = false;
			return {};
		}
		if (_known && sample <= _sample) {
			return {false, false, !_on, _arm_allowed};
		}
		_sample = sample;
		Result result{};
		if (off) {
			result.request_disarm = !_known || _on;
			_off_seen = true;
			_arm_allowed = false;
			result.off = true;
		} else {
			result.request_arm = _known && !_on && _off_seen;
			_arm_allowed = _arm_allowed || result.request_arm;
			_off_seen = false;
		}
		_on = on;
		_known = true;
		result.arm_allowed = _arm_allowed;
		return result;
	}
private:
	uint64_t _sample{0};
	uint64_t _arm_deadline{0};
	bool _known{false};
	bool _on{false};
	bool _off_seen{false};
	bool _arm_allowed{false};
};
} // namespace airlink
