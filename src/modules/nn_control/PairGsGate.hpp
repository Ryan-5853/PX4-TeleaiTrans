#pragma once

// Master-only, explicit GS request edge. After a timeout, a fresh request-off
// must be observed before a new request-on can take control. Monitoring and
// heartbeats alone never constitute a control request.
class PairGsGate
{
public:
	bool update(bool eligible, bool fresh, bool requested)
	{
		if (!fresh) {
			_active = false;
			return false;
		}
		const bool rising = requested && !_requested;
		_requested = requested;
		if (!eligible || !requested) {
			_active = false;
		} else if (rising) {
			_active = true;
		}
		return _active;
	}
private:
	bool _requested{false};
	bool _active{false};
};
