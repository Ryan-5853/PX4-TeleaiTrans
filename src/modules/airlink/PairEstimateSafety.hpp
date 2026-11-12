#pragma once
#include <cmath>
#include <cstdint>

namespace pair_estimate_safety {
constexpr uint64_t kFreshUs = 10000;
enum class Fault : uint8_t { None, Invalid, Stale, Source, Numerical, Reset };
inline bool fresh(uint64_t now, uint64_t stamp) { return stamp && now >= stamp && now - stamp <= kFreshUs; }
// Shared by AIRLINK, comp_control and the actuator-output owner. No allocation.
template<typename Estimate> Fault inspect(const Estimate &e, uint64_t now)
{
    if (!e.valid) return Fault::Invalid;
    if (!fresh(now, e.timestamp) || !fresh(now, e.timestamp_sample)) return Fault::Stale;
    constexpr uint16_t required = Estimate::HEALTH_SOURCE_A_FRESH | Estimate::HEALTH_SOURCE_B_FRESH | Estimate::HEALTH_EKF_VALID;
    if ((e.health_flags & required) != required || (e.health_flags & Estimate::HEALTH_DEGRADED)
        || e.source_age_a_us > kFreshUs || e.source_age_b_us > kFreshUs) return Fault::Source;
    float norm = 0.f;
    for (float value : e.q) { if (!std::isfinite(value)) return Fault::Numerical; norm += value * value; }
    if (!std::isfinite(norm) || fabsf(norm - 1.f) > .02f) return Fault::Numerical;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(e.angular_velocity[i]) || !std::isfinite(e.angular_acceleration[i])
            || !std::isfinite(e.specific_force[i])) return Fault::Numerical;
    }
    return Fault::None;
}
template<typename Estimate> bool healthy(const Estimate &e, uint64_t now) { return inspect(e, now) == Fault::None; }
class Guard {
public:
    template<typename Estimate> bool update(bool protected_pair, const Estimate &e, uint64_t now) {
        if (_fault != Fault::None) return true;
        const Fault failure = inspect(e, now);
        if (!protected_pair) {
            _seen = failure == Fault::None; _counter = e.reset_counter; return false;
        }
        if (failure != Fault::None) trip(failure);
        else if (!_seen || _counter != e.reset_counter) trip(Fault::Reset);
        return faulted();
    }
    void trip(Fault reason) { if (_fault == Fault::None) _fault = reason; }
    bool faulted() const { return _fault != Fault::None; }
    Fault fault() const { return _fault; }
    // Caller must require an explicit operator DISARM acknowledgement while disarmed.
    void acknowledge() { _fault = Fault::None; _seen = false; }
private:
    Fault _fault{Fault::None};
    bool _seen{false};
    uint8_t _counter{};
};
} // namespace pair_estimate_safety
