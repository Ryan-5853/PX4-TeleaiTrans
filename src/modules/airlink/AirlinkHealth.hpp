#pragma once
#include <cstdint>
#include <climits>

namespace airlink {
// Fixed-size, allocation-free counters. Peaks/counters live for the module lifetime.
inline uint32_t health_elapsed(uint64_t now, uint64_t then)
{
    return then && now >= then ? static_cast<uint32_t>(now - then > UINT32_MAX ? UINT32_MAX : now - then) : UINT32_MAX;
}
inline void health_peak(uint32_t &peak, uint32_t value) { if (value != UINT32_MAX && value > peak) peak = value; }
struct HealthGap {
    uint64_t last{};
    uint32_t current{}, maximum{}, over_limit{};
    void sample(uint64_t now) {
        if (last && now > last) {
            current = health_elapsed(now, last); health_peak(maximum, current);
            if (current > 10000) ++over_limit;
        }
        last = now;
    }
};
struct RuntimeHealth {
    HealthGap run, rx, tx;
    uint32_t work_us{}, work_max_us{}, rx_age_max_us{}, stream_timeouts{};
    uint32_t peer_imu_invalid_packets{}, peer_imu_invalid_events{};
    uint32_t local_imu_age_max_us{}, local_core_age_max_us{}, local_core_stale_events{};
    bool local_core_stale{};
    uint64_t expected_since{};
    bool stream_stale{}, peer_imu_invalid{};
    void peerImu(bool valid) {
        if (!valid) { ++peer_imu_invalid_packets; if (!peer_imu_invalid) ++peer_imu_invalid_events; }
        peer_imu_invalid = !valid;
    }
    void observe(uint64_t now, bool expected) {
        if (!expected) { expected_since = 0; stream_stale = false; return; }
        if (!expected_since) expected_since = now;
        const uint64_t basis = rx.last > expected_since ? rx.last : expected_since;
        const uint32_t age = health_elapsed(now, basis);
        health_peak(rx_age_max_us, age);
        const bool stale = age != UINT32_MAX && age > 10000;
        if (stale && !stream_stale) ++stream_timeouts;
        stream_stale = stale;
    }
    void sessionChanged() { rx.last = tx.last = expected_since = 0; stream_stale = peer_imu_invalid = false; }
    void work(uint64_t start, uint64_t end) { work_us = health_elapsed(end, start); health_peak(work_max_us, work_us); }
};
struct TimingHealth {
    uint64_t pending{}, last_probe{}, last_reply{};
    uint32_t peer_session{}, rtt_us{}, rtt_max_us{}, samples{}, timeouts{}, over_limit{};
    void session(uint32_t id) { if (id != peer_session) { pending = last_probe = last_reply = 0; peer_session = id; } }
    void expire(uint64_t now) { if (pending && health_elapsed(now, pending) > 100000) { ++timeouts; pending = 0; } }
    bool due(uint64_t now) const { return !pending && (!last_probe || health_elapsed(now, last_probe) >= 200000); }
    void sent(uint64_t now) { pending = last_probe = now; }
    void reply(uint64_t token, uint64_t now) {
        if (!pending || token != pending || now < pending) return;
        expire(now); if (!pending) return;
        rtt_us = health_elapsed(now, pending); health_peak(rtt_max_us, rtt_us);
        if (rtt_us > 10000) ++over_limit;
        ++samples; last_reply = now; pending = 0;
    }
};
} // namespace airlink
