#pragma once

#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>

namespace sensor_evidence {

struct Clock {
    uint32_t tick = 0;
    uint32_t wraps = 0;
    bool started = false;
    void observe(uint32_t now) {
        if (started && now < tick) ++wraps;
        tick = now;
        started = true;
    }
};

struct BootSlot {
    uint32_t magic, counter, inverse, check;
    static BootSlot make(uint32_t counter) {
        return {0x53453102U, counter, ~counter, 0x53453102U ^ counter ^ 0x91e10da5U};
    }
    bool valid() const {
        return (magic == 0x53453101U || magic == 0x53453102U) && counter != 0 && inverse == ~counter &&
            check == (magic ^ counter ^ 0x91e10da5U);
    }
};

struct Event {
    uint32_t sequence, tick, wraps, inputs, input_valid, executing_line, parsed_line;
    uint32_t context_age_us, spindle_age_us;
    int32_t x, y, z;
    float x_steps_per_mm, y_steps_per_mm, z_steps_per_mm, rpm, pwm;
    int32_t active_tool, target_tool;
    float tool_offset;
    uint8_t kind, halt, atc, context_valid;
};
static_assert(sizeof(Event) == 84, "Bounded sensor event layout");

// The caller serializes access between interrupt and main-loop writers.
template<size_t Capacity> struct History {
    Event events[Capacity];
    uint32_t sequence = 0, overwritten = 0;
    uint16_t head = 0, count = 0;
    bool frozen = false;
    bool push(const Event& event) {
        if (frozen) return false;
        events[head] = event;
        events[head].sequence = ++sequence;
        head = (head + 1) % Capacity;
        if (count < Capacity) ++count;
        else if (overwritten != UINT32_MAX) ++overwritten;
        return true;
    }
    const Event& oldest(size_t index) const {
        return events[(head + Capacity - count + index) % Capacity];
    }
    void rearm() { head = count = 0; overwritten = 0; frozen = false; }
};

struct SpindleWindow {
    uint32_t samples = 0, saturation_us = 0, window_us = 0, end_tick = 0, end_wraps = 0;
    float rpm_min = 0, rpm_max = 0, pwm_min = 0, pwm_max = 0;
    float rpm_mean = 0, rpm_m2 = 0, pwm_mean = 0, pwm_m2 = 0;
    uint32_t recovery_us = 0;
    bool add(float rpm, float pwm, float maximum, uint32_t duration) {
        if (!isfinite(rpm) || !isfinite(pwm) || !isfinite(maximum) || rpm < 0 || pwm < 0 ||
            samples == UINT32_MAX || duration > UINT32_MAX - window_us) return false;
        if (samples == 0) { rpm_min = rpm_max = rpm; pwm_min = pwm_max = pwm; }
        else {
            if (rpm < rpm_min) rpm_min = rpm;
            if (rpm > rpm_max) rpm_max = rpm;
            if (pwm < pwm_min) pwm_min = pwm;
            if (pwm > pwm_max) pwm_max = pwm;
        }
        ++samples;
        const float rpm_delta = rpm - rpm_mean;
        rpm_mean += rpm_delta / samples;
        rpm_m2 += rpm_delta * (rpm - rpm_mean);
        const float pwm_delta = pwm - pwm_mean;
        pwm_mean += pwm_delta / samples;
        pwm_m2 += pwm_delta * (pwm - pwm_mean);
        window_us += duration;
        if (pwm >= maximum && maximum > 0) saturation_us += duration;
        return isfinite(rpm_m2) && isfinite(pwm_m2);
    }
};

inline uint32_t checksum(const void* data, size_t length, uint32_t crc = 0xffffffffU) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    while (length--) {
        crc ^= *bytes++;
        for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
    }
    return crc;
}

struct FaultHeader {
    uint32_t magic, boot_id, sequence, count, tick, wraps, reason, crc;
};

inline uint64_t source_time(uint32_t tick, uint32_t wraps) {
    return (uint64_t(wraps) << 32) | tick;
}

inline bool event_position(const Event& event, float& x, float& y, float& z) {
    if (!isfinite(event.x_steps_per_mm) || !isfinite(event.y_steps_per_mm) || !isfinite(event.z_steps_per_mm) ||
        event.x_steps_per_mm <= 0 || event.y_steps_per_mm <= 0 || event.z_steps_per_mm <= 0) return false;
    x = float(event.x) / event.x_steps_per_mm;
    y = float(event.y) / event.y_steps_per_mm;
    z = float(event.z) / event.z_steps_per_mm;
    return isfinite(x) && isfinite(y) && isfinite(z);
}

inline bool valid_event(const Event& event) {
    float x, y, z;
    return event.kind >= 1 && event.kind <= 5 && !(event.input_valid & ~0xffffU) &&
        !(event.inputs & ~event.input_valid) && !(event.context_valid & ~0x1fU) &&
        (!(event.context_valid & 1) || ((event.context_valid & 16) && event.executing_line > 0)) &&
        isfinite(event.rpm) && event.rpm >= 0 && isfinite(event.pwm) && event.pwm >= 0 && isfinite(event.tool_offset) && event_position(event, x, y, z);
}

struct Contact {
    Event event;
    float feed;
    uint8_t axes, negative, type;
};

// Access is serialized by the caller. Loss remains observable even without a later hit.
template<size_t Capacity> struct ContactQueue {
    Contact contacts[Capacity];
    uint32_t sequence = 0, dropped = 0;
    uint8_t head = 0, count = 0;
    bool push(const Contact& contact) {
        ++sequence;
        if (count == Capacity) { if (dropped != UINT32_MAX) ++dropped; return false; }
        contacts[(head + count) % Capacity] = contact;
        contacts[(head + count) % Capacity].event.sequence = sequence;
        ++count;
        return true;
    }
    bool pop(Contact& contact) {
        if (!count) return false;
        contact = contacts[head];
        head = (head + 1) % Capacity;
        --count;
        return true;
    }
};

struct StallDetector {
    uint32_t armed_at = 0, low_at = 0;
    float previous_target = 0;
    bool armed = false, low = false;
    bool update(uint32_t now, bool on, float target, float rpm, uint32_t spinup_us,
                uint32_t low_us, float minimum_target, float minimum_rpm) {
        if (!isfinite(target) || !isfinite(rpm) || !isfinite(minimum_target) || !isfinite(minimum_rpm) ||
            rpm < 0 || minimum_target < 0 || minimum_rpm < 0 || spinup_us > INT32_MAX || low_us > INT32_MAX ||
            !on || target <= minimum_target) { armed = low = false; previous_target = target; return false; }
        if (!armed || target != previous_target) {
            armed = true; low = false; armed_at = now; previous_target = target;
        }
        if (uint32_t(now - armed_at) < spinup_us) { low = false; return false; }
        if (rpm >= minimum_rpm) { low = false; return false; }
        if (!low) { low = true; low_at = now; }
        return uint32_t(now - low_at) >= low_us;
    }
};

} // namespace sensor_evidence
