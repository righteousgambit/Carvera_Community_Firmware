#include "SensorEvidence.h"
#include "SensorEvidenceStorage.h"
#include "Kernel.h"
#include "Pin.h"
#include "SlowTicker.h"
#include "StepTicker.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "Conveyor.h"
#include "Block.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "ATCHandlerPublicAccess.h"
#include "SpindlePublicAccess.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "FirmwareFileSystem.h"
#include "us_ticker_api.h"
#include <errno.h>
#include <string.h>

namespace {
struct InterruptLock {
    uint32_t mask;
    InterruptLock() : mask(__get_PRIMASK()) { __disable_irq(); }
    ~InterruptLock() { if (!mask) __enable_irq(); }
};
}

SensorEvidence::SensorEvidence() : pins{}, measurement_ticks{}, measurement_wraps{}, measurement_mask(0), input_mask(0), input_valid(0), last_inputs(0), last_valid(0),
    boot_id(0), status_sequence(0), fault_sequence(0), fault_tick(0), fault_wraps(0), stage_tick(0),
    stage_wraps(0), stage_transitions(0), last_periodic(0), last_spindle_tick(0), parsed_line(0),
    last_context_tick(0), context_wraps(0), spindle_wraps(0), sampled_rpm(0), sampled_pwm(0), tool_offset(0), active_tool(-1), target_tool(-1), previous_fault{},
    pending_hit{}, pending_hit_feed(0), hit_sequence(0), dropped_hits(0), hit_axes(0), hit_negative(0), hit_type(0), hit_pending(false),
    recovery_start(0), recovery_wraps(0), previous_slot(0), persistence_state(0), previous_valid(false), spindle_valid(false),
    tool_valid(false), recovering(false), parsed_valid(0), current_stage(0), fault_reason(0), boot_valid(false), started(false) {}

void SensorEvidence::register_input(unsigned bit, Pin* pin)
{
    if (bit < 16) pins[bit] = pin;
}

void SensorEvidence::persist_boot_counter()
{
    boot_valid = sensor_evidence::advance_boot_counter("/sd/sensor-boot.dat", boot_id);
}

void SensorEvidence::start(bool sd_available)
{
    if (sd_available) { persist_boot_counter(); load_previous_fault(); }
    clock.observe(us_ticker_read());
    last_periodic = last_spindle_tick = stage_tick = clock.tick;
    stage_wraps = clock.wraps;
    started = true;
    THEKERNEL->slow_ticker->attach(100, this, &SensorEvidence::tick);
    capture(1);
}

void SensorEvidence::read_inputs()
{
    input_mask = input_valid = 0;
    for (unsigned i = 0; i < 16; ++i) {
        if (pins[i] && pins[i]->connected()) {
            input_valid |= 1U << i;
            if (pins[i]->get()) input_mask |= 1U << i;
        }
    }
}

sensor_evidence::Event SensorEvidence::event(uint8_t kind)
{
    sensor_evidence::Event value = {};
    value.tick = clock.tick; value.wraps = clock.wraps;
    const uint64_t now = (uint64_t(clock.wraps) << 32) | clock.tick;
    const uint64_t context_time = (uint64_t(context_wraps) << 32) | last_context_tick;
    const uint64_t spindle_time = (uint64_t(spindle_wraps) << 32) | last_spindle_tick;
    value.context_age_us = !parsed_valid && !tool_valid ? UINT32_MAX :
        (now - context_time > UINT32_MAX ? UINT32_MAX : uint32_t(now - context_time));
    value.spindle_age_us = !spindle_valid ? UINT32_MAX :
        (now - spindle_time > UINT32_MAX ? UINT32_MAX : uint32_t(now - spindle_time));
    value.kind = kind; value.halt = THEKERNEL->is_halted() ? THEKERNEL->get_halt_reason() : 0;
    value.rpm = sampled_rpm; value.pwm = sampled_pwm;
    value.active_tool = active_tool; value.target_tool = target_tool; value.tool_offset = tool_offset;
    value.atc = current_stage; value.inputs = input_mask; value.input_valid = input_valid;
    value.parsed_line = parsed_line; value.context_valid = (parsed_valid ? 2 : 0) | (spindle_valid ? 4 : 0) | (tool_valid ? 8 : 0);
    const Block* block = StepTicker::getInstance()->get_current_block();
    if (block && block->is_ready && block->is_ticking) {
        value.context_valid |= 16;
        if (block->line > 0) {
            value.executing_line = block->line;
            value.context_valid |= 1;
        }
    }
    if (THEROBOT->actuators.size() >= 3) {
        value.x = int32_t(THEROBOT->actuators[0]->get_current_step());
        value.y = int32_t(THEROBOT->actuators[1]->get_current_step());
        value.z = int32_t(THEROBOT->actuators[2]->get_current_step());
        value.x_steps_per_mm = THEROBOT->actuators[0]->get_steps_per_mm();
        value.y_steps_per_mm = THEROBOT->actuators[1]->get_steps_per_mm();
        value.z_steps_per_mm = THEROBOT->actuators[2]->get_steps_per_mm();
    }
    return value;
}

void SensorEvidence::capture(uint8_t kind)
{
    if (!started) return;
    InterruptLock lock;
    clock.observe(us_ticker_read());
    read_inputs();
    history.push(event(kind));
    last_inputs = input_mask; last_valid = input_valid;
}

uint32_t SensorEvidence::tick(uint32_t)
{
    if (!started) return 0;
    InterruptLock lock;
    clock.observe(us_ticker_read());
    read_inputs();
    if (input_mask != last_inputs || input_valid != last_valid) {
        history.push(event(2));
        last_inputs = input_mask; last_valid = input_valid;
    }
    if (uint32_t(clock.tick - last_periodic) >= 100000) {
        history.push(event(1));
        last_periodic = clock.tick;
    }
    return 0;
}

void SensorEvidence::probe_inputs()
{
    if (!started) return;
    InterruptLock lock;
    uint32_t changed = 0;
    for (unsigned i = 11; i <= 12; ++i) {
        if (pins[i] && pins[i]->connected()) {
            const uint32_t bit = 1U << i;
            const bool active = pins[i]->get();
            if (active != bool(input_mask & bit)) changed |= bit;
            input_valid |= bit;
            if (active) input_mask |= bit; else input_mask &= ~bit;
        }
    }
    if (changed) {
        clock.observe(us_ticker_read());
        history.push(event(2));
        last_inputs = input_mask; last_valid = input_valid;
    }
}

void SensorEvidence::probe_hit(uint8_t type)
{
    if (!started) return;
    InterruptLock lock;
    clock.observe(us_ticker_read());
    ++hit_sequence;
    if (hit_pending) { ++dropped_hits; return; }
    read_inputs();
    pending_hit = event(5);
    pending_hit.sequence = hit_sequence;
    hit_type = type; hit_axes = hit_negative = 0;
    for (unsigned i = 0; i < 3 && i < THEROBOT->actuators.size(); ++i) {
        if (THEROBOT->actuators[i]->is_moving()) hit_axes |= 1U << i;
        if (THEROBOT->actuators[i]->which_direction()) hit_negative |= 1U << i;
    }
    pending_hit_feed = THECONVEYOR->get_current_feedrate();
    hit_pending = true;
    history.push(pending_hit);
}

void SensorEvidence::flush_probe_hit()
{
    if (!hit_pending) return;
    sensor_evidence::Event value;
    float feed;
    unsigned axes, negative, type;
    uint32_t dropped;
    {
        InterruptLock lock;
        if (!hit_pending) return;
        value = pending_hit; feed = pending_hit_feed;
        axes = hit_axes; negative = hit_negative; type = hit_type; dropped = dropped_hits;
        hit_pending = false;
    }
    THEKERNEL->streams->printf("{PRH:1,%lu,%u,%lu,%lu,%lu,%u,%.4f,%.4f,%.4f,%u,%u,%.3f,3,%lu}\n",
        boot_id, unsigned(boot_valid), value.sequence, value.tick, value.wraps, type,
        float(value.x) / value.x_steps_per_mm, float(value.y) / value.y_steps_per_mm,
        float(value.z) / value.z_steps_per_mm, negative, axes, feed * 60.0f, dropped);
}

void SensorEvidence::fault()
{
    if (!started) return;
    InterruptLock lock;
    clock.observe(us_ticker_read()); read_inputs();
    if (!history.frozen) {
        sensor_evidence::Event value = event(3);
        value.halt = THEKERNEL->get_halt_reason();
        history.push(value);
        fault_sequence = history.sequence; fault_tick = clock.tick; fault_wraps = clock.wraps;
        fault_reason = THEKERNEL->get_halt_reason();
        history.frozen = true;
    }
}

void SensorEvidence::stage(uint8_t state)
{
    if (!started) { current_stage = state; return; }
    InterruptLock lock;
    if (state == current_stage) return;
    clock.observe(us_ticker_read());
    current_stage = state; stage_tick = clock.tick; stage_wraps = clock.wraps; ++stage_transitions;
    read_inputs(); history.push(event(4));
}

void SensorEvidence::spindle_sample(float rpm, float pwm, float maximum, float target)
{
    if (!started) return;
    // Only SlowTicker writes these windows; higher-priority step interrupts stay enabled.
    clock.observe(us_ticker_read());
    sampled_rpm = rpm; sampled_pwm = pwm; spindle_valid = true;
    if (target <= 0) recovering = false;
    else if (rpm < target * 0.95f && !recovering) { recovering = true; recovery_start = clock.tick; recovery_wraps = clock.wraps; }
    else if (rpm >= target * 0.95f && recovering) {
        const uint64_t elapsed = ((uint64_t(clock.wraps) << 32) | clock.tick) -
            ((uint64_t(recovery_wraps) << 32) | recovery_start);
        spindle_work.recovery_us = elapsed > UINT32_MAX ? UINT32_MAX : uint32_t(elapsed);
        recovering = false;
    }
    uint32_t duration = clock.tick - last_spindle_tick;
    last_spindle_tick = clock.tick;
    spindle_wraps = clock.wraps;
    spindle_work.add(rpm, pwm, maximum, duration);
    spindle_work.end_tick = clock.tick; spindle_work.end_wraps = clock.wraps;
    if (spindle_work.samples >= 100) {
        spindle_completed = spindle_work;
        spindle_work = sensor_evidence::SpindleWindow();
    }
}

void SensorEvidence::measurement(unsigned channel, bool valid)
{
    if (!started || channel >= 3) return;
    InterruptLock lock;
    clock.observe(us_ticker_read());
    measurement_ticks[channel] = clock.tick;
    measurement_wraps[channel] = clock.wraps;
    if (valid) measurement_mask |= 1U << channel;
    else measurement_mask &= ~(1U << channel);
}

void SensorEvidence::update_context()
{
    if (!started) return;
    if (THEKERNEL->is_halted() && history.frozen && persistence_state == 0) persist_fault();
    const uint32_t now = us_ticker_read();
    if (uint32_t(now - last_context_tick) < 100000) return;
    void* returned = nullptr;
    bool valid = PublicData::get_value(player_checksum, get_progress_checksum, &returned);
    const uint32_t line = valid ? static_cast<pad_progress*>(returned)->parsed_lines : 0;
    struct tool_status tool = {};
    const bool have_tool = PublicData::get_value(atc_handler_checksum, get_tool_status_checksum, &tool);
    InterruptLock lock;
    clock.observe(us_ticker_read());
    last_context_tick = clock.tick;
    context_wraps = clock.wraps;
    parsed_valid = valid;
    parsed_line = line;
    tool_valid = have_tool;
    active_tool = have_tool ? tool.active_tool : -1;
    target_tool = have_tool ? tool.target_tool : -1;
    tool_offset = have_tool ? tool.tool_offset : 0;
}

void SensorEvidence::append_status(std::string& status, uint32_t validity, unsigned position_source)
{
    update_context();
    sensor_evidence::Event value;
    sensor_evidence::SpindleWindow spindle;
    uint32_t sequence, stage_age, transitions, fault_seq, fault_time, fault_wrap, count, dropped;
    uint8_t reason, stage_value;
    unsigned queued;
    uint32_t ages[3];
    unsigned measurement_validity;
    {
        InterruptLock lock;
        clock.observe(us_ticker_read()); read_inputs(); value = event(1);
        sequence = ++status_sequence;
        spindle = spindle_completed.samples ? spindle_completed : spindle_work;
        const uint64_t now = (uint64_t(clock.wraps) << 32) | clock.tick;
        const uint64_t beginning = (uint64_t(stage_wraps) << 32) | stage_tick;
        stage_age = now - beginning > UINT32_MAX ? UINT32_MAX : uint32_t(now - beginning);
        transitions = stage_transitions; stage_value = current_stage;
        fault_seq = fault_sequence; fault_time = fault_tick; fault_wrap = fault_wraps;
        reason = fault_reason; count = history.count; dropped = history.overwritten;
        queued = THECONVEYOR->queued_blocks();
        measurement_validity = measurement_mask;
        for (unsigned i = 0; i < 3; ++i) {
            const uint64_t acquired = (uint64_t(measurement_wraps[i]) << 32) | measurement_ticks[i];
            ages[i] = !(measurement_mask & (1U << i)) || now - acquired > UINT32_MAX ? UINT32_MAX : uint32_t(now - acquired);
        }
    }
    if (parsed_valid) validity |= 1U << 5;
    if (value.context_valid & 1) validity |= 1U << 6;
    char buf[256];
    snprintf(buf, sizeof(buf), "|SE:1,%lu,%u,%lu,%lu,%lu,%lu,%u,1|SI:%lu,%lu|SC:%lu,%lu,%u,%u",
        boot_id, unsigned(boot_valid), sequence, value.tick, value.wraps, validity, position_source,
        value.inputs, value.input_valid, value.executing_line, value.parsed_line,
        queued, unsigned(value.context_valid & 0x13));
    status.append(buf);
    snprintf(buf, sizeof(buf), "|SS:%lu,%.1f,%.1f,%.3f,%.3f,%lu,%lu,%lu,%lu,%.1f,%.1f,%.4f,%.6f,%lu",
        spindle.samples, spindle.rpm_min, spindle.rpm_max, spindle.pwm_min, spindle.pwm_max,
        spindle.saturation_us, spindle.window_us, spindle.end_tick, spindle.end_wraps,
        spindle.rpm_mean, spindle.samples ? spindle.rpm_m2 / spindle.samples : 0,
        spindle.pwm_mean, spindle.samples ? spindle.pwm_m2 / spindle.samples : 0, spindle.recovery_us);
    status.append(buf);
    snprintf(buf, sizeof(buf), "|SA:%u,%lu,%lu,-1|SF:%lu,%lu,%lu,%u,%lu,%lu,%u|SP:%lu,%lu,%lu,%u",
        unsigned(stage_value), stage_age, transitions, fault_seq, fault_time, fault_wrap, unsigned(reason), count, dropped,
        unsigned(persistence_state), previous_fault.boot_id, previous_fault.sequence, previous_fault.count, unsigned(previous_valid));
    status.append(buf);
    snprintf(buf, sizeof(buf), "|ST:1,%u,%lu,%lu,%lu", measurement_validity, ages[0], ages[1], ages[2]);
    status.append(buf);
}

void SensorEvidence::dump(StreamOutput* stream, bool rearm, bool previous)
{
    if (!THEKERNEL->is_halted() && !THECONVEYOR->is_idle()) { stream->printf("{EE:1}\n"); return; }
    unsigned count, oldest_index;
    uint32_t last_sequence, dump_boot = boot_id;
    bool dump_boot_valid = boot_valid;
    {
        InterruptLock lock;
        count = history.count; last_sequence = history.sequence;
        oldest_index = (history.head + 16 - history.count) % 16;
    }
    FILE* file = nullptr;
    sensor_evidence::FaultHeader header;
    uint32_t crc = 0;
    if (previous) {
        if (!previous_valid || !read_fault(previous_slot, header) || header.boot_id != previous_fault.boot_id || header.sequence != previous_fault.sequence) {
            stream->printf("{EE:1}\n"); return;
        }
        char path[32]; snprintf(path, sizeof(path), "/sd/sensor-fault%u.dat", unsigned(previous_slot));
        file = fwfs::fopen(path, "rb");
        if (!file) { stream->printf("{EE:1}\n"); return; }
        if (fwfs::fseek(file, sizeof(header), SEEK_SET) != 0) {
            fwfs::fclose(file); stream->printf("{EE:1}\n"); return;
        }
        count = header.count; last_sequence = header.sequence; dump_boot = header.boot_id; dump_boot_valid = true;
        crc = sensor_evidence::checksum(&header, offsetof(sensor_evidence::FaultHeader, crc));
    }
    stream->printf("{EH:1,%lu,%u,%u,%lu}\n", dump_boot, unsigned(dump_boot_valid), count, last_sequence);
    bool complete = true;
    for (unsigned i = 0; i < count; ++i) {
        sensor_evidence::Event value;
        if (file) {
            if (fwfs::fread(&value, sizeof(value), 1, file) != 1) { complete = false; break; }
            crc = sensor_evidence::checksum(&value, sizeof(value), crc);
        } else {
            InterruptLock lock;
            value = history.events[(oldest_index + i) % 16];
            if (value.sequence != uint32_t(last_sequence - count + 1 + i)) { complete = false; break; }
        }
        stream->printf("{EV:%lu,%lu,%lu,%u,%lu,%lu,%u,%u,%lu,%lu,%.4f,%.4f,%.4f,%.1f,%.3f,%u,%ld,%ld,%.4f,%lu,%lu}\n",
            value.sequence, value.tick, value.wraps, unsigned(value.kind), value.inputs, value.input_valid,
            unsigned(value.halt), unsigned(value.atc), value.executing_line, value.parsed_line,
            float(value.x) / value.x_steps_per_mm, float(value.y) / value.y_steps_per_mm,
            float(value.z) / value.z_steps_per_mm, value.rpm, value.pwm, unsigned(value.context_valid), value.active_tool, value.target_tool, value.tool_offset, value.context_age_us, value.spindle_age_us);
    }
    if (file) complete = fwfs::fclose(file) == 0 && crc == header.crc && complete;
    stream->printf("{EE:%u}\n", complete ? 0U : 1U);
    if (complete && rearm && !previous && !THEKERNEL->is_halted() && THECONVEYOR->is_idle()) {
        InterruptLock lock;
        if (history.sequence == last_sequence) {
            history.rearm(); fault_sequence = fault_tick = fault_wraps = 0; fault_reason = 0; persistence_state = 0;
        }
    }
}


bool SensorEvidence::read_fault(unsigned slot, sensor_evidence::FaultHeader& header)
{
    header = {};
    char path[32]; snprintf(path, sizeof(path), "/sd/sensor-fault%u.dat", slot);
    FILE* file = fwfs::fopen(path, "rb");
    if (!file) return false;
    bool ok = fwfs::fread(&header, sizeof(header), 1, file) == 1 && header.magic == 0x53454601U &&
        header.boot_id != 0 && header.count > 0 && header.count <= 16;
    uint32_t crc = sensor_evidence::checksum(&header, offsetof(sensor_evidence::FaultHeader, crc));
    sensor_evidence::Event value;
    for (unsigned i = 0; ok && i < header.count; ++i) {
        ok = fwfs::fread(&value, sizeof(value), 1, file) == 1;
        if (ok) crc = sensor_evidence::checksum(&value, sizeof(value), crc);
    }
    const bool closed = fwfs::fclose(file) == 0;
    return ok && closed && crc == header.crc;
}

void SensorEvidence::load_previous_fault()
{
    for (unsigned slot = 0; slot < 2; ++slot) {
        sensor_evidence::FaultHeader candidate;
        if (read_fault(slot, candidate) && (!previous_valid || candidate.boot_id > previous_fault.boot_id ||
            (candidate.boot_id == previous_fault.boot_id && candidate.sequence > previous_fault.sequence))) {
            previous_fault = candidate; previous_slot = slot; previous_valid = true;
        }
    }
}

void SensorEvidence::persist_fault()
{
    persistence_state = 2;
    if (!boot_valid) return;
    sensor_evidence::FaultHeader header;
    {
        InterruptLock lock;
        header = {0x53454601U, boot_id, fault_sequence, history.count, fault_tick, fault_wraps, fault_reason, 0};
    }
    header.crc = sensor_evidence::checksum(&header, offsetof(sensor_evidence::FaultHeader, crc));
    for (unsigned i = 0; i < header.count; ++i)
        header.crc = sensor_evidence::checksum(&history.oldest(i), sizeof(sensor_evidence::Event), header.crc);
    // Keep the last validated slot until the new slot is closed and read back.
    const unsigned slot = previous_valid ? 1 - previous_slot : 0;
    char path[32]; snprintf(path, sizeof(path), "/sd/sensor-fault%u.dat", slot);
    FILE* file = fwfs::fopen(path, "wb");
    if (!file) return;
    const sensor_evidence::FaultHeader empty = {};
    bool ok = fwfs::fwrite(&empty, sizeof(empty), 1, file) == 1;
    for (unsigned i = 0; ok && i < header.count; ++i)
        ok = fwfs::fwrite(&history.oldest(i), sizeof(sensor_evidence::Event), 1, file) == 1;
    ok = ok && fwfs::fseek(file, 0, SEEK_SET) == 0 && fwfs::fwrite(&header, sizeof(header), 1, file) == 1;
    const bool flushed = ::fflush(file) == 0;
    const bool closed = fwfs::fclose(file) == 0;
    sensor_evidence::FaultHeader verified;
    if (ok && flushed && closed && read_fault(slot, verified) && memcmp(&header, &verified, sizeof(header)) == 0) {
        persistence_state = 1;
        previous_fault = verified; previous_slot = slot; previous_valid = true;
    }
}
