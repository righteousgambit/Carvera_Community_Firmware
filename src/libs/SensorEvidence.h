#pragma once

#include "SensorEvidenceState.h"
#include <string>

class Pin;
class StreamOutput;

class SensorEvidence {
public:
    SensorEvidence();
    void register_input(unsigned bit, Pin* pin);
    void start(bool sd_available);
    uint32_t tick(uint32_t);
    void capture(uint8_t kind);
    void fault();
    void stage(uint8_t state);
    void probe_inputs();
    void probe_hit(uint8_t type);
    void flush_probe_hit();
    void update_context();
    void measurement(unsigned channel, bool valid);
    bool measurement_valid(unsigned channel) const { return channel < 3 && (measurement_mask & (1U << channel)); }
    void append_status(std::string& status, uint32_t validity, unsigned position_source);
    void dump(StreamOutput* stream, bool rearm, bool previous);
    void spindle_sample(float rpm, float pwm, float maximum, float target);

private:
    void persist_boot_counter();
    void persist_fault();
    void load_previous_fault();
    bool read_fault(unsigned slot, sensor_evidence::FaultHeader& header);
    void read_inputs();
    sensor_evidence::Event event(uint8_t kind);
    sensor_evidence::History<16> history;
    sensor_evidence::Clock clock;
    sensor_evidence::SpindleWindow spindle_work, spindle_completed;
    Pin* pins[16];
    uint32_t measurement_ticks[3], measurement_wraps[3];
    uint8_t measurement_mask;
    uint32_t input_mask, input_valid, last_inputs, last_valid;
    uint32_t boot_id, status_sequence, fault_sequence, fault_tick, fault_wraps;
    uint32_t stage_tick, stage_wraps, stage_transitions, last_periodic, last_spindle_tick;
    uint32_t parsed_line, last_context_tick, context_wraps, spindle_wraps;
    float sampled_rpm, sampled_pwm, tool_offset;
    int32_t active_tool, target_tool;
    sensor_evidence::FaultHeader previous_fault;
    sensor_evidence::ContactQueue<2> contacts;
    uint32_t persistence_elapsed_us;
    uint8_t rearm_result;
    uint32_t recovery_start, recovery_wraps;
    uint8_t previous_slot, persistence_state;
    bool previous_valid, spindle_valid, tool_valid, recovering;
    uint8_t parsed_valid, current_stage, fault_reason;
    bool boot_valid, started;
};
static_assert(sizeof(SensorEvidence) <= 1984, "Sensor recorder AHB budget");
