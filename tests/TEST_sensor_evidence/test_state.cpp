#include "SensorEvidenceState.h"
#include "SensorEvidenceStorage.h"
#include <unistd.h>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace sensor_evidence;

int main()
{
    Clock clock;
    clock.observe(0xfffffff0U); clock.observe(0x20U);
    assert(clock.wraps == 1 && clock.tick == 0x20U);
    clock.observe(0x20U); assert(clock.wraps == 1);

    auto boot = BootSlot::make(7);
    assert(boot.valid());
    boot.inverse ^= 1; assert(!boot.valid());
    assert(!BootSlot::make(0).valid());
    assert(BootSlot::make(UINT32_MAX).valid());

    char counter_path[] = "/tmp/carvera-boot-counter-XXXXXX";
    int fd = mkstemp(counter_path); assert(fd >= 0); close(fd); unlink(counter_path);
    uint32_t counter = 0;
    assert(advance_boot_counter(counter_path, counter) && counter == 1);
    assert(advance_boot_counter(counter_path, counter) && counter == 2);
    // A torn update to the alternate slot leaves the other complete slot recoverable.
    FILE* store = fopen(counter_path, "r+b"); assert(store);
    uint32_t torn = 0;
    assert(fwrite(&torn, sizeof(torn), 1, store) == 1); assert(fclose(store) == 0);
    assert(advance_boot_counter(counter_path, counter) && counter == 3);
    store = fopen(counter_path, "r+b"); assert(store);
    BootSlot corrupt[2] = {}; assert(fwrite(corrupt, sizeof(corrupt), 1, store) == 1); assert(fclose(store) == 0);
    assert(!advance_boot_counter(counter_path, counter));
    store = fopen(counter_path, "wb"); assert(store);
    BootSlot exhausted[2] = {BootSlot::make(UINT32_MAX), BootSlot::make(UINT32_MAX - 1)};
    assert(fwrite(exhausted, sizeof(exhausted), 1, store) == 1); assert(fclose(store) == 0);
    assert(!advance_boot_counter(counter_path, counter));
    unlink(counter_path);
    assert(!advance_boot_counter("/a/nonexistent/carvera-test-dir/counter", counter));

    History<3> history;
    Event value = {};
    for (unsigned i = 1; i <= 5; ++i) { value.tick = i; assert(history.push(value)); }
    assert(history.count == 3 && history.overwritten == 2);
    assert(history.oldest(0).sequence == 3 && history.oldest(2).sequence == 5);
    // A streaming dump keeps physical-slot/sequence identity as new records arrive.
    const unsigned oldest = (history.head + 3 - history.count) % 3;
    const auto first_sequence = history.oldest(0).sequence;
    const auto copied_first = history.events[oldest];
    history.push(value);
    assert(copied_first.sequence == first_sequence);
    assert(history.events[(oldest + 1) % 3].sequence == first_sequence + 1);
    history.frozen = true;
    assert(!history.push(value) && history.sequence == 6);
    history.rearm(); assert(history.count == 0 && !history.frozen);
    history.push(value); assert(history.oldest(0).sequence == 7);

    history.sequence = UINT32_MAX;
    history.push(value); assert(history.sequence == 0 && history.oldest(history.count - 1).sequence == 0);
    history.overwritten = UINT32_MAX;
    history.push(value); history.push(value); history.push(value);
    assert(history.overwritten == UINT32_MAX);
    ContactQueue<2> queue;
    Contact contact = {};
    queue.sequence = UINT32_MAX - 1;
    assert(queue.push(contact) && queue.push(contact));
    assert(!queue.push(contact) && queue.dropped == 1 && queue.sequence == 1);
    assert(queue.pop(contact) && contact.event.sequence == UINT32_MAX);
    assert(queue.pop(contact) && contact.event.sequence == 0);
    assert(!queue.pop(contact));
    queue.dropped = UINT32_MAX; queue.push(contact); queue.push(contact); queue.push(contact);
    assert(queue.dropped == UINT32_MAX);
    value.x_steps_per_mm = value.y_steps_per_mm = value.z_steps_per_mm = 400;
    float x, y, z;
    assert(event_position(value, x, y, z));
    value.x_steps_per_mm = 0; assert(!event_position(value, x, y, z));
    value.x_steps_per_mm = NAN; assert(!event_position(value, x, y, z));

    const char fixture[] = "123456789";
    assert((checksum(fixture, 9) ^ 0xffffffffU) == 0xcbf43926U);
    uint32_t partial = checksum(fixture, 4);
    assert(checksum(fixture + 4, 5, partial) == checksum(fixture, 9));
    FaultHeader header = {0x53454601U, 1, 2, 1, 3, 0, 24, 0};
    auto crc = checksum(&header, offsetof(FaultHeader, crc));
    const auto good = checksum(&value, sizeof(value), crc);
    value.inputs ^= 1; assert(checksum(&value, sizeof(value), crc) != good);

    SpindleWindow stats;
    stats.add(9000, .8f, 1, 10000); stats.add(11000, 1, 1, 10000);
    assert(stats.samples == 2 && stats.saturation_us == 10000 && stats.window_us == 20000);
    assert(stats.rpm_min == 9000 && stats.rpm_max == 11000 && stats.rpm_mean == 10000);
    assert(stats.rpm_m2 / stats.samples == 1000000);
    assert(std::fabs(stats.pwm_m2 / stats.samples - .01f) < .000001f);

    const auto samples = stats.samples;
    assert(!stats.add(NAN, 1, 1, 100)); assert(stats.samples == samples);
    assert(!stats.add(1000, INFINITY, 1, 100));
    assert(!stats.add(1000, 1, 1, UINT32_MAX));

    StallDetector stall;
    assert(!stall.update(0, true, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(99, true, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(100, true, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(149, true, 12000, 0, 100, 50, 8000, 5000));
    assert(stall.update(150, true, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(151, false, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(152, true, 4000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(200, true, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(300, true, 12000, 9000, 100, 50, 8000, 5000));
    assert(!stall.update(301, true, 12000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(320, true, 11000, 0, 100, 50, 8000, 5000));
    assert(!stall.update(420, true, 11000, 0, 100, 50, 8000, 5000));
    assert(stall.update(470, true, 11000, 0, 100, 50, 8000, 5000));
    StallDetector wrapped;
    assert(!wrapped.update(0xfffffff0U, true, 12000, 0, 20, 20, 8000, 5000));
    assert(!wrapped.update(4, true, 12000, 0, 20, 20, 8000, 5000));
    assert(wrapped.update(24, true, 12000, 0, 20, 20, 8000, 5000));
    assert(!wrapped.update(100, true, NAN, 0, 0, 0, 8000, 5000));
    assert(!wrapped.update(100, true, 12000, NAN, 0, 0, 8000, 5000));
    assert(!wrapped.update(100, true, 12000, 0, UINT32_MAX, 0, 8000, 5000));
    std::cout << "sensor evidence state: all checks passed\n";
}
