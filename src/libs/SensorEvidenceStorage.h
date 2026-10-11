#pragma once

#include "SensorEvidenceState.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

namespace sensor_evidence {

// The same operations are used by firmware and injected-failure host tests.
struct FileIO {
    static FILE* open(const char* path, const char* mode) { return ::fopen(path, mode); }
    static size_t read(void* data, size_t size, FILE* file) { return ::fread(data, 1, size, file); }
    static size_t write(const void* data, size_t size, FILE* file) { return ::fwrite(data, 1, size, file); }
    static int seek(FILE* file, long offset, int origin) { return ::fseek(file, offset, origin); }
    static int flush(FILE* file) { return ::fflush(file); }
    static int close(FILE* file) { return ::fclose(file); }
    static int get(FILE* file) { return ::fgetc(file); }
    static int error(FILE* file) { return ::ferror(file); }
};

template<class IO = FileIO> bool advance_boot_counter(const char* path, uint32_t& counter)
{
    counter = 0;
    BootSlot slots[2] = {};
    errno = 0;
    FILE* file = IO::open(path, "r+b");
    bool fresh = false;
    if (!file) {
        if (errno != ENOENT) return false;
        file = IO::open(path, "w+b");
        fresh = true;
    }
    if (!file) return false;
    const bool read_ok = fresh || (IO::read(slots, sizeof(slots), file) == sizeof(slots) &&
        IO::get(file) == EOF && !IO::error(file));
    int newest = -1;
    if (read_ok) {
        for (int i = 0; i < 2; ++i)
            if (slots[i].valid() && (newest < 0 || slots[i].counter > slots[newest].counter)) newest = i;
    }
    // A lone legacy slot cannot prove whether its newer neighbour was lost.
    const bool ambiguous_legacy = newest >= 0 && slots[newest].magic == 0x53453101U &&
        (!slots[1 - newest].valid() || slots[1 - newest].counter > slots[newest].counter);
    if (!read_ok || (!fresh && newest < 0) || ambiguous_legacy ||
        (newest >= 0 && slots[newest].counter == UINT32_MAX)) {
        IO::close(file);
        return false;
    }
    const BootSlot next = BootSlot::make(newest < 0 ? 1 : slots[newest].counter + 1);
    bool ok = !fresh || IO::write(slots, sizeof(slots), file) == sizeof(slots);
    // Both copies hold the reserved high watermark before that identity is issued.
    // Preserve the newest old slot until the other new copy has been flushed.
    const unsigned first = newest < 0 ? 0 : 1 - newest;
    for (unsigned i = 0; ok && i < 2; ++i) {
        const unsigned target = i == 0 ? first : 1 - first;
        ok = IO::seek(file, target * sizeof(next), SEEK_SET) == 0 &&
            IO::write(&next, sizeof(next), file) == sizeof(next) && IO::flush(file) == 0;
    }
    const bool closed = IO::close(file) == 0;
    if (!ok || !closed) return false;
    file = IO::open(path, "rb");
    if (!file) return false;
    ok = IO::read(slots, sizeof(slots), file) == sizeof(slots) && IO::get(file) == EOF && !IO::error(file);
    const bool read_closed = IO::close(file) == 0;
    if (ok && read_closed && memcmp(&next, &slots[0], sizeof(next)) == 0 && memcmp(&next, &slots[1], sizeof(next)) == 0) {
        counter = next.counter;
        return true;
    }
    return false;
}

inline bool valid_fault_header(const FaultHeader& header) {
    return header.magic == 0x53454601U && header.boot_id != 0 && header.count > 0 && header.count <= 16 && header.reason != 0;
}

template<class IO = FileIO> bool read_fault(const char* path, FaultHeader& header)
{
    header = {};
    FILE* file = IO::open(path, "rb");
    if (!file) return false;
    bool ok = IO::read(&header, sizeof(header), file) == sizeof(header) && valid_fault_header(header);
    uint32_t crc = checksum(&header, offsetof(FaultHeader, crc));
    uint64_t previous_time = 0;
    Event event;
    for (unsigned i = 0; ok && i < header.count; ++i) {
        ok = IO::read(&event, sizeof(event), file) == sizeof(event) && valid_event(event) &&
            event.sequence == uint32_t(header.sequence - header.count + 1 + i);
        if (ok) {
            const uint64_t time = source_time(event.tick, event.wraps);
            ok = time >= previous_time && time <= source_time(header.tick, header.wraps);
            previous_time = time;
            if (i + 1 == header.count)
                ok = ok && event.kind == 3 && event.halt == header.reason && event.tick == header.tick && event.wraps == header.wraps;
            crc = checksum(&event, sizeof(event), crc);
        }
    }
    ok = ok && IO::get(file) == EOF && !IO::error(file) && crc == header.crc;
    const bool closed = IO::close(file) == 0;
    if (!ok || !closed) header = {};
    return ok && closed;
}

template<class IO = FileIO, class Events> bool write_fault(const char* path, FaultHeader& header, const Events& events)
{
    if (!valid_fault_header(header)) return false;
    header.crc = checksum(&header, offsetof(FaultHeader, crc));
    for (unsigned i = 0; i < header.count; ++i) {
        if (!valid_event(events(i))) return false;
        header.crc = checksum(&events(i), sizeof(Event), header.crc);
    }
    FILE* file = IO::open(path, "wb");
    if (!file) return false;
    const FaultHeader empty = {};
    bool ok = IO::write(&empty, sizeof(empty), file) == sizeof(empty);
    for (unsigned i = 0; ok && i < header.count; ++i)
        ok = IO::write(&events(i), sizeof(Event), file) == sizeof(Event);
    // Payload is flushed before the commit header. Flush/readback is not a
    // guarantee that an SD card has survived physical power loss.
    ok = ok && IO::flush(file) == 0 && IO::seek(file, 0, SEEK_SET) == 0 &&
        IO::write(&header, sizeof(header), file) == sizeof(header) && IO::flush(file) == 0;
    const bool closed = IO::close(file) == 0;
    FaultHeader verified;
    return ok && closed && read_fault<IO>(path, verified) && memcmp(&header, &verified, sizeof(header)) == 0;
}

} // namespace sensor_evidence
