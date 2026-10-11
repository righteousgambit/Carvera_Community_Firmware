#pragma once

#include "SensorEvidenceState.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

namespace sensor_evidence {

inline bool advance_boot_counter(const char* path, uint32_t& counter)
{
    sensor_evidence::BootSlot slots[2] = {};
    errno = 0;
    FILE* file = ::fopen(path, "r+b");
    bool fresh = false;
    if (!file) {
        if (errno != ENOENT) return false;
        file = ::fopen(path, "w+b");
        fresh = true;
    }
    if (!file) return false;
    bool read_ok = fresh || ::fread(slots, sizeof(slots), 1, file) == 1;
    int newest = -1;
    if (read_ok) {
        for (int i = 0; i < 2; ++i)
            if (slots[i].valid() && (newest < 0 || slots[i].counter > slots[newest].counter)) newest = i;
    }
    if (!read_ok || (!fresh && newest < 0) || (newest >= 0 && slots[newest].counter == UINT32_MAX)) {
        ::fclose(file);
        return false;
    }
    const unsigned target = newest < 0 ? 0 : 1 - newest;
    const sensor_evidence::BootSlot next = sensor_evidence::BootSlot::make(newest < 0 ? 1 : slots[newest].counter + 1);
    bool ok = true;
    if (fresh) ok = ::fwrite(slots, sizeof(slots), 1, file) == 1;
    ok = ok && ::fseek(file, target * sizeof(next), SEEK_SET) == 0 && ::fwrite(&next, sizeof(next), 1, file) == 1;
    const bool flushed = ::fflush(file) == 0;
    const bool closed = ::fclose(file) == 0;
    if (!ok || !flushed || !closed) return false;
    file = ::fopen(path, "rb");
    if (!file) return false;
    sensor_evidence::BootSlot verified = {};
    ok = ::fseek(file, target * sizeof(next), SEEK_SET) == 0 && ::fread(&verified, sizeof(verified), 1, file) == 1;
    const bool read_closed = ::fclose(file) == 0;
    if (ok && read_closed && verified.valid() && memcmp(&next, &verified, sizeof(next)) == 0) {
        counter = next.counter;
        return true;
    }
    return false;
}

} // namespace sensor_evidence
