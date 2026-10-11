#include "SensorEvidenceStorage.h"
#include <unistd.h>
#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>
using namespace sensor_evidence;

struct FailingIO : FileIO {
    static unsigned calls, fail_at;
    static bool failed;
    static bool fail() { ++calls; if (calls != fail_at) return false; failed = true; errno = EIO; return true; }
    static void reset(unsigned target) { calls = 0; fail_at = target; failed = false; }
    static FILE* open(const char* p, const char* m) { return fail() ? nullptr : FileIO::open(p, m); }
    static size_t read(void* p, size_t n, FILE* f) { return FileIO::read(p, fail() ? n / 2 : n, f); }
    static size_t write(const void* p, size_t n, FILE* f) { return FileIO::write(p, fail() ? n / 2 : n, f); }
    static int seek(FILE* f, long n, int w) { return fail() ? -1 : FileIO::seek(f, n, w); }
    static int flush(FILE* f) { const bool bad = fail(); const int result = FileIO::flush(f); return bad ? -1 : result; }
    static int close(FILE* f) { const bool bad = fail(); const int result = FileIO::close(f); return bad ? -1 : result; }
    static int get(FILE* f) { return fail() ? EOF : FileIO::get(f); }
    static int error(FILE* f) { return fail() || failed ? 1 : FileIO::error(f); }
};
unsigned FailingIO::calls = 0, FailingIO::fail_at = 0;
bool FailingIO::failed = false;

static void put(const char* path, const void* data, size_t bytes) {
    FILE* file = fopen(path, "wb"); assert(file);
    assert(fwrite(data, 1, bytes, file) == bytes); assert(fclose(file) == 0);
}
static std::vector<unsigned char> get(const char* path) {
    FILE* file = fopen(path, "rb"); assert(file);
    std::vector<unsigned char> bytes;
    for (int ch; (ch = fgetc(file)) != EOF;) bytes.push_back(ch);
    assert(!ferror(file)); assert(fclose(file) == 0);
    return bytes;
}
struct OrderedIO : FileIO {
    static std::string order;
    static size_t write(const void* data, size_t bytes, FILE* file) {
        uint32_t magic = 0; memcpy(&magic, data, sizeof(magic));
        order += bytes == sizeof(FaultHeader) ? (magic ? 'C' : 'H') : 'D';
        return FileIO::write(data, bytes, file);
    }
    static int flush(FILE* file) { order += 'F'; return FileIO::flush(file); }
};
std::string OrderedIO::order;

static BootSlot legacy(uint32_t counter) {
    BootSlot slot = BootSlot::make(counter); slot.magic = 0x53453101U;
    slot.check = slot.magic ^ counter ^ 0x91e10da5U; return slot;
}
static void sign(std::vector<unsigned char>& bytes) {
    FaultHeader header; memcpy(&header, bytes.data(), sizeof(header));
    header.crc = checksum(&header, offsetof(FaultHeader, crc));
    header.crc = checksum(bytes.data() + sizeof(header), bytes.size() - sizeof(header), header.crc);
    memcpy(bytes.data(), &header, sizeof(header));
}

int main() {
    char root[] = "/tmp/carvera-storage-XXXXXX"; assert(mkdtemp(root));
    const std::string boot_path = std::string(root) + "/boot";
    const std::string old_path = std::string(root) + "/old", new_path = std::string(root) + "/new";
    BootSlot slots[2] = {BootSlot::make(10), BootSlot::make(10)};
    uint32_t counter = 999;
    put(boot_path.c_str(), slots, sizeof(slots));
    FailingIO::reset(0); assert(advance_boot_counter<FailingIO>(boot_path.c_str(), counter) && counter == 11);
    const unsigned boot_calls = FailingIO::calls;
    for (unsigned fail = 1; fail <= boot_calls; ++fail) {
        put(boot_path.c_str(), slots, sizeof(slots));
        FailingIO::reset(fail); counter = 999;
        assert(!advance_boot_counter<FailingIO>(boot_path.c_str(), counter) && counter == 0);
        assert(advance_boot_counter(boot_path.c_str(), counter) && counter > 10);
    }
    // Corrupt either issued copy. The surviving high watermark prevents ID reuse.
    for (unsigned damaged = 0; damaged < 2; ++damaged) {
        BootSlot pair[2] = {BootSlot::make(17), BootSlot::make(17)};
        pair[damaged].check ^= 1; put(boot_path.c_str(), pair, sizeof(pair));
        assert(advance_boot_counter(boot_path.c_str(), counter) && counter == 18);
    }
    BootSlot old_slots[2] = {legacy(20), legacy(19)};
    put(boot_path.c_str(), old_slots, sizeof(old_slots));
    assert(advance_boot_counter(boot_path.c_str(), counter) && counter == 21);
    old_slots[0].magic = 0; put(boot_path.c_str(), old_slots, sizeof(old_slots));
    assert(!advance_boot_counter(boot_path.c_str(), counter) && counter == 0);
    slots[0] = slots[1] = BootSlot::make(UINT32_MAX); put(boot_path.c_str(), slots, sizeof(slots));
    assert(!advance_boot_counter(boot_path.c_str(), counter) && counter == 0);

    Event events[3] = {};
    for (unsigned i = 0; i < 3; ++i) {
        events[i].sequence = UINT32_MAX - 1 + i;
        events[i].tick = i == 0 ? UINT32_MAX - 10 : i * 10;
        events[i].wraps = i == 0 ? 0 : 1;
        events[i].kind = i == 2 ? 3 : 1;
        events[i].halt = i == 2 ? 24 : 0;
        events[i].x_steps_per_mm = events[i].y_steps_per_mm = events[i].z_steps_per_mm = 400;
        events[i].context_age_us = events[i].spindle_age_us = UINT32_MAX;
    }
    const FaultHeader initial = {0x53454601U, 1, 0, 3, 20, 1, 24, 0};
    FaultHeader header = initial, verified;
    const auto source = [&events](unsigned i) -> const Event& { return events[i]; };
    assert(write_fault(old_path.c_str(), header, source));
    const auto sealed = get(old_path.c_str());
    header = initial; assert(write_fault<OrderedIO>(new_path.c_str(), header, source));
    assert(OrderedIO::order == "HDDDFCF");
    FailingIO::reset(0); header = initial;
    assert(write_fault<FailingIO>(new_path.c_str(), header, source));
    const unsigned write_calls = FailingIO::calls;
    for (unsigned fail = 1; fail <= write_calls; ++fail) {
        header = initial; FailingIO::reset(fail);
        assert(!write_fault<FailingIO>(new_path.c_str(), header, source));
        assert(get(old_path.c_str()) == sealed);
        assert(read_fault(old_path.c_str(), verified) && verified.sequence == 0);
    }
    FailingIO::reset(0); assert(read_fault<FailingIO>(old_path.c_str(), verified));
    const unsigned read_calls = FailingIO::calls;
    for (unsigned fail = 1; fail <= read_calls; ++fail) {
        FailingIO::reset(fail); assert(!read_fault<FailingIO>(old_path.c_str(), verified));
        assert(verified.magic == 0);
    }
    for (size_t truncated = 0; truncated < sealed.size(); ++truncated) {
        put(new_path.c_str(), sealed.data(), truncated); assert(!read_fault(new_path.c_str(), verified));
    }
    for (size_t corrupted = 0; corrupted < sealed.size(); ++corrupted) {
        auto bytes = sealed; bytes[corrupted] ^= 1;
        put(new_path.c_str(), bytes.data(), bytes.size()); assert(!read_fault(new_path.c_str(), verified));
    }
    auto trailing = sealed; trailing.push_back(0);
    put(new_path.c_str(), trailing.data(), trailing.size()); assert(!read_fault(new_path.c_str(), verified));
    // Even a recomputed CRC cannot make nonsensical records admissible.
    for (unsigned bad = 0; bad < 9; ++bad) {
        auto bytes = sealed; Event event; memcpy(&event, bytes.data() + sizeof(FaultHeader), sizeof(event));
        switch (bad) {
            case 0: event.sequence = 123; break;
            case 1: event.kind = 9; break;
            case 2: event.input_valid = 1 << 20; break;
            case 3: event.inputs = 1; break;
            case 4: event.context_valid = 1; break;
            case 5: event.x_steps_per_mm = 0; break;
            case 6: event.rpm = NAN; break;
            case 7: event.tool_offset = INFINITY; break;
            case 8: event.wraps = 2; break;
        }
        memcpy(bytes.data() + sizeof(FaultHeader), &event, sizeof(event)); sign(bytes);
        put(new_path.c_str(), bytes.data(), bytes.size()); assert(!read_fault(new_path.c_str(), verified));
    }
    for (unsigned bad = 0; bad < 5; ++bad) {
        auto bytes = sealed; FaultHeader wrong; memcpy(&wrong, bytes.data(), sizeof(wrong));
        switch (bad) {
            case 0: wrong.boot_id = 0; break;
            case 1: wrong.count = 17; break;
            case 2: wrong.reason = 0; break;
            case 3: wrong.tick = 21; break;
            case 4: wrong.sequence = 1; break;
        }
        memcpy(bytes.data(), &wrong, sizeof(wrong)); sign(bytes);
        put(new_path.c_str(), bytes.data(), bytes.size()); assert(!read_fault(new_path.c_str(), verified));
    }
    unlink(boot_path.c_str()); unlink(old_path.c_str()); unlink(new_path.c_str()); rmdir(root);
    std::cout << "storage: " << boot_calls << " boot, " << write_calls << " write, " << read_calls
              << " read failure points; all truncations/bit flips and CRC-valid invalid records rejected\n";
}
