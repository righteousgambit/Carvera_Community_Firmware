#ifndef PLAYERPUBLICACCESS_H
#define PLAYERPUBLICACCESS_H

#include <cstdint>
#include <string>

#define player_checksum           CHECKSUM("player")
#define is_playing_checksum       CHECKSUM("is_playing")
#define is_suspended_checksum     CHECKSUM("is_suspended")
#define abort_play_checksum       CHECKSUM("abort_play")
#define get_progress_checksum     CHECKSUM("progress")
#define inner_playing_checksum    CHECKSUM("inner_playing")
#define restart_job_checksum    CHECKSUM("restart_job")
#define suspend_play_checksum   CHECKSUM("suspend_play")
#define resume_play_checksum    CHECKSUM("resume_play")

#ifdef WAVEFORM_ATTESTATION
#define bank_track_checksum CHECKSUM("bank_track")
#define bank_resume_snapshot_checksum CHECKSUM("bank_resume_snapshot")
#define bank_execution_checksum CHECKSUM("bank_execution")
enum bank_execution_phase : uint32_t {
    BANK_UNBOUND, BANK_ARMED, BANK_RUNNING, BANK_PAUSED, BANK_EOF_REACHED,
    BANK_EOF_SETTLED, BANK_ABORTED, BANK_INVALIDATED, BANK_IO_ERROR
};
struct bank_track_request {
    uint8_t nonce[16];
    uint8_t context[32];
    bool accepted;
};
struct bank_execution_record {
    uint8_t nonce[16];
    uint8_t context[32];
    uint8_t program_sha512[64];
    uint8_t filename_sha512[64];
    uint32_t size, cursor, parsed_lines, phase, generation, flags;
    uint8_t consumed_sha512[64];
    uint32_t consumed_size, consumed_verified;
};
static_assert(sizeof(bank_execution_record) == 272, "Fixed execution record layout");
#endif

struct pad_progress {
    unsigned int percent_complete;
    unsigned long played_lines;
    unsigned long elapsed_secs;
    std::string filename;
    bool is_playing;  // true only while file is actively playing (not paused, not finished)
    unsigned long parsed_lines;
};
#endif
