#include "SimpleShell.h"
#include "StreamOutput.h"

#ifndef WAVEFORM_ATTESTATION
void SimpleShell::bank_attest_command(string, StreamOutput *stream)
{
    stream->printf("ERROR: bank_attestation_disabled\n");
}
#else
#include "libs/monocypher/monocypher-ed25519.h"
#include "libs/monocypher/monocypher.h"
#include "libs/FirmwareFileSystem.h"
#include "Kernel.h"
#include "Robot.h"
#include "Conveyor.h"
#include "PublicData.h"
#include "modules/tools/atc/ATCHandlerPublicAccess.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include <cmath>
#include <cstring>
#include <limits>

#ifndef WAVEFORM_ATTESTATION_IMAGE_START
#error "Attestation requires an explicit image address"
#endif
#ifndef WAVEFORM_ATTESTATION_IMAGE_SIZE
#error "Attestation requires an explicit image byte count"
#endif
static_assert(WAVEFORM_ATTESTATION_IMAGE_SIZE > 0 &&
              WAVEFORM_ATTESTATION_IMAGE_START <= 512UL * 1024UL &&
              WAVEFORM_ATTESTATION_IMAGE_SIZE <= 512UL * 1024UL - WAVEFORM_ATTESTATION_IMAGE_START,
              "Image measurement must remain inside LPC1768 flash");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Attestation requires IEEE binary32");
static_assert(MAX_WCS == 9, "Attestation v1 has nine live WCS records");

namespace {
const size_t STATE_SIZE = 452;
const size_t MESSAGE_SIZE = 640;
uint32_t sequence = 0;
void word(uint8_t *&out, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i) *out++ = static_cast<uint8_t>(v >> (8 * i));
}
bool number(uint8_t *&out, float value)
{
    if (!std::isfinite(value)) return false;
    uint32_t bits; memcpy(&bits, &value, 4); word(out, bits); return true;
}
bool tuple(uint8_t *&out, const Robot::wcs_t &v)
{
    return number(out, std::get<0>(v)) && number(out, std::get<1>(v)) &&
           number(out, std::get<2>(v)) && number(out, std::get<3>(v)) && number(out, std::get<4>(v));
}
bool flag(uint16_t checksum, bool &value)
{
    void *data = nullptr;
    if (!PublicData::get_value(player_checksum, checksum, &data) || !data) return false;
    value = *static_cast<bool *>(data); return true;
}
bool snapshot(uint8_t state[STATE_SIZE])
{
    if (!THEKERNEL->factory_set || THEKERNEL->factory_set->MachineModel != CARVERA ||
        !THEKERNEL->eeprom_data || THEKERNEL->is_config_load_error() ||
        !THECONVEYOR->is_idle() || !THECONVEYOR->is_queue_empty()) return false;
    bool playing, inner;
    if (!flag(is_playing_checksum, playing) || !flag(inner_playing_checksum, inner) ||
        playing || inner || THEKERNEL->is_suspending()) return false;
    bank_state bank = {}; tool_status tool = {}; machine_offsets offsets = {};
    if (!PublicData::get_value(atc_handler_checksum, get_bank_state_checksum, &bank) ||
        !PublicData::get_value(atc_handler_checksum, get_tool_status_checksum, &tool) ||
        !PublicData::get_value(atc_handler_checksum, get_machine_offsets_checksum, &offsets) ||
        bank.status != 0 || bank.queued_scripts != 0) return false;
    uint8_t *out = state;
    word(out, static_cast<uint32_t>(tool.active_tool)); word(out, static_cast<uint32_t>(tool.target_tool));
    word(out, bank.status); word(out, bank.queued_scripts); word(out, inner);
    word(out, THEKERNEL->is_halted()); word(out, THEROBOT->get_tool_not_calibrated());
    word(out, THEKERNEL->eeprom_data->current_wcs); word(out, THEROBOT->get_current_wcs());
    word(out, THEKERNEL->factory_set->MachineModel);
    const float values[] = {tool.ref_tool_mz, tool.cur_tool_mz, tool.tool_offset, offsets.clearance_z,
                           THEKERNEL->eeprom_data->REFMZ, THEKERNEL->eeprom_data->TOOLMZ,
                           THEKERNEL->eeprom_data->TLO};
    for (float value : values) if (!number(out, value)) return false;
    for (unsigned i = 0; i < 6; ++i) {
        for (unsigned j = 0; j < 4; ++j) if (!number(out, THEKERNEL->eeprom_data->WCScoord[i][j])) return false;
        if (!number(out, THEKERNEL->eeprom_data->WCSrotation[i])) return false;
    }
    const auto live = THEROBOT->get_wcs_state();
    if (live.size() != 12) return false;
    for (size_t i = 1; i < live.size(); ++i) if (!tuple(out, live[i])) return false;
    for (unsigned i = 0; i < 9; ++i) if (!number(out, THEROBOT->r[i])) return false;
    word(out, THEROBOT->inch_mode); word(out, THEROBOT->absolute_mode);
    return out == state + STATE_SIZE;
}
bool hex(const string &text, uint8_t *out)
{
    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        unsigned v;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else return false;
        if ((i & 1) == 0) out[i / 2] = v << 4; else out[i / 2] |= v;
    }
    return true;
}
void print_hex(StreamOutput *stream, const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i) stream->printf("%02x", data[i]);
}
}

void SimpleShell::bank_attest_command(string parameters, StreamOutput *stream)
{
    // A 128-bit challenge occupies the first half of its 32-byte field;
    // the other half is zero. This keeps the request below the wire limit.
    if (parameters.size() != 97 || parameters[32] != ' ' || sequence == UINT32_MAX) {
        stream->printf("ERROR: bank_attestation_request_invalid\n"); return;
    }
    uint8_t message[MESSAGE_SIZE] = {}, before[STATE_SIZE], after[STATE_SIZE];
    static const char magic[] = "CVATTEST00000001";
    static_assert(sizeof(magic) - 1 == 16, "Fixed protocol marker");
    memcpy(message, magic, sizeof(magic) - 1);
    if (!hex(parameters.substr(0, 32), message + 16) ||
        !hex(parameters.substr(33, 64), message + 48) || !snapshot(before)) {
        stream->printf("ERROR: bank_attestation_state_unavailable\n"); return;
    }
    uint8_t seed[32] = {}, secret[64] = {}, public_key[32], signature[64];
    FILE *file = fwfs::fopen("/sd/waveform-attestation.seed", "rb");
    if (!file) { stream->printf("ERROR: bank_attestation_key_unavailable\n"); return; }
    const bool complete = fwfs::fread(seed, 1, 32, file) == 32 && fwfs::fgetc(file) == EOF;
    fwfs::fclose(file);
    if (!complete) { crypto_wipe(seed, 32); stream->printf("ERROR: bank_attestation_key_unavailable\n"); return; }
    crypto_ed25519_key_pair(secret, public_key, seed);
    memcpy(message + 80, public_key, 32);
    crypto_sha512(message + 112, reinterpret_cast<const uint8_t *>(WAVEFORM_ATTESTATION_IMAGE_START),
                  WAVEFORM_ATTESTATION_IMAGE_SIZE);
    uint8_t *out = message + 176;
    word(out, WAVEFORM_ATTESTATION_IMAGE_START); word(out, WAVEFORM_ATTESTATION_IMAGE_SIZE);
    word(out, ++sequence); memcpy(out, before, STATE_SIZE);
    crypto_ed25519_sign(signature, secret, message, MESSAGE_SIZE);
    crypto_wipe(secret, 64);
    if (!snapshot(after) || memcmp(before, after, STATE_SIZE) != 0) {
        crypto_wipe(signature, 64); stream->printf("ERROR: bank_attestation_state_changed\n"); return;
    }
    // The SD key and self-measured image do not establish protected boot trust.
    stream->printf("{\"schema_version\":\"waveform.carvera.firmware_attestation.v1\",\"message_hex\":\"");
    print_hex(stream, message, MESSAGE_SIZE);
    stream->printf("\",\"signature_hex\":\""); print_hex(stream, signature, 64);
    stream->printf("\"}\n"); crypto_wipe(signature, 64);
}
#endif
