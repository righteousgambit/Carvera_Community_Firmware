#ifndef PLAYER_ABORT_H
#define PLAYER_ABORT_H

#include <stdint.h>
#include <string>

namespace player_abort {
inline bool space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// Only a complete recovery option enables the correlated disposal receipt.
inline uint32_t recovery_nonce(const std::string& parameters)
{
    size_t i = 0;
    while (i < parameters.size() && space(parameters[i])) ++i;
    if (parameters.compare(i, 2, "-R") != 0) return 0;
    i += 2;
    if (i == parameters.size() || !space(parameters[i])) return 0;
    while (i < parameters.size() && space(parameters[i])) ++i;
    const size_t first_digit = i;
    uint32_t nonce = 0;
    while (i < parameters.size() && parameters[i] >= '0' && parameters[i] <= '9') {
        const uint32_t digit = static_cast<uint32_t>(parameters[i++] - '0');
        if (nonce > (UINT32_MAX - digit) / 10) return 0;
        nonce = nonce * 10 + digit;
    }
    if (i == first_digit || nonce == 0) return 0;
    while (i < parameters.size() && space(parameters[i])) ++i;
    return i == parameters.size() ? nonce : 0;
}

template<class BufferedQueue, class MacroQueue>
uint32_t dispose(std::string& parameters, BufferedQueue& buffered, MacroQueue& macros)
{
    const uint32_t nonce = recovery_nonce(parameters);
    while (!buffered.empty()) buffered.pop();
    while (!macros.empty()) macros.pop();
    if (nonce != 0) {
        // Empty parameters retain normal abort's spindle/flush behavior.
        parameters.clear();
    }
    return nonce;
}

// The receipt is delayed until abort's return, including every early return.
template<class Stream>
class Receipt {
public:
    Receipt(uint32_t nonce, Stream* stream) : nonce(nonce), stream(stream) {}
    ~Receipt() {
        if (nonce != 0) stream->printf("{RB:1,%lu}\r\n", static_cast<unsigned long>(nonce));
    }
    Receipt(const Receipt&) = delete;
    Receipt& operator=(const Receipt&) = delete;
private:
    uint32_t nonce;
    Stream* stream;
};
}

#endif
