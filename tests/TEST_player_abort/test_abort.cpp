#include "PlayerAbort.h"
#include <cassert>
#include <cstdio>
#include <iostream>
#include <queue>
#include <string>
#include <type_traits>

struct ReceiptStream {
    std::queue<std::string>& buffered;
    std::queue<std::string>& macros;
    std::string output;
    int printf(const char* format, unsigned long nonce) {
        // A receipt cannot be observed until both deferred command queues are empty.
        assert(buffered.empty() && macros.empty());
        char message[64];
        const int count = std::snprintf(message, sizeof(message), format, nonce);
        assert(count > 0 && static_cast<size_t>(count) < sizeof(message));
        output += message;
        return count;
    }
};
static_assert(!std::is_copy_constructible<player_abort::Receipt<ReceiptStream>>::value,
              "A receipt must not be copied and emitted twice");
static_assert(!std::is_move_constructible<player_abort::Receipt<ReceiptStream>>::value,
              "A receipt must keep a single scope owner");

static void disposal(const std::string& source, const std::string& expected_parameters,
                     const std::string& receipt, bool populated)
{
    std::queue<std::string> buffered, macros;
    if (populated) {
        buffered.push("M600");
        buffered.push("G0 Z10");
        buffered.push("G0 X20 Y30");
        macros.push("interrupted-macro.nc");
    }
    ReceiptStream stream{buffered, macros, ""};
    std::string parameters = source;
    const uint32_t nonce = player_abort::dispose(parameters, buffered, macros);
    assert(buffered.empty() && macros.empty());
    assert(stream.output.empty());
    {
        player_abort::Receipt<ReceiptStream> completion(nonce, &stream);
        // A blocked abort wait must not expose a completion receipt.
        for (unsigned wait_tick = 0; wait_tick != 10; ++wait_tick) assert(stream.output.empty());
    }
    assert(parameters == expected_parameters && stream.output == receipt);
    // With no playing file, a new job can only inherit its own deferred command.
    buffered.push("next-job-command");
    assert(buffered.size() == 1 && buffered.front() == "next-job-command");
}

static void early_return(std::string parameters, ReceiptStream& stream, bool halted)
{
    const uint32_t nonce = player_abort::dispose(parameters, stream.buffered, stream.macros);
    player_abort::Receipt<ReceiptStream> completion(nonce, &stream);
    assert(stream.output.empty());
    if (halted) return;
    // The idle/no-file path also returns without entering motion waits.
    return;
}

int main()
{
    for (bool populated : {false, true}) {
        disposal("-R 1", "", "{RB:1,1}\r\n", populated);
        disposal(" \t-R\t4294967295\r\n", "", "{RB:1,4294967295}\r\n", populated);
        disposal("-R 00000000000000000001", "", "{RB:1,1}\r\n", populated);
        for (const std::string source : {"", "-b", "-R", "-R ", "-R 0", "-R 4294967296",
                "-R 99999999999999999999", "-R -1", "-R +1", "-R 0x1", "-R 1 extra",
                "-R 1 -R 2", "-R1", "-r 1", "other -R 1", "-R 1.0", "-R 1e2"}) {
            disposal(source, source, "", populated);
        }
        const std::string embedded_nul("-R 1\0junk", 9);
        disposal(embedded_nul, embedded_nul, "", populated);
    }
    for (bool halted : {false, true}) {
        std::queue<std::string> buffered, macros;
        buffered.push("M600");
        macros.push("return.nc");
        ReceiptStream stream{buffered, macros, ""};
        early_return("-R 42", stream, halted);
        assert(stream.output == "{RB:1,42}\r\n");
    }
    std::cout << "PASS: exact nonce, early queue disposal, receipt once at scope exit, no receipt during wait\n";
}
