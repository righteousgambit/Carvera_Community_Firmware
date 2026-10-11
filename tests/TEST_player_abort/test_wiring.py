"""Check that production's early return and waits cannot bypass tested disposal."""
from pathlib import Path

root = Path(__file__).resolve().parents[2]
source = (root / "src/modules/utils/player/Player.cpp").read_text()
body = source.split("void Player::abort_command(", 1)[1].split("void Player::clear_buffered_queue", 1)[0]
call = "player_abort::dispose(parameters, this->buffered_queue, this->macro_file_queue);"
assert body.count(call) == 1
disposal = body.index(call)
receipt = "player_abort::Receipt<StreamOutput> recovery_receipt(recovery_nonce, stream);"
assert body.count(receipt) == 1
assert disposal < body.index(receipt) < body.index("#ifdef WAVEFORM_ATTESTATION")
assert "{RB:" not in body
assert disposal < body.index("PublicData::set_value")
assert disposal < body.index("if(!playing_file && current_file_handler == NULL)")
assert disposal < body.index("return;")
assert disposal < body.index("wait_for_idle()")
assert disposal < body.index("flush_queue()")
assert body.count("if (parameters.empty())") == 2
assert '#include "libs/PlayerAbort.h"' in source
print("PASS: production abort disposes at entry; shared RAII receipt encloses every return and wait")
