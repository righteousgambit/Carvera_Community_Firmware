"""Every registered attestation command must survive Player's later callback."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
shell = (root / "src/modules/utils/simpleshell/SimpleShell.cpp").read_text()
player = (root / "src/modules/utils/player/Player.cpp").read_text()
registered = set(re.findall(r'\{"(bank_[a-z0-9_]+)",\s*SimpleShell::\w+\}', shell))
assert registered == {"bank_track", "bank_attest", "bank_exec", "bank_exec2", "bank_exec3", "bank_exec4", "bank_exec5"}
callback = player.split("void Player::on_console_line_received", 1)[1].split("// Act depending on command", 1)[0]
guard = re.search(r'#ifdef WAVEFORM_ATTESTATION\s+if \((.*?)\) bank_invalidate\(\);\s+#endif', callback, re.S)
assert guard is not None
assert "!bank_dispatching_line" in guard.group(1)
preserved = set(re.findall(r'cmd != "([^\"]+)"', guard.group(1)))
assert registered <= preserved, f"Attestation commands invalidated by Player: {registered - preserved}"
assert not {"bank_exec50", "bank_exec6", "reset", "goto"} & preserved
kernel = (root / "src/libs/Kernel.cpp").read_text()
main = (root / "src/main.cpp").read_text()
assert "this->simpleshell    = new SimpleShell()" in kernel
assert "kernel->add_module( new Player() )" in main
assert "this->hooks[id_event].push_back(mod)" in kernel
assert "for (auto m : hooks[id_event])" in kernel
print("PASS: all seven production attestation commands survive the later Player callback")
