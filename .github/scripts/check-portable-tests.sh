#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
test_dir="$(mktemp -d "${TMPDIR:-/tmp}/carvera-host-tests.XXXXXX")"
trap 'rm -rf "$test_dir"' EXIT
compiler="${CXX:-c++}"

for source in \
  tests/TEST_sensor_evidence/test_state.cpp \
  tests/TEST_sensor_evidence/test_storage.cpp \
  tests/TEST_gcode_syntax/test_syntax.cpp \
  tests/TEST_player_abort/test_abort.cpp; do
  executable="$test_dir/$(basename "$(dirname "$source")")-$(basename "$source" .cpp)"
  "$compiler" -std=c++11 -O1 -g -fno-omit-frame-pointer \
    -fsanitize=address,undefined -fno-sanitize-recover=all \
    -Wall -Wextra -I "$repo_root/src/libs" "$repo_root/$source" -o "$executable"
  "$executable"
done

"$compiler" -std=c++11 -O1 -g -fno-omit-frame-pointer \
  -fsanitize=address,undefined -fno-sanitize-recover=all \
  -Wall -Wextra -Wno-unused-parameter \
  -I "$repo_root/tests/TEST_fat_open_errno" \
  -I "$repo_root/src/libs/ChaNFS/CHAN_FS" -I "$repo_root/src/libs" \
  -include "$repo_root/tests/TEST_fat_open_errno/stubs.h" \
  "$repo_root/src/libs/ChaNFS/FATFileSystem.cpp" \
  "$repo_root/tests/TEST_fat_open_errno/test_open.cpp" \
  -o "$test_dir/fat-open"
mkdir "$test_dir/fat-storage"
"$test_dir/fat-open" "$test_dir/fat-storage"

python3 "$repo_root/tests/TEST_player_abort/test_wiring.py"
python3 "$repo_root/tests/TEST_bank_attestation/test_player_whitelist.py"
python3 "$repo_root/tests/TEST_version_script/test_version.py"
echo "Portable production-helper tests passed with address and undefined-behavior sanitizers."
