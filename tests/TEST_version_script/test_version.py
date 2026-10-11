"""Version output must serve both Actions outputs and the build's stdout reader."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
for ref, expected in [
    ("refs/tags/v2.3.0", "v2.3.0"),
    ("refs/heads/master", "master-1234567"),
    ("refs/pull/12/merge", "pr-12-1234567"),
    ("refs/other/check", "check-1234567"),
]:
    with tempfile.TemporaryDirectory(prefix="carvera-version-") as directory:
        output = Path(directory) / "outputs"
        output.write_text("EXISTING=kept\n")
        env = dict(os.environ, GITHUB_REF=ref, GITHUB_SHA="1234567890abcdef", GITHUB_OUTPUT=str(output))
        result = subprocess.run(["bash", str(root / ".github/scripts/set-version.sh")],
                                env=env, text=True, capture_output=True, check=True)
        assert result.stdout == f"VERSION={expected}\n"
        assert output.read_text() == f"EXISTING=kept\nVERSION={expected}\n"
        assert result.stdout.split("=", 1)[1].strip() == expected
print("PASS: version is retained in Actions output and emitted for all build event forms")
