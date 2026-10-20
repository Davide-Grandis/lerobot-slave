#!/usr/bin/python3 -I
"""Root-owned helper: never execute a command supplied by the follower."""

import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

DIRECTORY = Path("/var/lib/cf-robot")
OPERATIONS = {
    "sudo reboot": ["/usr/sbin/shutdown", "-r", "+1"],
    "sudo shutdown -h now": ["/usr/sbin/shutdown", "-h", "+1"],
}


def main():
    request_file = DIRECTORY / "power-request"
    with request_file.open(encoding="ascii") as source:
        request = json.loads(source.read(1024))
    request_file.unlink()
    if (
        not isinstance(request, dict)
        or set(request) != {"command", "request_id"}
        or not isinstance(request["command"], str)
        or request["command"] not in OPERATIONS
        or not isinstance(request["request_id"], str)
        or not re.fullmatch(r"[0-9a-f]{32}", request["request_id"])
    ):
        raise ValueError("invalid power request")
    try:
        result = subprocess.run(
            OPERATIONS[request["command"]], stdin=subprocess.DEVNULL,
            capture_output=True, text=True, timeout=10, check=False,
        )
        status = {
            "request_id": request["request_id"], "exit_code": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr,
        }
    except OSError as exc:
        status = {
            "request_id": request["request_id"], "exit_code": 1,
            "stdout": "", "stderr": str(exc),
        }
    except subprocess.TimeoutExpired:
        status = {
            "request_id": request["request_id"], "exit_code": 1,
            "stdout": "", "outcome_unknown": True,
            "stderr": "Power command timed out; outcome is unknown. Do not retry blindly.\n",
        }
    # The parent is writable by robot; use a safely-created file, not a fixed
    # temporary path that could be replaced by a symlink before a root write.
    with tempfile.NamedTemporaryFile(mode="w", dir=DIRECTORY, delete=False) as output:
        json.dump(status, output)
        os.fchmod(output.fileno(), 0o644)
        temporary = output.name
    os.replace(temporary, DIRECTORY / "power-status.json")


if __name__ == "__main__":
    main()
