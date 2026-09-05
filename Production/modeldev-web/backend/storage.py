"""Small, filesystem-backed persistence helpers for simulation runs."""

from __future__ import annotations

import json
import os
from collections import deque
from pathlib import Path
from typing import Any, Mapping


MANIFEST_SCHEMA_VERSION = 2
SUPPORTED_MANIFEST_SCHEMA_VERSIONS = frozenset({1, MANIFEST_SCHEMA_VERSION})
MANIFEST_FILENAME = "run.json"
LOG_FILENAME = "run.log"
MAX_MANIFEST_BYTES = 1024 * 1024


class RunStorageError(ValueError):
    """Raised when a persisted run manifest is missing or malformed."""


def write_manifest(run_dir: Path, payload: Mapping[str, Any]) -> None:
    """Atomically replace one run manifest in its own directory."""

    target = run_dir / MANIFEST_FILENAME
    temporary = run_dir / f".{MANIFEST_FILENAME}.tmp"
    serialized = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        indent=2,
        sort_keys=True,
    )
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(serialized)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, target)


def read_manifest(run_dir: Path) -> dict[str, Any]:
    path = run_dir / MANIFEST_FILENAME
    try:
        size = path.stat().st_size
        if size > MAX_MANIFEST_BYTES:
            raise RunStorageError("run manifest exceeds the size limit")
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RunStorageError("run manifest is not readable JSON") from exc
    if not isinstance(payload, dict):
        raise RunStorageError("run manifest must be a JSON object")
    return payload


def append_log(run_dir: Path, line: str) -> None:
    with (run_dir / LOG_FILENAME).open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(line)
        stream.write("\n")


def read_log_tail(run_dir: Path, max_lines: int = 400) -> deque[str]:
    lines: deque[str] = deque(maxlen=max_lines)
    path = run_dir / LOG_FILENAME
    try:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for line in stream:
                clean = line.rstrip("\r\n")[:4096]
                if clean:
                    lines.append(clean)
    except FileNotFoundError:
        pass
    return lines
