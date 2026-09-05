"""Extensible parsers for ModelDev result artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Protocol

import numpy as np


MAX_RESULT_BYTES = 512 * 1024 * 1024


class ResultParseError(ValueError):
    """Raised when a result artifact does not satisfy its declared schema."""


@dataclass(frozen=True, slots=True)
class ParsedResult:
    summary: dict[str, int | float]
    series: dict[str, list[float]]


class ResultParser(Protocol):
    def parse(self, path: Path, max_points: int) -> ParsedResult: ...


@dataclass(frozen=True, slots=True)
class WhitespaceTrajectoryParser:
    """Parse the legacy headerless whitespace result format.

    Common columns are time, launch-frame NUE position, and speed.  The LLA
    altitude column differs between existing model families and is therefore
    supplied by each parser instance.
    """

    altitude_column: int

    def parse(self, path: Path, max_points: int) -> ParsedResult:
        if max_points < 2:
            raise ResultParseError("max_points must be at least 2")
        if not path.is_file():
            raise ResultParseError("result file does not exist")
        if path.stat().st_size > MAX_RESULT_BYTES:
            raise ResultParseError("result file exceeds the configured size limit")

        columns = (0, 1, 2, 3, 4, self.altitude_column)
        try:
            raw = np.loadtxt(path, dtype=np.float64, usecols=columns, ndmin=2)
        except (OSError, ValueError, IndexError) as exc:
            raise ResultParseError("result file is not a supported trajectory table") from exc

        if raw.shape[0] == 0:
            raise ResultParseError("result file is empty")

        finite_rows = np.all(np.isfinite(raw), axis=1)
        values = raw[finite_rows]
        if values.shape[0] == 0:
            raise ResultParseError("result file has no finite trajectory rows")
        if np.any(np.diff(values[:, 0]) < 0.0):
            raise ResultParseError("trajectory time must be monotonic")

        sample_count = int(values.shape[0])
        returned_count = min(sample_count, max_points)
        if sample_count <= max_points:
            selected = values
        else:
            indices = np.unique(
                np.linspace(0, sample_count - 1, max_points, dtype=np.int64)
            )
            selected = values[indices]
            returned_count = int(selected.shape[0])

        # Legacy result position order is North, Up, East.
        time_s = selected[:, 0]
        north_m = selected[:, 1]
        up_m = selected[:, 2]
        east_m = selected[:, 3]
        speed_m_s = selected[:, 4]
        altitude_m = selected[:, 5]
        final = values[-1]

        return ParsedResult(
            summary={
                "sample_count": sample_count,
                "returned_sample_count": returned_count,
                "duration_s": float(values[-1, 0] - values[0, 0]),
                "max_altitude_m": float(np.max(values[:, 5])),
                "max_speed_m_s": float(np.max(values[:, 4])),
                "final_time_s": float(final[0]),
                "final_north_m": float(final[1]),
                "final_east_m": float(final[3]),
                "final_up_m": float(final[2]),
                "final_speed_m_s": float(final[4]),
                "final_altitude_m": float(final[5]),
            },
            series={
                "time_s": time_s.tolist(),
                "north_m": north_m.tolist(),
                "east_m": east_m.tolist(),
                "up_m": up_m.tolist(),
                "speed_m_s": speed_m_s.tolist(),
                "altitude_m": altitude_m.tolist(),
            },
        )


def build_default_parsers() -> Mapping[str, ResultParser]:
    return {
        "htv2": WhitespaceTrajectoryParser(altitude_column=42),
        "bgm": WhitespaceTrajectoryParser(altitude_column=33),
    }
