from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from backend.parsers import ResultParseError, WhitespaceTrajectoryParser


def test_parser_maps_nue_and_preserves_endpoints(tmp_path: Path):
    rows = np.zeros((101, 6), dtype=np.float64)
    rows[:, 0] = np.arange(101) * 0.1
    rows[:, 1] = np.arange(101) * 2.0
    rows[:, 2] = 50.0 + np.arange(101)
    rows[:, 3] = -np.arange(101) * 3.0
    rows[:, 4] = 200.0 + np.arange(101)
    rows[:, 5] = 1000.0 + np.arange(101) * 5.0
    path = tmp_path / "result.dat"
    np.savetxt(path, rows)

    result = WhitespaceTrajectoryParser(altitude_column=5).parse(path, max_points=7)

    assert result.summary["sample_count"] == 101
    assert result.summary["returned_sample_count"] == 7
    assert result.series["time_s"][0] == 0.0
    assert result.series["time_s"][-1] == 10.0
    assert result.series["north_m"][-1] == 200.0
    assert result.series["east_m"][-1] == -300.0
    assert result.series["up_m"][-1] == 150.0
    assert result.summary["max_altitude_m"] == 1500.0


def test_parser_rejects_non_monotonic_time(tmp_path: Path):
    rows = np.zeros((3, 6), dtype=np.float64)
    rows[:, 0] = [0.0, 2.0, 1.0]
    path = tmp_path / "result.dat"
    np.savetxt(path, rows)

    with pytest.raises(ResultParseError, match="monotonic"):
        WhitespaceTrajectoryParser(altitude_column=5).parse(path, max_points=10)
