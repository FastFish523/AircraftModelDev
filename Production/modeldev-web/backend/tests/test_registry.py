from __future__ import annotations

import math
from pathlib import Path

import pytest

from backend.registry import (
    ParameterValidationError,
    build_default_catalog,
    build_default_registry,
)


def test_default_catalog_matches_repository_sources_and_integration_boundary():
    repository_root = Path(__file__).resolve().parents[4]
    catalog = build_default_catalog(repository_root)
    categories = catalog.list_categories()

    assert [category.id for category in categories] == [
        "vehicle",
        "environment",
        "guidance",
        "control",
        "sensor",
        "recorder",
    ]
    models = [model for category in categories for model in category.models]
    assert len({model.id for model in models}) == len(models)
    assert all(catalog.source_present(model) for model in models)
    assert all(model.version is None for model in models)
    assert {
        model.id: model.run_model_id
        for model in models
        if model.run_model_id is not None
    } == {"HTV2": "HTV2", "BGM": "BGM"}
    assert all(
        model.kind == "source_component" and model.run_model_id is None
        for category in categories[1:]
        for model in category.models
    )
    assert all(
        not path.is_absolute() and ".." not in path.parts
        for model in models
        for path in model.source_relative_paths
    )


def test_default_registry_exposes_versioned_trusted_cli_parameters(tmp_path: Path):
    executable = tmp_path / "_Build/out/windows/Release/HTV2_Test.exe"
    executable.parent.mkdir(parents=True)
    executable.write_bytes(b"test executable placeholder")

    model = build_default_registry(tmp_path).get("HTV2")
    assert model is not None
    assert model.result_schema_version == 1
    assert [item.id for item in model.parameters] == [
        "launch_lon_deg",
        "launch_lat_deg",
        "launch_altitude_m",
        "target_lon_deg",
        "target_lat_deg",
        "target_altitude_m",
        "launch_theta_deg",
        "max_sim_time_s",
    ]
    assert model.validate_parameters() == {
        "launch_lon_deg": 120.0,
        "launch_lat_deg": 40.0,
        "launch_altitude_m": 10.0,
        "target_lon_deg": 140.0,
        "target_lat_deg": 20.0,
        "target_altitude_m": 0.0,
        "launch_theta_deg": 89.0,
        "max_sim_time_s": 1000.0,
    }
    assert model.validate_modules() == {
        "guidance": {
            "id": "phase_pull_bias",
            "name": "分阶段拉偏制导",
            "parameters": {
                "navigation_constant": 4.0,
                "hold_start_distance_m": 80000.0,
                "hold_duration_s": 15.0,
                "mount_az_deg": 20.0,
                "mount_el_deg": -10.0,
            },
        },
        "control": {
            "id": "p6dof_pi",
            "name": "六自由度 PI 控制",
            "parameters": {"gain_scale": 1.0, "rudder_limit_deg": 45.0},
        },
    }

    command = model.command({"launch_theta_deg": 60.5})
    assert command is not None
    assert Path(command[0]) == executable.resolve()
    cli = dict(zip(command[1::2], command[2::2]))
    assert cli["--launch-altitude-m"] == "10"
    assert cli["--target-altitude-m"] == "0"
    assert cli["--launch-theta-deg"] == "60.5"
    assert cli["--max-sim-time-s"] == "1000"
    assert cli["--guidance-module"] == "phase_pull_bias"
    assert cli["--guidance-hold-duration-s"] == "15"
    assert cli["--control-module"] == "p6dof_pi"
    assert cli["--control-gain-scale"] == "1"
    assert all(flag.startswith("--") for flag in command[1::2])


@pytest.mark.parametrize(
    "parameters",
    [
        {"launch_lon_deg": "120"},
        {"launch_lon_deg": True},
        {"launch_lon_deg": math.inf},
        {"launch_lat_deg": 90},
        {"untrusted_flag": 1},
        {
            "launch_lon_deg": 120,
            "launch_lat_deg": 40,
            "target_lon_deg": 120.000001,
            "target_lat_deg": 40,
        },
    ],
)
def test_parameter_validation_rejects_unsafe_values(
    tmp_path: Path, parameters: dict[str, object]
):
    model = build_default_registry(tmp_path).get("HTV2")
    assert model is not None
    with pytest.raises(ParameterValidationError):
        model.validate_parameters(parameters)


def test_bgm_adds_cruise_parameters(tmp_path: Path):
    model = build_default_registry(tmp_path).get("BGM")
    assert model is not None
    parameters = {item.id: item for item in model.parameters}
    assert parameters["cruise_altitude_m"].default == 100
    assert parameters["cruise_altitude_m"].minimum == 10
    assert parameters["cruise_altitude_m"].maximum == 20000
    assert parameters["cruise_mach"].default == 0.7
    assert parameters["cruise_mach"].minimum == 0.1
    assert parameters["cruise_mach"].maximum == 5
    modules = {module.id: module for module in model.modules}
    assert modules["guidance"].default == "phase_l1"
    assert [option.id for option in modules["guidance"].options] == [
        "phase_l1",
        "phase_pn",
    ]
    l1_parameters = {
        parameter.id: parameter for parameter in modules["guidance"].options[0].parameters
    }
    assert l1_parameters["first_waypoint_distance_m"].default == 20000
    assert "实时切换触发距离" in l1_parameters["first_waypoint_distance_m"].description


def test_module_selection_uses_only_registered_option_parameters(tmp_path: Path):
    executable = tmp_path / "_Build/out/windows/Release/HTV2_Test.exe"
    executable.parent.mkdir(parents=True)
    executable.write_bytes(b"test executable placeholder")
    model = build_default_registry(tmp_path).get("HTV2")
    assert model is not None

    modules = {
        "guidance": {
            "id": "phase_standard",
            "parameters": {"navigation_constant": 5.5},
        },
        "control": {
            "id": "p6dof_p",
            "parameters": {"gain_scale": 0.75, "rudder_limit_deg": 30},
        },
    }
    normalized = model.validate_modules(modules)
    assert normalized["guidance"]["parameters"] == {"navigation_constant": 5.5}
    command = model.command(modules=modules)
    assert command is not None
    cli = dict(zip(command[1::2], command[2::2]))
    assert cli["--guidance-module"] == "phase_standard"
    assert "--guidance-hold-duration-s" not in cli
    assert cli["--control-module"] == "p6dof_p"
    assert cli["--control-gain-scale"] == "0.75"

    invalid_modules = (
        {"unknown": {"id": "phase_standard", "parameters": {}}},
        {"guidance": {"id": "unknown", "parameters": {}}},
        {
            "guidance": {
                "id": "phase_standard",
                "parameters": {"hold_duration_s": 15},
            }
        },
        {
            "control": {
                "id": "p6dof_p",
                "parameters": {"gain_scale": math.inf},
            }
        },
        {
            "control": {
                "id": "p6dof_p",
                "parameters": {"rudder_limit_deg": 4},
            }
        },
    )
    for supplied in invalid_modules:
        with pytest.raises(ParameterValidationError):
            model.validate_modules(supplied)
