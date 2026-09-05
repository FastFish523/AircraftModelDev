from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from backend.app import create_app
from backend.registry import (
    CatalogModelSpec,
    ModelCatalog,
    ModelCategorySpec,
    ModelRegistry,
    ModelSpec,
    ModuleOptionSpec,
    ModuleSlotSpec,
    ParameterSpec,
)


TERMINAL = {"succeeded", "failed", "cancelled"}

FAKE_RUNNER = r'''from __future__ import annotations
import sys
import time
from pathlib import Path

mode = sys.argv[1]
result_path = Path(sys.argv[2])
column_count = int(sys.argv[3])
altitude_column = int(sys.argv[4])

print("fake-run-start", flush=True)
if mode == "slow":
    result_path.parent.mkdir(parents=True, exist_ok=True)
    row = [0.0] * column_count
    row[4] = 300.0
    row[altitude_column] = 1000.0
    result_path.write_text(" ".join(str(value) for value in row) + "\n")
    print("fake-partial-result", flush=True)
    time.sleep(10)
    raise SystemExit(0)

result_path.parent.mkdir(parents=True, exist_ok=True)
with result_path.open("w", encoding="utf-8") as stream:
    for index in range(20):
        row = [0.0] * column_count
        row[0] = index * 0.5
        row[1] = index * 10.0
        row[2] = 100.0 + index * 2.0
        row[3] = -index * 3.0
        row[4] = 300.0 + index
        row[altitude_column] = 1000.0 + index * 10.0
        stream.write(" ".join(str(value) for value in row) + "\n")
print("fake-run-complete", flush=True)
'''


def make_registry(tmp_path: Path) -> ModelRegistry:
    script = tmp_path / "fake_runner.py"
    script.write_text(FAKE_RUNNER, encoding="utf-8")
    python = Path(sys.executable)
    module_gain = ParameterSpec(
        id="gain_scale",
        label="Gain scale",
        description="feedback gain scale",
        default=1.0,
        minimum=0.25,
        maximum=2.0,
        unit="1",
        cli_flag="--control-gain-scale",
    )
    modules = (
        ModuleSlotSpec(
            id="guidance",
            name="Guidance",
            description="test guidance slot",
            cli_flag="--guidance-module",
            default="phase_standard",
            options=(
                ModuleOptionSpec(
                    id="phase_standard",
                    name="Standard guidance",
                    description="default test guidance",
                    cli_value="phase_standard",
                ),
                ModuleOptionSpec(
                    id="phase_pn",
                    name="PN guidance",
                    description="alternate test guidance",
                    cli_value="phase_pn",
                ),
            ),
        ),
        ModuleSlotSpec(
            id="control",
            name="Control",
            description="test control slot",
            cli_flag="--control-module",
            default="p6dof_pi",
            options=(
                ModuleOptionSpec(
                    id="p6dof_pi",
                    name="PI control",
                    description="default test control",
                    cli_value="p6dof_pi",
                    parameters=(module_gain,),
                ),
                ModuleOptionSpec(
                    id="p6dof_p",
                    name="P control",
                    description="alternate test control",
                    cli_value="p6dof_p",
                    parameters=(module_gain,),
                ),
            ),
        ),
    )
    return ModelRegistry(
        (
            ModelSpec(
                id="HTV2",
                name="HTV2",
                description="test HTV2",
                executable_candidates=(python,),
                result_relative_path=Path("Results/HTV2/result.dat"),
                parser_id="htv2",
                parameters=(
                    ParameterSpec(
                        id="gain",
                        label="Gain",
                        description="test numeric parameter",
                        default=1.0,
                        minimum=0.0,
                        maximum=2.0,
                        unit="1",
                        cli_flag="--gain",
                    ),
                ),
                command_suffix=(
                    str(script),
                    "success",
                    "Results/HTV2/result.dat",
                    "49",
                    "42",
                ),
                modules=modules,
            ),
            ModelSpec(
                id="BGM",
                name="BGM",
                description="test BGM",
                executable_candidates=(python,),
                result_relative_path=Path("Results/BGM/result.dat"),
                parser_id="bgm",
                command_suffix=(
                    str(script),
                    "slow",
                    "Results/BGM/result.dat",
                    "38",
                    "33",
                ),
                modules=modules,
            ),
        )
    )


def make_app(
    tmp_path: Path,
    *,
    timeout: float = 2.0,
    registry: ModelRegistry | None = None,
    catalog: ModelCatalog | None = None,
):
    static_dir = tmp_path / "static"
    static_dir.mkdir(exist_ok=True)
    (static_dir / "index.html").write_text(
        "<h1>ModelDev test UI</h1>", encoding="utf-8"
    )
    return create_app(
        registry=registry if registry is not None else make_registry(tmp_path),
        catalog=catalog,
        run_root=tmp_path / "runs",
        static_dir=static_dir,
        run_timeout_seconds=timeout,
        result_max_points=5,
    )


def test_model_catalog_separates_runnable_models_from_source_components(
    tmp_path: Path,
):
    ready_source = tmp_path / "include/HTV2/HTV2Missile.h"
    ready_source.parent.mkdir(parents=True)
    ready_source.write_text("class Missile;\n", encoding="utf-8")
    missing_binary_source = tmp_path / "include/UNBUILT/Missile.h"
    missing_binary_source.parent.mkdir(parents=True)
    missing_binary_source.write_text("class Missile;\n", encoding="utf-8")
    component_source = tmp_path / "include/Util/Atmosphere.h"
    component_source.parent.mkdir(parents=True)
    component_source.write_text("class Atmosphere;\n", encoding="utf-8")
    dangling_source = tmp_path / "include/DANGLING/Missile.h"
    dangling_source.parent.mkdir(parents=True)
    dangling_source.write_text("class Missile;\n", encoding="utf-8")

    base_registry = make_registry(tmp_path)
    registry = ModelRegistry(
        (
            *base_registry.list(),
            ModelSpec(
                id="UNBUILT",
                name="Unbuilt",
                description="registered but missing executable",
                executable_candidates=(tmp_path / "missing.exe",),
                result_relative_path=Path("Results/UNBUILT/result.dat"),
                parser_id="htv2",
            ),
        )
    )
    catalog = ModelCatalog(
        tmp_path,
        (
            ModelCategorySpec(
                id="vehicle",
                name="Vehicle",
                description="test vehicles",
                models=(
                    CatalogModelSpec(
                        id="HTV2",
                        name="HTV2",
                        description="integrated vehicle",
                        kind="vehicle_model",
                        version=None,
                        capabilities=("simulation",),
                        contracts=("POST /api/runs",),
                        source_relative_paths=(Path("include/HTV2/HTV2Missile.h"),),
                        run_model_id="HTV2",
                    ),
                    CatalogModelSpec(
                        id="UNBUILT",
                        name="Unbuilt",
                        description="integrated but not built",
                        kind="vehicle_model",
                        version=None,
                        capabilities=("source",),
                        contracts=("POST /api/runs after build",),
                        source_relative_paths=(Path("include/UNBUILT/Missile.h"),),
                        run_model_id="UNBUILT",
                    ),
                    CatalogModelSpec(
                        id="DANGLING",
                        name="Dangling mapping",
                        description="catalog mapping missing from the runnable registry",
                        kind="vehicle_model",
                        version=None,
                        capabilities=("source",),
                        contracts=("no runnable contract",),
                        source_relative_paths=(Path("include/DANGLING/Missile.h"),),
                        run_model_id="MISSING_REGISTRY_ID",
                    ),
                ),
            ),
            ModelCategorySpec(
                id="environment",
                name="Environment",
                description="test components",
                models=(
                    CatalogModelSpec(
                        id="atmosphere",
                        name="Atmosphere",
                        description="in-process source component",
                        kind="source_component",
                        version=None,
                        capabilities=("density",),
                        contracts=("C++ source component",),
                        source_relative_paths=(Path("include/Util/Atmosphere.h"),),
                    ),
                ),
            ),
        ),
    )

    with TestClient(make_app(tmp_path, registry=registry, catalog=catalog)) as client:
        legacy_models = client.get("/api/models")
        assert legacy_models.status_code == 200
        assert [model["id"] for model in legacy_models.json()["models"]] == [
            "HTV2",
            "BGM",
            "UNBUILT",
        ]

        response = client.get("/api/model-catalog")
        assert response.status_code == 200
        categories = response.json()["categories"]
        assert [category["id"] for category in categories] == [
            "vehicle",
            "environment",
        ]
        entries = {
            model["id"]: model
            for category in categories
            for model in category["models"]
        }
        assert entries["HTV2"]["status"] == "runnable"
        assert entries["HTV2"]["runnable"] is True
        assert entries["HTV2"]["run_model_id"] == "HTV2"
        assert entries["UNBUILT"]["status"] == "build_required"
        assert entries["UNBUILT"]["runnable"] is False
        assert entries["DANGLING"]["status"] == "not_integrated"
        assert entries["DANGLING"]["status_label"] == "运行映射无效"
        assert entries["DANGLING"]["runnable"] is False
        assert entries["atmosphere"] == {
            "id": "atmosphere",
            "name": "Atmosphere",
            "description": "in-process source component",
            "kind": "source_component",
            "version": None,
            "capabilities": ["density"],
            "contracts": ["C++ source component"],
            "evidence": ["include/Util/Atmosphere.h"],
            "source_path": "include/Util/Atmosphere.h",
            "status": "not_integrated",
            "status_label": "随整机运行",
            "run_model_id": None,
            "source_present": True,
            "runnable": False,
        }
        assert client.post("/api/runs", json={"model_id": "atmosphere"}).status_code == 404


def wait_for_terminal(client: TestClient, run_id: str, timeout: float = 3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = client.get(f"/api/runs/{run_id}")
        assert response.status_code == 200
        payload = response.json()
        if payload["status"] in TERMINAL:
            return payload
        time.sleep(0.02)
    pytest.fail("run did not reach a terminal state")


def wait_for_status(
    client: TestClient, run_id: str, expected: str, timeout: float = 2.0
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        payload = client.get(f"/api/runs/{run_id}").json()
        if payload["status"] == expected:
            return payload
        time.sleep(0.02)
    pytest.fail(f"run did not reach status {expected}")


def start_and_wait(
    client: TestClient, payload: dict[str, object], timeout: float = 3.0
) -> tuple[str, dict[str, object]]:
    for _ in range(20):
        response = client.post("/api/runs", json=payload)
        if response.status_code != 409:
            break
        time.sleep(0.02)
    assert response.status_code == 202, response.text
    run_id = response.json()["run_id"]
    detail = wait_for_terminal(client, run_id, timeout=timeout)
    assert detail["status"] == "succeeded", detail
    return run_id, detail


def test_models_static_site_run_and_downsampled_result(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        models = client.get("/api/models")
        assert models.status_code == 200
        model_payload = models.json()["models"]
        assert [model["id"] for model in model_payload] == ["HTV2", "BGM"]
        assert model_payload[0]["parameters"] == [
            {
                "id": "gain",
                "name": "Gain",
                "label": "Gain",
                "description": "test numeric parameter",
                "type": "number",
                "default": 1.0,
                "minimum": 0.0,
                "maximum": 2.0,
                "unit": "1",
            }
        ]
        assert [module["id"] for module in model_payload[0]["modules"]] == [
            "guidance",
            "control",
        ]
        assert model_payload[0]["modules"][1]["default"] == "p6dof_pi"
        assert model_payload[0]["modules"][1]["options"][0]["parameters"][0][
            "id"
        ] == "gain_scale"
        assert "cli_flag" not in json.dumps(model_payload)
        assert "cli_value" not in json.dumps(model_payload)
        assert model_payload[1]["parameters"] == []
        assert "ModelDev test UI" in client.get("/").text

        response = client.post("/api/runs", json={"model_id": "HTV2"})
        assert response.status_code == 202
        assert set(response.json()) == {"run_id", "status"}
        run_id = response.json()["run_id"]

        detail = wait_for_terminal(client, run_id)
        assert detail["status"] == "succeeded"
        assert detail["return_code"] == 0
        assert detail["error"] is None
        assert detail["result_available"] is True
        assert detail["name"] == "HTV2 仿真"
        assert detail["parameters"] == {"gain": 1.0}
        assert detail["modules"] == {
            "guidance": {
                "id": "phase_standard",
                "name": "Standard guidance",
                "parameters": {},
            },
            "control": {
                "id": "p6dof_pi",
                "name": "PI control",
                "parameters": {"gain_scale": 1.0},
            },
        }
        assert detail["summary"]["sample_count"] == 20
        assert "fake-run-start" in detail["logs"]
        assert "fake-run-complete" in detail["logs"]

        result_response = client.get(f"/api/runs/{run_id}/result")
        assert result_response.status_code == 200
        result = result_response.json()
        assert result["run_id"] == run_id
        assert result["model_id"] == "HTV2"
        assert result["summary"]["sample_count"] == 20
        assert result["summary"]["returned_sample_count"] == 5
        assert result["summary"]["max_altitude_m"] == 1190.0
        assert len(result["series"]["time_s"]) == 5
        assert result["series"]["time_s"][0] == 0.0
        assert result["series"]["time_s"][-1] == 9.5
        assert result["series"]["north_m"][-1] == 190.0
        assert result["series"]["east_m"][-1] == -57.0
        assert result["series"]["up_m"][-1] == 138.0


def test_body_rejects_frontend_executable_path_and_unknown_model(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        injected = client.post(
            "/api/runs",
            json={"model_id": "HTV2", "executable": "C:/untrusted/model.exe"},
        )
        assert injected.status_code == 422
        nested_injected = client.post(
            "/api/runs",
            json={
                "model_id": "HTV2",
                "modules": {
                    "control": {
                        "id": "p6dof_pi",
                        "parameters": {},
                        "cli_flag": "--untrusted",
                    }
                },
            },
        )
        assert nested_injected.status_code == 422
        assert client.post("/api/runs", json={"model_id": "UNKNOWN"}).status_code == 404


def test_single_worker_and_cancellation(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        started = client.post("/api/runs", json={"model_id": "BGM"})
        assert started.status_code == 202
        run_id = started.json()["run_id"]
        wait_for_status(client, run_id, "running")
        assert client.get(f"/api/runs/{run_id}/result").status_code == 409

        busy = client.post("/api/runs", json={"model_id": "HTV2"})
        assert busy.status_code == 409

        cancelled = client.post(f"/api/runs/{run_id}/cancel")
        assert cancelled.status_code == 200
        assert cancelled.json() == {"run_id": run_id, "status": "cancelled"}
        assert wait_for_terminal(client, run_id)["status"] == "cancelled"


def test_timeout_is_reported_as_failed(tmp_path: Path):
    with TestClient(make_app(tmp_path, timeout=0.15)) as client:
        started = client.post("/api/runs", json={"model_id": "BGM"})
        run_id = started.json()["run_id"]
        detail = wait_for_terminal(client, run_id)
        assert detail["status"] == "failed"
        assert "timed out" in detail["error"]


def test_unavailable_whitelisted_model(tmp_path: Path):
    registry = ModelRegistry(
        (
            ModelSpec(
                id="HTV2",
                name="HTV2",
                description="missing executable",
                executable_candidates=(tmp_path / "missing.exe",),
                result_relative_path=Path("Results/HTV2/result.dat"),
                parser_id="htv2",
            ),
        )
    )
    static_dir = tmp_path / "static"
    static_dir.mkdir()
    app = create_app(
        registry=registry,
        run_root=tmp_path / "runs",
        static_dir=static_dir,
    )
    with TestClient(app) as client:
        assert client.get("/api/models").json()["models"][0]["available"] is False
        assert client.post("/api/runs", json={"model_id": "HTV2"}).status_code == 503
        assert client.get("/api/runs/not-found").status_code == 404
        assert client.get("/api/runs/not-found/result").status_code == 404


def test_zero_exit_without_result_is_failed(tmp_path: Path):
    script = tmp_path / "no_result.py"
    script.write_text("print('completed without output', flush=True)\n", encoding="utf-8")
    registry = ModelRegistry(
        (
            ModelSpec(
                id="NO_RESULT",
                name="No result",
                description="runner exits zero without a result file",
                executable_candidates=(Path(sys.executable),),
                result_relative_path=Path("Results/none/result.dat"),
                parser_id="htv2",
                command_suffix=(str(script),),
            ),
        )
    )
    static_dir = tmp_path / "static"
    static_dir.mkdir()
    app = create_app(
        registry=registry,
        run_root=tmp_path / "runs",
        static_dir=static_dir,
    )

    with TestClient(app) as client:
        started = client.post("/api/runs", json={"model_id": "NO_RESULT"})
        detail = wait_for_terminal(client, started.json()["run_id"])
        assert detail["status"] == "failed"
        assert detail["return_code"] == 0
        assert detail["result_available"] is False
        assert detail["error"] == "model process completed without a result file"


def test_zero_exit_with_invalid_result_is_failed_before_success(tmp_path: Path):
    script = tmp_path / "invalid_result.py"
    script.write_text(
        "from pathlib import Path\n"
        "p = Path('Results/bad/result.dat')\n"
        "p.parent.mkdir(parents=True)\n"
        "p.write_text('0 1\\n')\n",
        encoding="utf-8",
    )
    registry = ModelRegistry(
        (
            ModelSpec(
                id="BAD_RESULT",
                name="Bad result",
                description="runner writes an invalid result file",
                executable_candidates=(Path(sys.executable),),
                result_relative_path=Path("Results/bad/result.dat"),
                parser_id="htv2",
                command_suffix=(str(script),),
            ),
        )
    )
    with TestClient(make_app(tmp_path, registry=registry)) as client:
        started = client.post("/api/runs", json={"model_id": "BAD_RESULT"})
        detail = wait_for_terminal(client, started.json()["run_id"])
        assert detail["status"] == "failed"
        assert detail["return_code"] == 0
        assert detail["result_available"] is True
        assert detail["summary"] is None
        assert detail["error"].startswith("result validation failed:")


def test_parameter_validation_and_run_echo(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        invalid_payloads = (
            {"model_id": "HTV2", "parameters": {"gain": 3}},
            {"model_id": "HTV2", "parameters": {"gain": True}},
            {"model_id": "HTV2", "parameters": {"unknown": 1}},
            {
                "model_id": "HTV2",
                "modules": {"unknown": {"id": "anything", "parameters": {}}},
            },
            {
                "model_id": "HTV2",
                "modules": {"control": {"id": "unknown", "parameters": {}}},
            },
            {
                "model_id": "HTV2",
                "modules": {
                    "guidance": {
                        "id": "phase_standard",
                        "parameters": {"inactive_parameter": 1},
                    }
                },
            },
            {
                "model_id": "HTV2",
                "modules": {
                    "control": {
                        "id": "p6dof_p",
                        "parameters": {"gain_scale": True},
                    }
                },
            },
            {
                "model_id": "HTV2",
                "modules": {
                    "control": {
                        "id": "p6dof_p",
                        "parameters": {"gain_scale": 3},
                    }
                },
            },
        )
        for payload in invalid_payloads:
            response = client.post("/api/runs", json=payload)
            assert response.status_code == 422

        run_id, detail = start_and_wait(
            client,
            {
                "model_id": "HTV2",
                "name": "custom scenario",
                "parameters": {"gain": 1.5},
                "modules": {
                    "guidance": {"id": "phase_pn", "parameters": {}},
                    "control": {
                        "id": "p6dof_p",
                        "parameters": {"gain_scale": 1.5},
                    },
                },
            },
        )
        assert detail["run_id"] == run_id
        assert detail["name"] == "custom scenario"
        assert detail["parameters"] == {"gain": 1.5}
        assert detail["modules"] == {
            "guidance": {
                "id": "phase_pn",
                "name": "PN guidance",
                "parameters": {},
            },
            "control": {
                "id": "p6dof_p",
                "name": "P control",
                "parameters": {"gain_scale": 1.5},
            },
        }


def test_persisted_history_result_and_logs_survive_registry_change(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        run_id, detail = start_and_wait(
            client,
            {
                "model_id": "HTV2",
                "name": "persist me",
                "parameters": {"gain": 1.25},
                "modules": {
                    "guidance": {"id": "phase_pn", "parameters": {}},
                    "control": {
                        "id": "p6dof_p",
                        "parameters": {"gain_scale": 1.25},
                    },
                },
            },
        )
        assert detail["summary"]["sample_count"] == 20

    run_dir = tmp_path / "runs" / run_id
    manifest = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 2
    assert manifest["status"] == "succeeded"
    assert manifest["name"] == "persist me"
    assert manifest["parameters"] == {"gain": 1.25}
    assert manifest["modules"] == {
        "guidance": {
            "id": "phase_pn",
            "name": "PN guidance",
            "parameters": {},
        },
        "control": {
            "id": "p6dof_p",
            "name": "P control",
            "parameters": {"gain_scale": 1.25},
        },
    }
    assert manifest["result_contract"] == {
        "parser_id": "htv2",
        "relative_path": "Results/HTV2/result.dat",
        "schema_version": 1,
    }
    assert manifest["summary"]["sample_count"] == 20
    assert "fake-run-complete" in (run_dir / "run.log").read_text(encoding="utf-8")

    # The persisted parser/path snapshot keeps an old result readable even when
    # that model is no longer present in the live registry.
    with TestClient(make_app(tmp_path, registry=ModelRegistry(()))) as client:
        history = client.get("/api/runs")
        assert history.status_code == 200
        assert [item["run_id"] for item in history.json()["runs"]] == [run_id]
        restored = client.get(f"/api/runs/{run_id}").json()
        assert restored["name"] == "persist me"
        assert restored["parameters"] == {"gain": 1.25}
        assert restored["modules"] == manifest["modules"]
        assert "fake-run-complete" in restored["logs"]
        assert client.get(f"/api/runs/{run_id}/result").status_code == 200


def test_incomplete_manifest_is_failed_on_restart(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        run_id, _ = start_and_wait(client, {"model_id": "HTV2"})

    manifest_path = tmp_path / "runs" / run_id / "run.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest.update(
        {"status": "running", "finished_at": None, "error": None, "summary": None}
    )
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with TestClient(make_app(tmp_path)) as client:
        detail = client.get(f"/api/runs/{run_id}").json()
        assert detail["status"] == "failed"
        assert detail["finished_at"] is not None
        assert detail["error"] == "service restarted before the run completed"
        assert detail["modules"]["control"]["id"] == "p6dof_pi"
        assert client.get(f"/api/runs/{run_id}/result").status_code == 409


def test_v1_manifest_restores_without_assigning_current_module_defaults(
    tmp_path: Path,
):
    with TestClient(make_app(tmp_path)) as client:
        run_id, _ = start_and_wait(client, {"model_id": "HTV2"})

    manifest_path = tmp_path / "runs" / run_id / "run.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["schema_version"] = 1
    manifest.pop("modules")
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with TestClient(make_app(tmp_path, registry=ModelRegistry(()))) as client:
        restored = client.get(f"/api/runs/{run_id}")
        assert restored.status_code == 200
        assert restored.json()["modules"] == {}
        assert client.get(f"/api/runs/{run_id}/result").status_code == 200


def test_corrupt_and_escaping_manifests_are_ignored(tmp_path: Path):
    run_root = tmp_path / "runs"
    corrupt_dir = run_root / ("a" * 32)
    corrupt_dir.mkdir(parents=True)
    (corrupt_dir / "run.json").write_text("{broken", encoding="utf-8")

    with TestClient(make_app(tmp_path)) as client:
        run_id, _ = start_and_wait(client, {"model_id": "HTV2"})

    manifest_path = run_root / run_id / "run.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["result_contract"]["relative_path"] = "../../outside.dat"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with TestClient(make_app(tmp_path)) as client:
        assert client.get(f"/api/runs/{'a' * 32}").status_code == 404
        assert client.get(f"/api/runs/{run_id}").status_code == 404
        assert client.get("/api/runs").json() == {"runs": []}


def test_export_json_csv_and_raw(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        run_id, _ = start_and_wait(client, {"model_id": "HTV2"})
        result = client.get(f"/api/runs/{run_id}/result").json()

        exported_json = client.get(f"/api/runs/{run_id}/export?format=json")
        assert exported_json.status_code == 200
        assert exported_json.json() == result
        assert "attachment" in exported_json.headers["content-disposition"]

        exported_csv = client.get(f"/api/runs/{run_id}/export?format=csv")
        assert exported_csv.status_code == 200
        csv_lines = exported_csv.text.splitlines()
        assert csv_lines[0] == (
            "time_s,north_m,east_m,up_m,speed_m_s,altitude_m"
        )
        assert len(csv_lines) == 6
        assert exported_csv.headers["x-modeldev-max-points"] == "5"

        exported_raw = client.get(f"/api/runs/{run_id}/export?format=raw")
        raw_path = tmp_path / "runs" / run_id / "Results/HTV2/result.dat"
        assert exported_raw.status_code == 200
        assert exported_raw.content == raw_path.read_bytes()
        assert client.get(f"/api/runs/{run_id}/export?format=bad").status_code == 422


def test_comparison_order_limits_and_history_filters(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        first, _ = start_and_wait(
            client, {"model_id": "HTV2", "name": "first", "parameters": {"gain": 0.5}}
        )
        second, _ = start_and_wait(
            client,
            {
                "model_id": "HTV2",
                "name": "second",
                "parameters": {"gain": 1.5},
                "modules": {
                    "guidance": {"id": "phase_pn", "parameters": {}},
                    "control": {
                        "id": "p6dof_p",
                        "parameters": {"gain_scale": 1.5},
                    },
                },
            },
        )

        history = client.get("/api/runs?limit=1&model_id=HTV2&status=succeeded")
        assert history.status_code == 200
        assert [item["run_id"] for item in history.json()["runs"]] == [second]

        compared = client.post(
            "/api/comparisons",
            json={"run_ids": [second, first], "max_points": 100},
        )
        assert compared.status_code == 200
        payload = compared.json()
        assert [item["run_id"] for item in payload["runs"]] == [second, first]
        assert [item["name"] for item in payload["runs"]] == ["second", "first"]
        assert payload["runs"][0]["modules"]["control"]["id"] == "p6dof_p"
        assert payload["runs"][1]["modules"]["control"]["id"] == "p6dof_pi"
        assert len(payload["runs"][0]["series"]["time_s"]) == 5
        assert payload["units"]["speed_m_s"] == "m/s"

        assert client.post(
            "/api/comparisons", json={"run_ids": [first, first]}
        ).status_code == 422
        assert client.post(
            "/api/comparisons", json={"run_ids": [first, "f" * 32]}
        ).status_code == 404

    with TestClient(make_app(tmp_path, registry=ModelRegistry(()))) as client:
        restored = client.post(
            "/api/comparisons",
            json={"run_ids": [first, second], "max_points": 100},
        )
        assert restored.status_code == 200
        assert [item["run_id"] for item in restored.json()["runs"]] == [
            first,
            second,
        ]


def test_damaged_persisted_result_returns_422(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        run_id, _ = start_and_wait(client, {"model_id": "HTV2"})
        result_path = tmp_path / "runs" / run_id / "Results/HTV2/result.dat"
        result_path.write_text("0 1\n", encoding="utf-8")
        assert client.get(f"/api/runs/{run_id}/result").status_code == 422
        assert client.post(
            "/api/comparisons", json={"run_ids": [run_id, run_id + "x"]}
        ).status_code == 422


def test_unsupported_persisted_result_schema_is_not_misparsed(tmp_path: Path):
    with TestClient(make_app(tmp_path)) as client:
        run_id, _ = start_and_wait(client, {"model_id": "HTV2"})

    manifest_path = tmp_path / "runs" / run_id / "run.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["result_contract"]["schema_version"] = 2
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with TestClient(make_app(tmp_path, registry=ModelRegistry(()))) as client:
        detail = client.get(f"/api/runs/{run_id}")
        assert detail.status_code == 200
        assert detail.json()["result_contract"]["schema_version"] == 2
        result = client.get(f"/api/runs/{run_id}/result")
        assert result.status_code == 409
        assert "unsupported" in result.json()["detail"]
