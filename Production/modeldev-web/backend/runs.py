"""Single-worker subprocess orchestration and persisted run history."""

from __future__ import annotations

import locale
import math
import os
import re
import subprocess
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Literal, Mapping

from .parsers import ParsedResult, ResultParseError, ResultParser
from .registry import ModelRegistry, ModelSpec
from .storage import (
    MANIFEST_FILENAME,
    MANIFEST_SCHEMA_VERSION,
    SUPPORTED_MANIFEST_SCHEMA_VERSIONS,
    RunStorageError,
    append_log,
    read_log_tail,
    read_manifest,
    write_manifest,
)


RunStatus = Literal["queued", "running", "succeeded", "failed", "cancelled"]
TERMINAL_STATUSES: frozenset[RunStatus] = frozenset(
    {"succeeded", "failed", "cancelled"}
)
RUN_ID_PATTERN = re.compile(r"^[0-9a-f]{32}$")


class RunManagerError(RuntimeError):
    pass


class ModelNotFoundError(RunManagerError):
    pass


class ModelUnavailableError(RunManagerError):
    pass


class WorkerBusyError(RunManagerError):
    pass


class RunNotFoundError(RunManagerError):
    pass


class RunNotCancellableError(RunManagerError):
    pass


class ResultUnavailableError(RunManagerError):
    pass


@dataclass(slots=True)
class RunRecord:
    run_id: str
    model_id: str
    model_name: str
    name: str
    parameters: dict[str, float]
    modules: dict[str, dict[str, object]]
    parser_id: str
    result_relative_path: Path
    result_schema_version: int
    status: RunStatus
    created_at: datetime
    run_dir: Path
    command: tuple[str, ...] = field(default=(), repr=False)
    started_at: datetime | None = None
    finished_at: datetime | None = None
    return_code: int | None = None
    error: str | None = None
    summary: dict[str, int | float] | None = None
    provenance: dict[str, int | str] | None = None
    logs: deque[str] = field(default_factory=lambda: deque(maxlen=400))
    cancel_requested: bool = False
    process: subprocess.Popen[str] | None = field(default=None, repr=False)


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _parse_datetime(value: object, *, required: bool = False) -> datetime | None:
    if value is None and not required:
        return None
    if not isinstance(value, str):
        raise RunStorageError("persisted timestamp must be an ISO-8601 string")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise RunStorageError("persisted timestamp is invalid") from exc
    if parsed.tzinfo is None:
        raise RunStorageError("persisted timestamp must include a timezone")
    return parsed.astimezone(timezone.utc)


def _numeric_mapping(value: object, field_name: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise RunStorageError(f"persisted {field_name} must be an object")
    normalized: dict[str, float] = {}
    for key, item in value.items():
        if not isinstance(key, str) or isinstance(item, bool) or not isinstance(
            item, (int, float)
        ):
            raise RunStorageError(f"persisted {field_name} is invalid")
        number = float(item)
        if not math.isfinite(number):
            raise RunStorageError(f"persisted {field_name} must be finite")
        normalized[key] = number
    return normalized


def _module_mapping(value: object, field_name: str = "modules") -> dict[str, dict[str, object]]:
    if not isinstance(value, dict):
        raise RunStorageError(f"persisted {field_name} must be an object")
    normalized: dict[str, dict[str, object]] = {}
    for slot_id, raw_selection in value.items():
        if not isinstance(slot_id, str) or not slot_id or not isinstance(
            raw_selection, dict
        ):
            raise RunStorageError(f"persisted {field_name} is invalid")
        if set(raw_selection) != {"id", "name", "parameters"}:
            raise RunStorageError(f"persisted {field_name} is invalid")
        option_id = raw_selection.get("id")
        option_name = raw_selection.get("name")
        if not isinstance(option_id, str) or not option_id:
            raise RunStorageError(f"persisted {field_name} option id is invalid")
        if not isinstance(option_name, str) or not option_name:
            raise RunStorageError(f"persisted {field_name} option name is invalid")
        normalized[slot_id] = {
            "id": option_id,
            "name": option_name,
            "parameters": _numeric_mapping(
                raw_selection.get("parameters"), f"{field_name}.{slot_id}.parameters"
            ),
        }
    return normalized


def _copy_modules(
    modules: Mapping[str, Mapping[str, object]],
) -> dict[str, dict[str, object]]:
    return {
        slot_id: {
            "id": selection["id"],
            "name": selection["name"],
            "parameters": dict(selection["parameters"]),
        }
        for slot_id, selection in modules.items()
    }


class RunManager:
    """Run one trusted model subprocess at a time and persist every run."""

    def __init__(
        self,
        registry: ModelRegistry,
        parsers: Mapping[str, ResultParser],
        run_root: Path,
        *,
        run_timeout_seconds: float = 300.0,
        result_max_points: int = 5000,
        termination_grace_seconds: float = 2.0,
        popen_factory: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
    ) -> None:
        if run_timeout_seconds <= 0:
            raise ValueError("run_timeout_seconds must be positive")
        if result_max_points < 2:
            raise ValueError("result_max_points must be at least 2")

        self.registry = registry
        self.parsers = dict(parsers)
        self.run_root = run_root.resolve()
        self.run_timeout_seconds = float(run_timeout_seconds)
        self.result_max_points = int(result_max_points)
        self.termination_grace_seconds = max(0.1, float(termination_grace_seconds))
        self._popen_factory = popen_factory
        self._lock = threading.RLock()
        self._runs: dict[str, RunRecord] = {}
        self._active_run_id: str | None = None
        self._threads: dict[str, threading.Thread] = {}
        self._result_cache: dict[tuple[str, int, int, int], ParsedResult] = {}
        self._storage_warnings: list[str] = []
        self._load_persisted_runs()

    def list_models(self) -> tuple[ModelSpec, ...]:
        return self.registry.list()

    def list_runs(
        self,
        *,
        limit: int = 50,
        model_id: str | None = None,
        status: RunStatus | None = None,
    ) -> list[dict[str, object]]:
        with self._lock:
            records = [
                record
                for record in self._runs.values()
                if (model_id is None or record.model_id == model_id)
                and (status is None or record.status == status)
            ]
            records.sort(key=lambda item: (item.created_at, item.run_id), reverse=True)
            return [self._summary_snapshot(record) for record in records[:limit]]

    def start(
        self,
        model_id: str,
        *,
        name: str | None = None,
        parameters: Mapping[str, object] | None = None,
        modules: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        model = self.registry.get(model_id)
        if model is None:
            raise ModelNotFoundError(f"unknown model id: {model_id}")
        normalized_parameters, normalized_modules, command = model.prepare_run(
            parameters, modules
        )
        if command is None:
            raise ModelUnavailableError(f"model executable is unavailable: {model_id}")

        normalized_name = (name or "").strip() or f"{model.name} 仿真"
        with self._lock:
            if self._active_run_id is not None:
                raise WorkerBusyError("the single simulation worker is busy")

            self.run_root.mkdir(parents=True, exist_ok=True)
            run_id = uuid.uuid4().hex
            run_dir = (self.run_root / run_id).resolve()
            run_dir.relative_to(self.run_root)
            run_dir.mkdir(parents=False, exist_ok=False)
            record = RunRecord(
                run_id=run_id,
                model_id=model.id,
                model_name=model.name,
                name=normalized_name,
                parameters=normalized_parameters,
                modules=normalized_modules,
                parser_id=model.parser_id,
                result_relative_path=model.result_relative_path,
                result_schema_version=model.result_schema_version,
                status="queued",
                created_at=_utc_now(),
                run_dir=run_dir,
                command=command,
                provenance=self._build_provenance(command),
            )
            self._persist(record)
            self._runs[run_id] = record
            self._active_run_id = run_id
            worker = threading.Thread(
                target=self._execute,
                args=(run_id,),
                name=f"modeldev-run-{run_id[:8]}",
                daemon=True,
            )
            self._threads[run_id] = worker
            worker.start()
            return self._snapshot(record)

    def get(self, run_id: str) -> dict[str, object]:
        with self._lock:
            return self._snapshot(self._require_run(run_id))

    def cancel(self, run_id: str) -> dict[str, object]:
        process: subprocess.Popen[str] | None = None
        with self._lock:
            record = self._require_run(run_id)
            if record.status in TERMINAL_STATUSES:
                raise RunNotCancellableError("run is already complete")
            record.cancel_requested = True
            record.status = "cancelled"
            record.error = None
            if record.started_at is None:
                record.finished_at = _utc_now()
                if self._active_run_id == run_id:
                    self._active_run_id = None
            process = record.process
            self._persist(record)
            snapshot = self._snapshot(record)

        if process is not None and process.poll() is None:
            self._terminate(process)
        return snapshot

    def result_available(self, run_id: str) -> bool:
        with self._lock:
            record = self._require_run(run_id)
            return self._result_path(record).is_file()

    def result_path(self, run_id: str) -> Path:
        with self._lock:
            record = self._require_run(run_id)
            if record.status != "succeeded":
                raise ResultUnavailableError("run has not completed successfully")
            result_path = self._result_path(record)
        if not result_path.is_file():
            raise ResultUnavailableError("run result is not available")
        return result_path

    def parse_result(
        self, run_id: str, max_points: int | None = None
    ) -> ParsedResult:
        point_limit = self.result_max_points if max_points is None else int(max_points)
        if point_limit < 2 or point_limit > self.result_max_points:
            raise ValueError(
                f"max_points must be between 2 and {self.result_max_points}"
            )

        with self._lock:
            record = self._require_run(run_id)
            if record.status != "succeeded":
                raise ResultUnavailableError("run has not completed successfully")
            parser = self._parser_for(record)
            result_path = self._result_path(record)

        if not result_path.is_file():
            raise ResultUnavailableError("run result is not available")
        stat = result_path.stat()
        cache_key = (run_id, point_limit, stat.st_size, stat.st_mtime_ns)
        with self._lock:
            cached = self._result_cache.get(cache_key)
        if cached is not None:
            return cached

        result = parser.parse(result_path, point_limit)
        with self._lock:
            self._result_cache[cache_key] = result
            while len(self._result_cache) > 8:
                self._result_cache.pop(next(iter(self._result_cache)))
        return result

    def shutdown(self) -> None:
        with self._lock:
            active_id = self._active_run_id
            record = self._runs.get(active_id) if active_id is not None else None
            if record is not None:
                record.cancel_requested = True
                if record.status not in TERMINAL_STATUSES:
                    record.status = "cancelled"
                    record.error = None
                    self._persist(record)
                process = record.process
            else:
                process = None
            threads = list(self._threads.values())

        if process is not None and process.poll() is None:
            self._terminate(process)
        for thread in threads:
            thread.join(timeout=self.termination_grace_seconds + 1.0)
        if process is not None and process.poll() is None:
            self._kill(process)

    def _execute(self, run_id: str) -> None:
        process: subprocess.Popen[str] | None = None
        reader: threading.Thread | None = None
        try:
            with self._lock:
                record = self._require_run(run_id)
                if record.cancel_requested or record.status == "cancelled":
                    return
                record.status = "running"
                record.started_at = _utc_now()
                command = list(record.command)
                run_dir = record.run_dir
                self._persist(record)

            encoding = locale.getpreferredencoding(False) or "utf-8"
            process = self._popen_factory(
                command,
                cwd=run_dir,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                shell=False,
                text=True,
                encoding=encoding,
                errors="replace",
                bufsize=1,
            )
            with self._lock:
                record.process = process

            reader = threading.Thread(
                target=self._collect_stdout,
                args=(run_id, process),
                name=f"modeldev-log-{run_id[:8]}",
                daemon=True,
            )
            reader.start()

            deadline = time.monotonic() + self.run_timeout_seconds
            termination_reason: Literal["cancelled", "timeout"] | None = None
            termination_started = 0.0

            while process.poll() is None:
                now = time.monotonic()
                with self._lock:
                    cancel_requested = record.cancel_requested

                if termination_reason is None and cancel_requested:
                    termination_reason = "cancelled"
                    termination_started = now
                    self._terminate(process)
                elif termination_reason is None and now >= deadline:
                    termination_reason = "timeout"
                    termination_started = now
                    self._terminate(process)
                elif (
                    termination_reason is not None
                    and now - termination_started >= self.termination_grace_seconds
                ):
                    self._kill(process)
                time.sleep(0.05)

            return_code = process.wait()
            if reader is not None:
                reader.join(timeout=1.0)

            parsed_result: ParsedResult | None = None
            parse_error: str | None = None
            with self._lock:
                result_path = self._result_path(record)
                should_validate = (
                    not record.cancel_requested
                    and termination_reason is None
                    and return_code == 0
                    and result_path.is_file()
                )
                try:
                    parser = self._parser_for(record)
                except ResultUnavailableError:
                    parser = None
            if should_validate:
                if parser is None:
                    parse_error = "result parser is unavailable"
                else:
                    try:
                        parsed_result = parser.parse(
                            result_path, self.result_max_points
                        )
                    except ResultParseError as exc:
                        parse_error = str(exc)

            with self._lock:
                record.return_code = return_code
                record.finished_at = _utc_now()
                if record.cancel_requested or termination_reason == "cancelled":
                    record.status = "cancelled"
                    record.error = None
                elif termination_reason == "timeout":
                    record.status = "failed"
                    record.error = (
                        f"run timed out after {self.run_timeout_seconds:g} seconds"
                    )
                elif return_code == 0 and not result_path.is_file():
                    record.status = "failed"
                    record.error = "model process completed without a result file"
                elif return_code != 0:
                    record.status = "failed"
                    record.error = f"model process exited with code {return_code}"
                elif parse_error is not None:
                    record.status = "failed"
                    record.error = f"result validation failed: {parse_error}"
                else:
                    record.status = "succeeded"
                    record.error = None
                    record.summary = (
                        parsed_result.summary if parsed_result is not None else None
                    )
                    if parsed_result is not None:
                        stat = result_path.stat()
                        self._result_cache[
                            (
                                run_id,
                                self.result_max_points,
                                stat.st_size,
                                stat.st_mtime_ns,
                            )
                        ] = parsed_result
                        while len(self._result_cache) > 8:
                            self._result_cache.pop(next(iter(self._result_cache)))
                self._persist(record)
        except Exception as exc:  # Keep worker failures observable through the API.
            with self._lock:
                record = self._runs.get(run_id)
                if record is not None:
                    record.status = "cancelled" if record.cancel_requested else "failed"
                    record.finished_at = _utc_now()
                    if not record.cancel_requested:
                        record.error = f"could not run model process: {type(exc).__name__}"
                    self._persist(record)
        finally:
            with self._lock:
                record = self._runs.get(run_id)
                if record is not None:
                    record.process = None
                    if record.status == "cancelled" and record.finished_at is None:
                        record.finished_at = _utc_now()
                        self._persist(record)
                if self._active_run_id == run_id:
                    self._active_run_id = None
                self._threads.pop(run_id, None)

    def _collect_stdout(
        self, run_id: str, process: subprocess.Popen[str]
    ) -> None:
        if process.stdout is None:
            return
        try:
            for line in process.stdout:
                clean = line.rstrip("\r\n")[:4096]
                if not clean:
                    continue
                with self._lock:
                    record = self._runs.get(run_id)
                    if record is not None:
                        record.logs.append(clean)
                        try:
                            append_log(record.run_dir, clean)
                        except OSError:
                            pass
        finally:
            process.stdout.close()

    @staticmethod
    def _terminate(process: subprocess.Popen[str]) -> None:
        try:
            process.terminate()
        except OSError:
            pass

    @staticmethod
    def _kill(process: subprocess.Popen[str]) -> None:
        try:
            process.kill()
        except OSError:
            pass

    def _require_run(self, run_id: str) -> RunRecord:
        record = self._runs.get(run_id)
        if record is None:
            raise RunNotFoundError(f"unknown run id: {run_id}")
        return record

    def _parser_for(self, record: RunRecord) -> ResultParser:
        if record.result_schema_version != 1:
            raise ResultUnavailableError(
                f"result schema version {record.result_schema_version} is unsupported"
            )
        parser = self.parsers.get(record.parser_id)
        if parser is None:
            raise ResultUnavailableError("result parser is unavailable")
        return parser

    def _result_path(self, record: RunRecord) -> Path:
        relative_path = record.result_relative_path
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise ResultUnavailableError(
                "persisted result path escapes its run directory"
            )
        root_text = os.path.normcase(os.path.realpath(os.fspath(record.run_dir)))
        path_text = os.path.normcase(
            os.path.realpath(os.fspath(record.run_dir / relative_path))
        )
        try:
            contained = os.path.commonpath((root_text, path_text)) == root_text
        except ValueError:
            contained = False
        if not contained:
            raise ResultUnavailableError(
                "persisted result path escapes its run directory"
            )
        return Path(path_text)

    def _snapshot(self, record: RunRecord) -> dict[str, object]:
        snapshot = self._summary_snapshot(record)
        snapshot["logs"] = list(record.logs)
        return snapshot

    def _summary_snapshot(self, record: RunRecord) -> dict[str, object]:
        return {
            "run_id": record.run_id,
            "model_id": record.model_id,
            "model_name": record.model_name,
            "name": record.name,
            "parameters": dict(record.parameters),
            "modules": _copy_modules(record.modules),
            "status": record.status,
            "created_at": record.created_at,
            "started_at": record.started_at,
            "finished_at": record.finished_at,
            "return_code": record.return_code,
            "error": record.error,
            "summary": dict(record.summary) if record.summary is not None else None,
            "result_available": self._result_path(record).is_file(),
            "result_contract": {
                "parser_id": record.parser_id,
                "schema_version": record.result_schema_version,
            },
        }

    @staticmethod
    def _build_provenance(command: tuple[str, ...]) -> dict[str, int | str] | None:
        if not command:
            return None
        executable = Path(command[0])
        try:
            stat = executable.stat()
        except OSError:
            return {"executable_name": executable.name}
        return {
            "executable_name": executable.name,
            "executable_size": stat.st_size,
            "executable_mtime_ns": stat.st_mtime_ns,
        }

    def _persist(self, record: RunRecord) -> None:
        result_path = self._result_path(record)
        artifact: dict[str, int] | None = None
        try:
            stat = result_path.stat()
            artifact = {"size_bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns}
        except OSError:
            pass
        write_manifest(
            record.run_dir,
            {
                "schema_version": MANIFEST_SCHEMA_VERSION,
                "run_id": record.run_id,
                "model_id": record.model_id,
                "model_name": record.model_name,
                "name": record.name,
                "parameters": record.parameters,
                "modules": _copy_modules(record.modules),
                "status": record.status,
                "created_at": record.created_at.isoformat(),
                "started_at": (
                    record.started_at.isoformat() if record.started_at is not None else None
                ),
                "finished_at": (
                    record.finished_at.isoformat()
                    if record.finished_at is not None
                    else None
                ),
                "return_code": record.return_code,
                "error": record.error,
                "summary": record.summary,
                "result_contract": {
                    "parser_id": record.parser_id,
                    "schema_version": record.result_schema_version,
                    "relative_path": record.result_relative_path.as_posix(),
                },
                "artifact": artifact,
                "provenance": record.provenance,
            },
        )

    def _load_persisted_runs(self) -> None:
        if not self.run_root.is_dir():
            return
        for run_dir in self.run_root.iterdir():
            if not run_dir.is_dir() or RUN_ID_PATTERN.fullmatch(run_dir.name) is None:
                continue
            try:
                if (run_dir / MANIFEST_FILENAME).is_file():
                    record = self._record_from_manifest(run_dir)
                else:
                    record = self._import_legacy_run(run_dir)
                    if record is None:
                        continue
                if record.run_id in self._runs:
                    raise RunStorageError("duplicate persisted run id")
                self._runs[record.run_id] = record
                if record.status not in TERMINAL_STATUSES:
                    record.status = "failed"
                    record.finished_at = _utc_now()
                    record.error = "service restarted before the run completed"
                    self._persist(record)
            except (OSError, RunStorageError, ResultUnavailableError) as exc:
                self._storage_warnings.append(f"{run_dir.name}: {exc}")

    def _record_from_manifest(self, run_dir: Path) -> RunRecord:
        payload = read_manifest(run_dir)
        manifest_schema_version = payload.get("schema_version")
        if manifest_schema_version not in SUPPORTED_MANIFEST_SCHEMA_VERSIONS:
            raise RunStorageError("unsupported run manifest schema version")
        run_id = payload.get("run_id")
        if not isinstance(run_id, str) or run_id != run_dir.name:
            raise RunStorageError("run id does not match its directory")
        status = payload.get("status")
        if status not in {"queued", "running", *TERMINAL_STATUSES}:
            raise RunStorageError("persisted run status is invalid")
        model_id = payload.get("model_id")
        model_name = payload.get("model_name")
        name = payload.get("name")
        if not all(isinstance(item, str) and item for item in (model_id, model_name, name)):
            raise RunStorageError("persisted run names are invalid")
        contract = payload.get("result_contract")
        if not isinstance(contract, dict):
            raise RunStorageError("persisted result contract is missing")
        parser_id = contract.get("parser_id")
        schema_version = contract.get("schema_version")
        relative_raw = contract.get("relative_path")
        if not isinstance(parser_id, str) or not parser_id:
            raise RunStorageError("persisted parser id is invalid")
        if not isinstance(schema_version, int) or schema_version < 1:
            raise RunStorageError("persisted result schema version is invalid")
        if not isinstance(relative_raw, str) or not relative_raw:
            raise RunStorageError("persisted result path is invalid")
        relative_path = Path(relative_raw)
        if relative_path.is_absolute():
            raise RunStorageError("persisted result path must be relative")

        return_code = payload.get("return_code")
        if return_code is not None and (
            isinstance(return_code, bool) or not isinstance(return_code, int)
        ):
            raise RunStorageError("persisted return code is invalid")
        error = payload.get("error")
        if error is not None and not isinstance(error, str):
            raise RunStorageError("persisted error is invalid")
        raw_summary = payload.get("summary")
        summary = (
            None
            if raw_summary is None
            else {
                key: int(value) if float(value).is_integer() else float(value)
                for key, value in _numeric_mapping(raw_summary, "summary").items()
            }
        )
        provenance = payload.get("provenance")
        if provenance is not None and not isinstance(provenance, dict):
            raise RunStorageError("persisted provenance is invalid")

        created_at = _parse_datetime(payload.get("created_at"), required=True)
        if created_at is None:
            raise RunStorageError("persisted created_at is required")
        modules = (
            {}
            if manifest_schema_version == 1
            else _module_mapping(payload.get("modules"))
        )
        record = RunRecord(
            run_id=run_id,
            model_id=model_id,
            model_name=model_name,
            name=name,
            parameters=_numeric_mapping(payload.get("parameters", {}), "parameters"),
            modules=modules,
            parser_id=parser_id,
            result_relative_path=relative_path,
            result_schema_version=schema_version,
            status=status,
            created_at=created_at,
            run_dir=run_dir,
            started_at=_parse_datetime(payload.get("started_at")),
            finished_at=_parse_datetime(payload.get("finished_at")),
            return_code=return_code,
            error=error,
            summary=summary,
            provenance=provenance,
            logs=read_log_tail(run_dir),
        )
        self._result_path(record)
        return record

    def _import_legacy_run(self, run_dir: Path) -> RunRecord | None:
        matches: list[tuple[ModelSpec, Path]] = []
        for model in self.registry.list():
            candidate = (run_dir / model.result_relative_path).resolve()
            try:
                candidate.relative_to(run_dir.resolve())
            except ValueError:
                continue
            if candidate.is_file():
                matches.append((model, candidate))
        if len(matches) != 1:
            return None
        model, result_path = matches[0]
        parser = self.parsers.get(model.parser_id)
        if parser is None:
            return None
        timestamp = datetime.fromtimestamp(result_path.stat().st_mtime, tz=timezone.utc)
        try:
            summary = parser.parse(result_path, self.result_max_points).summary
            status: RunStatus = "succeeded"
            error = None
        except ResultParseError as exc:
            summary = None
            status = "failed"
            error = f"legacy result validation failed: {exc}"
        record = RunRecord(
            run_id=run_dir.name,
            model_id=model.id,
            model_name=model.name,
            name=f"{model.name} 历史仿真",
            parameters=model.validate_parameters(),
            modules={},
            parser_id=model.parser_id,
            result_relative_path=model.result_relative_path,
            result_schema_version=model.result_schema_version,
            status=status,
            created_at=timestamp,
            run_dir=run_dir,
            started_at=None,
            finished_at=timestamp,
            return_code=0 if status == "succeeded" else None,
            error=error,
            summary=summary,
            logs=read_log_tail(run_dir),
        )
        self._persist(record)
        return record
