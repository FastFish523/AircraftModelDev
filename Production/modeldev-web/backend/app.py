"""FastAPI application for local ModelDev simulation workflows."""

from __future__ import annotations

import csv
import io
import os
import re
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Literal, Mapping

from fastapi import FastAPI, HTTPException, Query, status
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .parsers import ResultParseError, ResultParser, build_default_parsers
from .registry import (
    BACKEND_DIR,
    REPOSITORY_ROOT,
    ModelCatalog,
    ModelRegistry,
    ParameterValidationError,
    build_default_catalog,
    build_default_registry,
)
from .runs import (
    ModelNotFoundError,
    ModelUnavailableError,
    ResultUnavailableError,
    RunManager,
    RunNotCancellableError,
    RunNotFoundError,
    WorkerBusyError,
)


RunStatus = Literal["queued", "running", "succeeded", "failed", "cancelled"]
ExportFormat = Literal["json", "csv", "raw"]
DEFAULT_STATIC_DIR = BACKEND_DIR.parent / "static"
DEFAULT_RUN_ROOT = BACKEND_DIR / "data" / "runs"
SERIES_UNITS = {
    "time_s": "s",
    "north_m": "m",
    "east_m": "m",
    "up_m": "m",
    "speed_m_s": "m/s",
    "altitude_m": "m",
}


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ParameterInfo(StrictModel):
    id: str
    name: str
    label: str
    description: str
    type: Literal["number"]
    default: float
    minimum: float
    maximum: float
    unit: str


class ModuleOptionInfo(StrictModel):
    id: str
    name: str
    description: str
    parameters: list[ParameterInfo]


class ModuleSlotInfo(StrictModel):
    id: str
    name: str
    description: str
    default: str
    options: list[ModuleOptionInfo]


class ModelInfo(StrictModel):
    id: str
    name: str
    description: str
    available: bool
    parameters: list[ParameterInfo]
    modules: list[ModuleSlotInfo]


class ModelListResponse(StrictModel):
    models: list[ModelInfo]


class CatalogModelInfo(StrictModel):
    id: str
    name: str
    description: str
    kind: Literal["vehicle_model", "source_component"]
    version: str | None
    capabilities: list[str]
    contracts: list[str]
    evidence: list[str]
    source_path: str
    status: Literal["runnable", "build_required", "not_integrated"]
    status_label: str
    run_model_id: str | None
    source_present: bool
    runnable: bool


class ModelCategoryInfo(StrictModel):
    id: str
    name: str
    description: str
    models: list[CatalogModelInfo]


class ModelCatalogResponse(StrictModel):
    categories: list[ModelCategoryInfo]


class ModuleSelectionRequest(StrictModel):
    id: str = Field(min_length=1, max_length=64)
    parameters: dict[str, object] = Field(default_factory=dict)


class RunCreateRequest(StrictModel):
    model_id: str = Field(min_length=1, max_length=64)
    name: str | None = Field(default=None, min_length=1, max_length=100)
    parameters: dict[str, object] = Field(default_factory=dict)
    modules: dict[str, ModuleSelectionRequest] = Field(default_factory=dict)


class RunCreateResponse(StrictModel):
    run_id: str
    status: RunStatus


class ResultContractInfo(StrictModel):
    parser_id: str
    schema_version: int


class NormalizedModuleSelection(StrictModel):
    id: str
    name: str
    parameters: dict[str, float]


class RunSummaryResponse(StrictModel):
    run_id: str
    model_id: str
    model_name: str
    name: str
    parameters: dict[str, float]
    modules: dict[str, NormalizedModuleSelection]
    status: RunStatus
    created_at: datetime
    started_at: datetime | None
    finished_at: datetime | None
    return_code: int | None
    error: str | None
    summary: dict[str, int | float] | None
    result_available: bool
    result_contract: ResultContractInfo


class RunDetailResponse(RunSummaryResponse):
    logs: list[str]


class RunListResponse(StrictModel):
    runs: list[RunSummaryResponse]


class ResultSeries(StrictModel):
    time_s: list[float]
    north_m: list[float]
    east_m: list[float]
    up_m: list[float]
    speed_m_s: list[float]
    altitude_m: list[float]


class RunResultResponse(StrictModel):
    run_id: str
    model_id: str
    summary: dict[str, int | float]
    series: ResultSeries


class ComparisonRequest(StrictModel):
    run_ids: list[str] = Field(min_length=2, max_length=4)
    max_points: int = Field(default=2000, ge=100, le=5000)

    @model_validator(mode="after")
    def unique_run_ids(self) -> "ComparisonRequest":
        if len(set(self.run_ids)) != len(self.run_ids):
            raise ValueError("run_ids must be unique")
        return self


class ComparisonRun(StrictModel):
    run_id: str
    model_id: str
    model_name: str
    name: str
    created_at: datetime
    parameters: dict[str, float]
    modules: dict[str, NormalizedModuleSelection]
    result_contract: ResultContractInfo
    summary: dict[str, int | float]
    series: ResultSeries


class ComparisonResponse(StrictModel):
    units: dict[str, str]
    runs: list[ComparisonRun]


def _configured_timeout() -> float:
    raw = os.environ.get("MODELDEV_RUN_TIMEOUT_SECONDS", "300")
    try:
        value = float(raw)
    except ValueError:
        return 300.0
    return min(max(value, 1.0), 3600.0)


def _configured_run_root() -> Path:
    configured = os.environ.get("MODELDEV_RUN_ROOT")
    return Path(configured).expanduser() if configured else DEFAULT_RUN_ROOT


def _download_stem(record: Mapping[str, object]) -> str:
    model = re.sub(r"[^A-Za-z0-9_-]+", "-", str(record["model_id"]))[:32]
    run_id = re.sub(r"[^0-9a-f]+", "", str(record["run_id"]).lower())[:8]
    return f"{model or 'model'}-{run_id or 'run'}"


def create_app(
    *,
    registry: ModelRegistry | None = None,
    catalog: ModelCatalog | None = None,
    parsers: Mapping[str, ResultParser] | None = None,
    run_root: Path | None = None,
    static_dir: Path | None = None,
    run_timeout_seconds: float | None = None,
    result_max_points: int = 5000,
) -> FastAPI:
    model_registry = registry or build_default_registry()
    model_catalog = catalog or build_default_catalog(REPOSITORY_ROOT)
    result_parsers = parsers or build_default_parsers()
    manager = RunManager(
        registry=model_registry,
        parsers=result_parsers,
        run_root=run_root or _configured_run_root(),
        run_timeout_seconds=(
            run_timeout_seconds
            if run_timeout_seconds is not None
            else _configured_timeout()
        ),
        result_max_points=result_max_points,
    )

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        yield
        manager.shutdown()

    application = FastAPI(
        title="ModelDev Web API",
        version="0.5.0",
        lifespan=lifespan,
    )
    application.state.run_manager = manager
    application.add_middleware(GZipMiddleware, minimum_size=1000)

    @application.get("/api/models", response_model=ModelListResponse)
    def get_models() -> ModelListResponse:
        return ModelListResponse(
            models=[
                ModelInfo(
                    id=model.id,
                    name=model.name,
                    description=model.description,
                    available=model.resolve_executable() is not None,
                    parameters=[
                        ParameterInfo(
                            id=parameter.id,
                            name=parameter.label,
                            label=parameter.label,
                            description=parameter.description,
                            type=parameter.type,
                            default=parameter.default,
                            minimum=parameter.minimum,
                            maximum=parameter.maximum,
                            unit=parameter.unit,
                        )
                        for parameter in model.parameters
                    ],
                    modules=[
                        ModuleSlotInfo(
                            id=module.id,
                            name=module.name,
                            description=module.description,
                            default=module.default,
                            options=[
                                ModuleOptionInfo(
                                    id=option.id,
                                    name=option.name,
                                    description=option.description,
                                    parameters=[
                                        ParameterInfo(
                                            id=parameter.id,
                                            name=parameter.label,
                                            label=parameter.label,
                                            description=parameter.description,
                                            type=parameter.type,
                                            default=parameter.default,
                                            minimum=parameter.minimum,
                                            maximum=parameter.maximum,
                                            unit=parameter.unit,
                                        )
                                        for parameter in option.parameters
                                    ],
                                )
                                for option in module.options
                            ],
                        )
                        for module in model.modules
                    ],
                )
                for model in manager.list_models()
            ]
        )

    @application.get("/api/model-catalog", response_model=ModelCatalogResponse)
    def get_model_catalog() -> ModelCatalogResponse:
        categories: list[ModelCategoryInfo] = []
        for category in model_catalog.list_categories():
            catalog_models: list[CatalogModelInfo] = []
            for catalog_model in category.models:
                run_model = (
                    model_registry.get(catalog_model.run_model_id)
                    if catalog_model.run_model_id is not None
                    else None
                )
                if catalog_model.run_model_id is None:
                    model_status = "not_integrated"
                    status_label = (
                        "随整机运行"
                        if catalog_model.kind == "source_component"
                        else "未接入"
                    )
                elif run_model is None:
                    model_status = "not_integrated"
                    status_label = "运行映射无效"
                elif run_model.resolve_executable() is not None:
                    model_status = "runnable"
                    status_label = "可运行"
                else:
                    model_status = "build_required"
                    status_label = "待构建"
                catalog_models.append(
                    CatalogModelInfo(
                        id=catalog_model.id,
                        name=catalog_model.name,
                        description=catalog_model.description,
                        kind=catalog_model.kind,
                        version=catalog_model.version,
                        capabilities=list(catalog_model.capabilities),
                        contracts=list(catalog_model.contracts),
                        evidence=[
                            path.as_posix()
                            for path in catalog_model.source_relative_paths
                        ],
                        source_path=catalog_model.source_relative_paths[0].as_posix(),
                        status=model_status,
                        status_label=status_label,
                        run_model_id=catalog_model.run_model_id,
                        source_present=model_catalog.source_present(catalog_model),
                        runnable=model_status == "runnable",
                    )
                )
            categories.append(
                ModelCategoryInfo(
                    id=category.id,
                    name=category.name,
                    description=category.description,
                    models=catalog_models,
                )
            )
        return ModelCatalogResponse(categories=categories)

    @application.post(
        "/api/runs",
        response_model=RunCreateResponse,
        status_code=status.HTTP_202_ACCEPTED,
    )
    def create_run(payload: RunCreateRequest) -> RunCreateResponse:
        try:
            record = manager.start(
                payload.model_id,
                name=payload.name,
                parameters=payload.parameters,
                modules={
                    slot_id: selection.model_dump()
                    for slot_id, selection in payload.modules.items()
                },
            )
        except ParameterValidationError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        except ModelNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ModelUnavailableError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except WorkerBusyError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return RunCreateResponse(
            run_id=str(record["run_id"]), status=str(record["status"])
        )

    @application.get("/api/runs", response_model=RunListResponse)
    def list_runs(
        limit: int = Query(default=50, ge=1, le=100),
        model_id: str | None = Query(default=None, min_length=1, max_length=64),
        run_status: RunStatus | None = Query(default=None, alias="status"),
    ) -> RunListResponse:
        return RunListResponse(
            runs=[
                RunSummaryResponse.model_validate(record)
                for record in manager.list_runs(
                    limit=limit, model_id=model_id, status=run_status
                )
            ]
        )

    @application.get("/api/runs/{run_id}", response_model=RunDetailResponse)
    def get_run(run_id: str) -> RunDetailResponse:
        try:
            record = manager.get(run_id)
        except RunNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return RunDetailResponse.model_validate(record)

    @application.post(
        "/api/runs/{run_id}/cancel", response_model=RunCreateResponse
    )
    def cancel_run(run_id: str) -> RunCreateResponse:
        try:
            record = manager.cancel(run_id)
        except RunNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RunNotCancellableError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return RunCreateResponse(
            run_id=str(record["run_id"]), status=str(record["status"])
        )

    def build_result(run_id: str, max_points: int | None = None) -> RunResultResponse:
        record = manager.get(run_id)
        result = manager.parse_result(run_id, max_points=max_points)
        return RunResultResponse(
            run_id=run_id,
            model_id=str(record["model_id"]),
            summary=result.summary,
            series=ResultSeries.model_validate(result.series),
        )

    @application.get(
        "/api/runs/{run_id}/result", response_model=RunResultResponse
    )
    def get_result(run_id: str) -> RunResultResponse:
        try:
            return build_result(run_id)
        except RunNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ResultUnavailableError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ResultParseError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @application.get("/api/runs/{run_id}/export")
    def export_result(
        run_id: str,
        export_format: ExportFormat = Query(default="json", alias="format"),
    ) -> Response:
        try:
            record = manager.get(run_id)
            stem = _download_stem(record)
            if export_format == "raw":
                path = manager.result_path(run_id)
                return FileResponse(
                    path,
                    media_type="application/octet-stream",
                    filename=f"{stem}-result.dat",
                )

            result = build_result(run_id)
            if export_format == "json":
                return JSONResponse(
                    content=result.model_dump(mode="json"),
                    headers={
                        "Content-Disposition": f'attachment; filename="{stem}-result.json"'
                    },
                )

            stream = io.StringIO(newline="")
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(SERIES_UNITS)
            series = result.series
            writer.writerows(
                zip(
                    series.time_s,
                    series.north_m,
                    series.east_m,
                    series.up_m,
                    series.speed_m_s,
                    series.altitude_m,
                )
            )
            return Response(
                content=stream.getvalue(),
                media_type="text/csv",
                headers={
                    "Content-Disposition": f'attachment; filename="{stem}-trajectory.csv"',
                    "X-ModelDev-Max-Points": str(manager.result_max_points),
                },
            )
        except RunNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ResultUnavailableError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ResultParseError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @application.post("/api/comparisons", response_model=ComparisonResponse)
    def compare_runs(payload: ComparisonRequest) -> ComparisonResponse:
        runs: list[ComparisonRun] = []
        try:
            point_limit = min(payload.max_points, manager.result_max_points)
            comparison_model_id: str | None = None
            for run_id in payload.run_ids:
                record = manager.get(run_id)
                record_model_id = str(record["model_id"])
                if comparison_model_id is None:
                    comparison_model_id = record_model_id
                elif record_model_id != comparison_model_id:
                    raise HTTPException(
                        status_code=422,
                        detail="comparison runs must use the same model",
                    )
                result = manager.parse_result(run_id, max_points=point_limit)
                runs.append(
                    ComparisonRun(
                        run_id=run_id,
                        model_id=str(record["model_id"]),
                        model_name=str(record["model_name"]),
                        name=str(record["name"]),
                        created_at=record["created_at"],
                        parameters=record["parameters"],
                        modules=record["modules"],
                        result_contract=ResultContractInfo.model_validate(
                            record["result_contract"]
                        ),
                        summary=result.summary,
                        series=ResultSeries.model_validate(result.series),
                    )
                )
        except RunNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ResultUnavailableError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ResultParseError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return ComparisonResponse(units=SERIES_UNITS, runs=runs)

    # API routes are registered before this catch-all static mount.
    web_root = static_dir if static_dir is not None else DEFAULT_STATIC_DIR
    application.mount(
        "/",
        StaticFiles(directory=web_root, html=True, check_dir=False),
        name="static",
    )
    return application


app = create_app()
