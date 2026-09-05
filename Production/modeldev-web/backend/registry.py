"""Trusted, server-side model registry.

The browser may select a model id, but it never supplies an executable path or
command-line argument.  Every command in this module is assembled from static
server configuration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Literal, Mapping


BACKEND_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = BACKEND_DIR.parents[2]


ParameterType = Literal["number"]
ParameterValue = bool | int | float | str
ModuleSelection = dict[str, object]
CatalogKind = Literal["vehicle_model", "source_component"]


class ParameterValidationError(ValueError):
    """Raised when browser-supplied scenario parameters violate a model schema."""


@dataclass(frozen=True, slots=True)
class ParameterSpec:
    """One numeric parameter whose CLI flag is fixed by the server."""

    id: str
    label: str
    description: str
    default: float
    minimum: float
    maximum: float
    unit: str
    cli_flag: str
    type: ParameterType = "number"

    def validate(self, value: object) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ParameterValidationError(f"parameter {self.id} must be a number")
        normalized = float(value)
        if not math.isfinite(normalized):
            raise ParameterValidationError(f"parameter {self.id} must be finite")
        if normalized < self.minimum or normalized > self.maximum:
            raise ParameterValidationError(
                f"parameter {self.id} must be between {self.minimum:g} and "
                f"{self.maximum:g}"
            )
        return normalized

    def cli_arguments(self, value: float) -> tuple[str, str]:
        return self.cli_flag, format(value, ".17g")


@dataclass(frozen=True, slots=True)
class ModuleOptionSpec:
    """One trusted implementation available in a control or guidance slot."""

    id: str
    name: str
    description: str
    cli_value: str
    parameters: tuple[ParameterSpec, ...] = ()

    def __post_init__(self) -> None:
        parameter_ids = [parameter.id for parameter in self.parameters]
        if len(set(parameter_ids)) != len(parameter_ids):
            raise ValueError(f"module option {self.id} parameter ids must be unique")

    def validate_parameters(
        self, supplied: Mapping[str, object] | None = None
    ) -> dict[str, float]:
        supplied = supplied or {}
        specs = {parameter.id: parameter for parameter in self.parameters}
        unknown = sorted(set(supplied) - set(specs))
        if unknown:
            raise ParameterValidationError(
                f"module option {self.id} has unknown parameter(s): "
                + ", ".join(unknown)
            )
        return {
            parameter.id: parameter.validate(
                supplied.get(parameter.id, parameter.default)
            )
            for parameter in self.parameters
        }


@dataclass(frozen=True, slots=True)
class ModuleSlotSpec:
    """One first-class selectable module slot whose CLI flag is server-owned."""

    id: str
    name: str
    description: str
    cli_flag: str
    default: str
    options: tuple[ModuleOptionSpec, ...]

    def __post_init__(self) -> None:
        option_ids = [option.id for option in self.options]
        if not self.options:
            raise ValueError(f"module slot {self.id} must expose at least one option")
        if len(set(option_ids)) != len(option_ids):
            raise ValueError(f"module slot {self.id} option ids must be unique")
        if self.default not in option_ids:
            raise ValueError(f"module slot {self.id} default option is not registered")
        if not self.cli_flag.startswith("--"):
            raise ValueError(f"module slot {self.id} CLI flag must start with --")

    def get_option(self, option_id: str) -> ModuleOptionSpec | None:
        return next((option for option in self.options if option.id == option_id), None)

    def normalize(self, supplied: Mapping[str, object] | None = None) -> ModuleSelection:
        if supplied is None:
            option_id = self.default
            raw_parameters: Mapping[str, object] = {}
        else:
            unknown_fields = sorted(set(supplied) - {"id", "parameters"})
            if unknown_fields:
                raise ParameterValidationError(
                    f"module slot {self.id} has unknown field(s): "
                    + ", ".join(unknown_fields)
                )
            option_id = supplied.get("id")
            if not isinstance(option_id, str) or not option_id:
                raise ParameterValidationError(
                    f"module slot {self.id} must select a non-empty option id"
                )
            raw_parameters_value = supplied.get("parameters", {})
            if not isinstance(raw_parameters_value, Mapping):
                raise ParameterValidationError(
                    f"module slot {self.id} parameters must be an object"
                )
            raw_parameters = raw_parameters_value

        option = self.get_option(option_id)
        if option is None:
            raise ParameterValidationError(
                f"unknown option for module slot {self.id}: {option_id}"
            )
        return {
            "id": option.id,
            "name": option.name,
            "parameters": option.validate_parameters(raw_parameters),
        }

    def cli_arguments(self, selection: Mapping[str, object]) -> tuple[str, ...]:
        option_id = selection.get("id")
        if not isinstance(option_id, str):
            raise ParameterValidationError(
                f"module slot {self.id} selection is not normalized"
            )
        option = self.get_option(option_id)
        if option is None:
            raise ParameterValidationError(
                f"unknown option for module slot {self.id}: {option_id}"
            )
        raw_parameters = selection.get("parameters")
        if not isinstance(raw_parameters, Mapping):
            raise ParameterValidationError(
                f"module slot {self.id} parameters are not normalized"
            )
        parameters = option.validate_parameters(raw_parameters)
        arguments: list[str] = [self.cli_flag, option.cli_value]
        for parameter in option.parameters:
            arguments.extend(parameter.cli_arguments(parameters[parameter.id]))
        return tuple(arguments)


@dataclass(frozen=True, slots=True)
class ModelSpec:
    """One trusted model executable and its result contract."""

    id: str
    name: str
    description: str
    executable_candidates: tuple[Path, ...]
    result_relative_path: Path
    parser_id: str
    result_schema_version: int = 1
    parameters: tuple[ParameterSpec, ...] = ()
    modules: tuple[ModuleSlotSpec, ...] = ()
    command_suffix: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        parameter_ids = [parameter.id for parameter in self.parameters]
        module_ids = [module.id for module in self.modules]
        if len(set(parameter_ids)) != len(parameter_ids):
            raise ValueError(f"model {self.id} parameter ids must be unique")
        if len(set(module_ids)) != len(module_ids):
            raise ValueError(f"model {self.id} module slot ids must be unique")

    def resolve_executable(self) -> Path | None:
        """Return the first existing trusted executable, if any."""

        for candidate in self.executable_candidates:
            resolved = candidate.resolve()
            if resolved.is_file():
                return resolved
        return None

    def validate_parameters(
        self, supplied: Mapping[str, object] | None = None
    ) -> dict[str, float]:
        supplied = supplied or {}
        specs = {parameter.id: parameter for parameter in self.parameters}
        unknown = sorted(set(supplied) - set(specs))
        if unknown:
            raise ParameterValidationError(
                "unknown parameter(s): " + ", ".join(unknown)
            )
        normalized = {
            parameter.id: parameter.validate(
                supplied.get(parameter.id, parameter.default)
            )
            for parameter in self.parameters
        }
        if {
            "launch_lon_deg",
            "launch_lat_deg",
            "target_lon_deg",
            "target_lat_deg",
        }.issubset(normalized):
            launch_lat = math.radians(normalized["launch_lat_deg"])
            target_lat = math.radians(normalized["target_lat_deg"])
            delta_lat = target_lat - launch_lat
            delta_lon = math.radians(
                math.remainder(
                    normalized["target_lon_deg"] - normalized["launch_lon_deg"],
                    360.0,
                )
            )
            # Local tangent-plane distance is accurate enough for this <1 m guard.
            horizontal_distance = 6371000.0 * math.hypot(
                delta_lat, delta_lon * math.cos((launch_lat + target_lat) * 0.5)
            )
            if horizontal_distance < 1.0:
                raise ParameterValidationError(
                    "launch and target horizontal separation must be at least 1 m"
                )
        return normalized

    def validate_modules(
        self, supplied: Mapping[str, object] | None = None
    ) -> dict[str, ModuleSelection]:
        supplied = supplied or {}
        slots = {module.id: module for module in self.modules}
        unknown = sorted(set(supplied) - set(slots))
        if unknown:
            raise ParameterValidationError(
                "unknown module slot(s): " + ", ".join(unknown)
            )

        normalized: dict[str, ModuleSelection] = {}
        for module in self.modules:
            raw_selection = supplied.get(module.id)
            if raw_selection is not None and not isinstance(raw_selection, Mapping):
                raise ParameterValidationError(
                    f"module slot {module.id} selection must be an object"
                )
            normalized[module.id] = module.normalize(raw_selection)
        return normalized

    def prepare_run(
        self,
        supplied_parameters: Mapping[str, object] | None = None,
        supplied_modules: Mapping[str, object] | None = None,
    ) -> tuple[dict[str, float], dict[str, ModuleSelection], tuple[str, ...] | None]:
        """Validate browser configuration and build one fully trusted argv tuple."""

        parameters = self.validate_parameters(supplied_parameters)
        modules = self.validate_modules(supplied_modules)
        executable = self.resolve_executable()
        if executable is None:
            return parameters, modules, None

        arguments: list[str] = []
        for parameter in self.parameters:
            arguments.extend(parameter.cli_arguments(parameters[parameter.id]))
        for module in self.modules:
            arguments.extend(module.cli_arguments(modules[module.id]))
        command = (str(executable), *self.command_suffix, *arguments)
        return parameters, modules, command

    def command(
        self,
        supplied: Mapping[str, object] | None = None,
        modules: Mapping[str, object] | None = None,
    ) -> tuple[str, ...] | None:
        _, _, command = self.prepare_run(supplied, modules)
        return command


class ModelRegistry:
    """Immutable lookup table for models exposed by the API."""

    def __init__(self, models: Iterable[ModelSpec]):
        entries = list(models)
        self._models = {entry.id: entry for entry in entries}
        if len(self._models) != len(entries):
            raise ValueError("model ids must be unique")

    def list(self) -> tuple[ModelSpec, ...]:
        return tuple(self._models.values())

    def get(self, model_id: str) -> ModelSpec | None:
        return self._models.get(model_id)


@dataclass(frozen=True, slots=True)
class CatalogModelSpec:
    """One repository model shown in the library, runnable or not yet integrated."""

    id: str
    name: str
    description: str
    kind: CatalogKind
    version: str | None
    capabilities: tuple[str, ...]
    contracts: tuple[str, ...]
    source_relative_paths: tuple[Path, ...]
    run_model_id: str | None = None

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("catalog model id must not be empty")
        if not self.source_relative_paths:
            raise ValueError(f"catalog model {self.id} must declare source paths")
        if not self.capabilities:
            raise ValueError(f"catalog model {self.id} must declare capabilities")
        if not self.contracts:
            raise ValueError(f"catalog model {self.id} must declare contracts")
        for relative_path in self.source_relative_paths:
            if relative_path.is_absolute() or ".." in relative_path.parts:
                raise ValueError(
                    f"catalog model {self.id} source paths must be repository-relative"
                )


@dataclass(frozen=True, slots=True)
class ModelCategorySpec:
    """A stable display category in the repository model library."""

    id: str
    name: str
    description: str
    models: tuple[CatalogModelSpec, ...]

    def __post_init__(self) -> None:
        if not self.models:
            raise ValueError(f"model category {self.id} must contain models")
        model_ids = [model.id for model in self.models]
        if len(set(model_ids)) != len(model_ids):
            raise ValueError(f"model category {self.id} model ids must be unique")


class ModelCatalog:
    """Trusted model-library metadata independent of the runnable registry."""

    def __init__(
        self, repository_root: Path, categories: Iterable[ModelCategorySpec]
    ) -> None:
        root = repository_root.resolve()
        entries = tuple(categories)
        category_ids = [category.id for category in entries]
        if len(set(category_ids)) != len(category_ids):
            raise ValueError("model category ids must be unique")

        model_ids = [model.id for category in entries for model in category.models]
        if len(set(model_ids)) != len(model_ids):
            raise ValueError("catalog model ids must be unique across categories")

        self._repository_root = root
        self._categories = entries

    def list_categories(self) -> tuple[ModelCategorySpec, ...]:
        return self._categories

    def source_present(self, model: CatalogModelSpec) -> bool:
        for relative_path in model.source_relative_paths:
            try:
                if not _trusted_repo_path(
                    self._repository_root, str(relative_path)
                ).exists():
                    return False
            except (OSError, ValueError):
                # A missing/denied path or a symlink escaping the repository is
                # catalog evidence that is unavailable, not a reason to fail
                # the entire public catalog response.
                return False
        return True


def _trusted_repo_path(repository_root: Path, relative_path: str) -> Path:
    """Resolve a hard-coded repository-relative path without allowing escape."""

    root = repository_root.resolve()
    candidate = (root / relative_path).resolve()
    candidate.relative_to(root)
    return candidate


def build_default_catalog(repository_root: Path = REPOSITORY_ROOT) -> ModelCatalog:
    """Describe repository model sources without claiming Web-run integration."""

    def vehicle_model(
        model_id: str,
        name: str,
        description: str,
        *,
        test_directory: str | None = None,
        run_model_id: str | None = None,
    ) -> CatalogModelSpec:
        test_path = Path("Test") / (test_directory or model_id) / "main.cpp"
        if run_model_id is not None:
            contracts = (
                f"POST /api/runs（model_id={run_model_id}）",
                "GET /api/runs/{run_id}/result",
            )
            capabilities = ("C++ 六自由度仿真", "Web 场景运行与结果解析")
        else:
            contracts = ("C++ 源码与测试入口；尚未接入 Web 参数、执行和结果解析契约",)
            capabilities = ("C++ 飞行器模型源码", "仓库测试入口（未作 Web 运行承诺）")
        return CatalogModelSpec(
            id=model_id,
            name=name,
            description=description,
            kind="vehicle_model",
            version=None,
            capabilities=capabilities,
            contracts=contracts,
            source_relative_paths=(
                Path("include") / model_id,
                Path("src") / model_id,
                test_path,
            ),
            run_model_id=run_model_id,
        )

    def source_component(
        component_id: str,
        name: str,
        description: str,
        capabilities: tuple[str, ...],
        *source_paths: str,
    ) -> CatalogModelSpec:
        return CatalogModelSpec(
            id=component_id,
            name=name,
            description=description,
            kind="source_component",
            version=None,
            capabilities=capabilities,
            contracts=("C++ 源码组件；由整机模型调用，不支持独立 Web 仿真",),
            source_relative_paths=tuple(Path(path) for path in source_paths),
        )

    return ModelCatalog(
        repository_root,
        (
            ModelCategorySpec(
                id="vehicle",
                name="飞行器模型",
                description="整机飞行器源码模型；仅 HTV2 与 BGM 已接入当前 Web 运行链路",
                models=(
                    vehicle_model(
                        "HTV2",
                        "HTV-2",
                        "HTV-2 助推滑翔飞行器六自由度模型",
                        run_model_id="HTV2",
                    ),
                    vehicle_model(
                        "BGM",
                        "BGM",
                        "BGM 巡航弹六自由度模型",
                        run_model_id="BGM",
                    ),
                    vehicle_model("AGM86C", "AGM86C", "AGM86C 巡航弹源码模型"),
                    vehicle_model(
                        "HACM", "HACM", "HACM 高超声速巡航弹源码模型"
                    ),
                    vehicle_model("PAC2", "PAC2", "PAC2 近程防空源码模型"),
                    vehicle_model(
                        "PAC500", "PAC500", "PAC500 中程拦截源码模型"
                    ),
                    vehicle_model("SM6", "SM6", "SM6 防空拦截源码模型"),
                    vehicle_model("AIM9", "AIM-9", "AIM-9 近程空空导弹源码模型"),
                    vehicle_model(
                        "AIM120D", "AIM-120D", "AIM-120D 远程空空导弹源码模型"
                    ),
                    vehicle_model("LRHW", "LRHW", "LRHW 滑翔飞行器源码模型"),
                    vehicle_model("R11", "R11", "R11 弹道弹源码模型"),
                    vehicle_model(
                        "Aircraft", "通用飞机", "通用飞机运动源码模型"
                    ),
                    vehicle_model("Su27", "Su-27", "Su-27 飞机运动源码模型"),
                    vehicle_model(
                        "OrbitModel", "轨道模型", "轨道动力学源码模型"
                    ),
                    vehicle_model("GPI", "GPI", "GPI 源码模型"),
                    vehicle_model(
                        "HXD3530",
                        "HXD3530",
                        "HXD3530 源码模型",
                        test_directory="HXD",
                    ),
                    vehicle_model(
                        "TheoreticalModel", "理论模型", "通用理论源码模型"
                    ),
                ),
            ),
            ModelCategorySpec(
                id="environment",
                name="环境与动力学",
                description="大气与六自由度动力学公共源码组件",
                models=(
                    source_component(
                        "atmosphere",
                        "NRLMSISE-00 大气",
                        "提供密度与温度计算的源码组件，随整机模型调用",
                        ("大气密度计算", "大气温度计算"),
                        "include/Util/Atmosphere.h",
                        "src/Util/Atmosphere.cpp",
                    ),
                    source_component(
                        "six_dof_dynamics",
                        "六自由度动力学",
                        "提供 ECF 加速度与本体系力矩计算的源码组件，随整机模型调用",
                        ("ECF 加速度计算", "本体系力矩计算"),
                        "include/Util/Dynamics.h",
                        "src/Util/Dynamics.cpp",
                    ),
                ),
            ),
            ModelCategorySpec(
                id="guidance",
                name="制导模块",
                description="已在 HTV2/BGM 整机内部调用的制导源码组件",
                models=(
                    source_component(
                        "htv2_guidance",
                        "HTV-2 分阶段制导",
                        "HTV-2 整机内部的分阶段、滑翔和末段制导源码组件",
                        ("助推/滑翔/俯冲分阶段制导", "末段比例导航"),
                        "include/HTV2/Guidance.h",
                        "src/HTV2/Guidance.cpp",
                    ),
                    source_component(
                        "bgm_guidance",
                        "BGM L1/PN 制导",
                        "BGM 整机内部的 L1 航路与比例导航源码组件",
                        ("L1 航路制导", "比例导航"),
                        "include/BGM/Guidance.h",
                        "src/BGM/Guidance.cpp",
                    ),
                ),
            ),
            ModelCategorySpec(
                id="control",
                name="控制模块",
                description="已在 HTV2/BGM 整机内部调用的六自由度控制源码组件",
                models=(
                    source_component(
                        "htv2_control",
                        "HTV-2 六自由度控制",
                        "HTV-2 整机内部的 P/PI 六自由度控制源码组件",
                        ("P 控制", "PI 控制", "舵偏限幅"),
                        "include/HTV2/Control.h",
                        "src/HTV2/Control.cpp",
                    ),
                    source_component(
                        "bgm_control",
                        "BGM 六自由度控制",
                        "BGM 整机内部的 P/PI 六自由度控制源码组件",
                        ("P 控制", "PI 控制", "舵偏限幅"),
                        "include/BGM/Control.h",
                        "src/BGM/Control.cpp",
                    ),
                ),
            ),
            ModelCategorySpec(
                id="sensor",
                name="传感器模块",
                description="导引头与惯性测量源码组件",
                models=(
                    source_component(
                        "common_seeker",
                        "公共导引头",
                        "根据目标与自身状态计算视线信息的源码组件",
                        ("目标视线信息计算",),
                        "include/Seeker/Seeker.h",
                        "src/Seeker/Seeker.cpp",
                    ),
                    source_component(
                        "htv2_imu",
                        "HTV-2 IMU",
                        "HTV-2 整机内部的惯性测量源码组件",
                        ("本体系 IMU 信息计算",),
                        "include/HTV2/IMU.h",
                        "src/HTV2/IMU.cpp",
                    ),
                    source_component(
                        "bgm_imu",
                        "BGM IMU",
                        "BGM 整机内部的惯性测量源码组件",
                        ("本体系 IMU 信息计算",),
                        "include/BGM/IMU.h",
                        "src/BGM/IMU.cpp",
                    ),
                ),
            ),
            ModelCategorySpec(
                id="recorder",
                name="记录器模块",
                description="随整机仿真写出轨迹与气动数据的源码组件",
                models=(
                    source_component(
                        "htv2_file_saver",
                        "HTV-2 数据记录器",
                        "HTV-2 整机内部的轨迹与气动数据文件记录组件",
                        ("轨迹数据写出", "气动数据写出"),
                        "include/HTV2/FileSaver.h",
                        "src/HTV2/FileSaver.cpp",
                    ),
                    source_component(
                        "bgm_file_saver",
                        "BGM 数据记录器",
                        "BGM 整机内部的轨迹与气动数据文件记录组件",
                        ("轨迹数据写出", "气动数据写出"),
                        "include/BGM/FileSaver.h",
                        "src/BGM/FileSaver.cpp",
                    ),
                ),
            ),
        ),
    )


def build_default_registry(repository_root: Path = REPOSITORY_ROOT) -> ModelRegistry:
    """Build the first-release static registry for HTV2 and BGM."""

    def candidates(target: str) -> tuple[Path, ...]:
        return (
            _trusted_repo_path(repository_root, f"_Build/out/windows/Release/{target}.exe"),
            _trusted_repo_path(repository_root, f"_Build/out/windows/Debug/{target}.exe"),
        )

    def number(
        parameter_id: str,
        label: str,
        description: str,
        default: float,
        minimum: float,
        maximum: float,
        unit: str,
        cli_flag: str | None = None,
    ) -> ParameterSpec:
        return ParameterSpec(
            id=parameter_id,
            label=label,
            description=description,
            default=default,
            minimum=minimum,
            maximum=maximum,
            unit=unit,
            cli_flag=cli_flag or "--" + parameter_id.replace("_", "-"),
        )

    common = (
        number("launch_lon_deg", "发射经度", "发射点经度", 0, -180, 180, "deg"),
        number("launch_lat_deg", "发射纬度", "发射点纬度", 0, -89.9, 89.9, "deg"),
        number(
            "launch_altitude_m", "发射高度", "发射点海拔高度", 0, 0, 100000, "m"
        ),
        number("target_lon_deg", "目标经度", "目标点经度", 0, -180, 180, "deg"),
        number("target_lat_deg", "目标纬度", "目标点纬度", 0, -89.9, 89.9, "deg"),
        number(
            "target_altitude_m",
            "目标高度",
            "目标点海拔高度",
            0,
            -1000,
            100000,
            "m",
        ),
        number("launch_theta_deg", "发射倾角", "发射速度倾角", 45, 1, 89.9, "deg"),
        number(
            "max_sim_time_s", "最长仿真时间", "仿真时间上限", 1000, 1, 2000, "s"
        ),
    )

    def with_defaults(
        specs: tuple[ParameterSpec, ...], defaults: Mapping[str, float]
    ) -> tuple[ParameterSpec, ...]:
        return tuple(
            ParameterSpec(
                id=spec.id,
                label=spec.label,
                description=spec.description,
                default=defaults.get(spec.id, spec.default),
                minimum=spec.minimum,
                maximum=spec.maximum,
                unit=spec.unit,
                cli_flag=spec.cli_flag,
            )
            for spec in specs
        )

    control_parameters = (
        number(
            "gain_scale",
            "反馈增益倍率",
            "仅缩放控制器反馈项，不缩放前馈项",
            1,
            0.25,
            2,
            "1",
            "--control-gain-scale",
        ),
        number(
            "rudder_limit_deg",
            "舵偏限幅",
            "控制器舵偏绝对值上限",
            45,
            5,
            45,
            "deg",
            "--control-rudder-limit-deg",
        ),
    )
    control_slot = ModuleSlotSpec(
        id="control",
        name="控制模块",
        description="选择六自由度控制反馈结构",
        cli_flag="--control-module",
        default="p6dof_pi",
        options=(
            ModuleOptionSpec(
                id="p6dof_pi",
                name="六自由度 PI 控制",
                description="使用比例与积分反馈的六自由度控制器",
                cli_value="p6dof_pi",
                parameters=control_parameters,
            ),
            ModuleOptionSpec(
                id="p6dof_p",
                name="六自由度 P 控制",
                description="仅使用比例反馈的六自由度控制器",
                cli_value="p6dof_p",
                parameters=control_parameters,
            ),
        ),
    )

    htv2_navigation = number(
        "navigation_constant",
        "比例导航系数",
        "仅用于末段比例导航（PN）",
        4,
        1,
        8,
        "1",
        "--guidance-navigation-constant",
    )
    htv2_guidance_slot = ModuleSlotSpec(
        id="guidance",
        name="制导模块",
        description="选择 HTV2 分阶段制导实现",
        cli_flag="--guidance-module",
        default="phase_pull_bias",
        options=(
            ModuleOptionSpec(
                id="phase_pull_bias",
                name="分阶段拉偏制导",
                description="包含拉偏保持段，并在末段切换到比例导航",
                cli_value="phase_pull_bias",
                parameters=(
                    htv2_navigation,
                    number(
                        "hold_start_distance_m",
                        "保持段起始距离",
                        "进入拉偏保持段所使用的距离配置",
                        80000,
                        1000,
                        500000,
                        "m",
                        "--guidance-hold-start-distance-m",
                    ),
                    number(
                        "hold_duration_s",
                        "保持段时长",
                        "拉偏保持段持续时间",
                        15,
                        0.1,
                        120,
                        "s",
                        "--guidance-hold-duration-s",
                    ),
                    number(
                        "mount_az_deg",
                        "安装方位角",
                        "拉偏保持段使用的安装方位角",
                        20,
                        -180,
                        180,
                        "deg",
                        "--guidance-mount-az-deg",
                    ),
                    number(
                        "mount_el_deg",
                        "安装俯仰角",
                        "拉偏保持段使用的安装俯仰角",
                        -10,
                        -89,
                        89,
                        "deg",
                        "--guidance-mount-el-deg",
                    ),
                ),
            ),
            ModuleOptionSpec(
                id="phase_standard",
                name="标准分阶段制导",
                description="不使用拉偏保持配置，末段采用比例导航",
                cli_value="phase_standard",
                parameters=(htv2_navigation,),
            ),
        ),
    )

    bgm_navigation = number(
        "navigation_constant",
        "比例导航系数",
        "用于末段或回退比例导航（PN）",
        4,
        1,
        8,
        "1",
        "--guidance-navigation-constant",
    )
    bgm_guidance_slot = ModuleSlotSpec(
        id="guidance",
        name="制导模块",
        description="选择 BGM 分阶段制导实现",
        cli_flag="--guidance-module",
        default="phase_l1",
        options=(
            ModuleOptionSpec(
                id="phase_l1",
                name="分阶段 L1 航路制导",
                description="使用 L1 航路制导，末段或回退阶段采用比例导航",
                cli_value="phase_l1",
                parameters=(
                    bgm_navigation,
                    number(
                        "l1_lookahead_factor",
                        "L1 前视因子",
                        "L1 航路制导的前视距离因子",
                        5,
                        1,
                        20,
                        "1",
                        "--guidance-l1-lookahead-factor",
                    ),
                    number(
                        "first_waypoint_distance_m",
                        "首航点距目标",
                        "首航点相对目标的配置距离，不是实时切换触发距离",
                        20000,
                        1000,
                        100000,
                        "m",
                        "--guidance-first-waypoint-distance-m",
                    ),
                    number(
                        "pull_bias_angle_deg",
                        "拉偏角",
                        "L1 航路首航段使用的拉偏角配置",
                        10,
                        -60,
                        60,
                        "deg",
                        "--guidance-pull-bias-angle-deg",
                    ),
                ),
            ),
            ModuleOptionSpec(
                id="phase_pn",
                name="分阶段 PN 制导",
                description="使用分阶段比例导航，不使用 L1 航路",
                cli_value="phase_pn",
                parameters=(bgm_navigation,),
            ),
        ),
    )

    return ModelRegistry(
        (
            ModelSpec(
                id="HTV2",
                name="HTV2",
                description="HTV2 可配置发射点、目标点与仿真时长场景",
                executable_candidates=candidates("HTV2_Test"),
                result_relative_path=Path("Results/HTV2/result.dat"),
                parser_id="htv2",
                parameters=with_defaults(
                    common,
                    {
                        "launch_lon_deg": 120,
                        "launch_lat_deg": 40,
                        "launch_altitude_m": 10,
                        "target_lon_deg": 140,
                        "target_lat_deg": 20,
                        "target_altitude_m": 0,
                        "launch_theta_deg": 89,
                        "max_sim_time_s": 1000,
                    },
                ),
                modules=(htv2_guidance_slot, control_slot),
            ),
            ModelSpec(
                id="BGM",
                name="BGM",
                description="BGM 可配置场景，并支持巡航高度与马赫数指令",
                executable_candidates=candidates("BGMTest"),
                result_relative_path=Path("Results/BGM/result.dat"),
                parser_id="bgm",
                parameters=with_defaults(
                    common,
                    {
                        "launch_lon_deg": 121,
                        "launch_lat_deg": 40,
                        "launch_altitude_m": 2,
                        "target_lon_deg": 122,
                        "target_lat_deg": 40,
                        "target_altitude_m": 0,
                        "launch_theta_deg": 45,
                        "max_sim_time_s": 1500,
                    },
                )
                + (
                    number(
                        "cruise_altitude_m",
                        "巡航高度",
                        "BGM 巡航高度指令",
                        100,
                        10,
                        20000,
                        "m",
                    ),
                    number(
                        "cruise_mach",
                        "巡航马赫数",
                        "BGM 巡航速度指令",
                        0.7,
                        0.1,
                        5,
                        "Mach",
                    ),
                ),
                modules=(bgm_guidance_slot, control_slot),
            ),
        )
    )
