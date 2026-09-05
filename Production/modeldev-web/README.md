# ModelDev Web

这是一个面向个人本机使用的 Web 仿真闭环。浏览器选择白名单模型并填写场景参数；FastAPI 在独立目录中启动对应的 C++ 可执行程序，持久化运行记录，并将结果解析、降采样后返回页面。

当前接入：

- `HTV2`：`_Build/out/windows/Release/HTV2_Test.exe`
- `BGM`：`_Build/out/windows/Release/BGMTest.exe`

当前核心功能包括：

1. 真实模型库：按飞行器、环境与动力学、制导、控制、传感器、记录器六类展示仓库模型；仅把已接入且存在可执行文件的 HTV2/BGM 标记为可运行。
2. 动态场景参数：发射点、目标点、发射倾角、仿真时长；BGM 另支持巡航高度和巡航马赫数。
3. 可配置 GNC 模块：每个模型可从服务端白名单中选择制导与控制算法，并填写当前算法专属参数；选择和参数会进入 C++ 计算并随运行记录持久化。
4. 持久化历史：重启服务后仍可载入旧结果，并可导出 JSON、CSV 或原始 `result.dat`。
5. 结果比较：选择 2–4 条同模型成功运行，叠加比较高度、速度和摘要指标；允许比较同一模型的不同制导/控制组合。

页面还提供开始/取消、运行状态、日志、摘要、高度/速度曲线和东向—北向轨迹。

## 1. 构建 C++ 目标

请先在 Visual Studio Developer PowerShell 中，从项目根目录构建：

```powershell
cmake --build cmake-build-release --target HTV2_Test
cmake --build cmake-build-release --target BGMTest
```

服务优先使用 Release 程序；Release 不存在时才退回 Debug。

本次模块化修改扩展了 HTV2/BGM 的类布局。若复用修改前的构建目录，请至少执行一次干净重编译，避免旧对象文件与新头文件混用：

```powershell
cmake --build cmake-build-release --target HTV2_Test BGMTest --clean-first
```

## 2. 启动

在项目根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\Production\modeldev-web\start.ps1
```

第一次启动会在 `Production/modeldev-web/.venv` 创建虚拟环境并安装依赖。启动脚本刻意保持纯 ASCII，以兼容 Windows PowerShell 5.1 对无 BOM UTF-8 脚本的读取方式。若 Python 没有加入 `PATH`，可显式指定：

```powershell
powershell -ExecutionPolicy Bypass -File .\Production\modeldev-web\start.ps1 `
  -Python "C:\Users\17298\AppData\Local\Programs\Python\Python312\python.exe"
```

然后打开 <http://127.0.0.1:8000/>。

可使用 `-Port 8080` 修改端口。运行文件默认保存在 `backend/data/runs/<run_id>`；该目录和虚拟环境均不会提交到 Git。

## API

- `GET /api/models`：模型、可用状态、场景参数 schema，以及制导/控制模块白名单
- `POST /api/runs`：请求体同时支持场景参数和模块配置，例如：

  ```json
  {
    "model_id": "BGM",
    "name": "L1 与 P6DOF-P 短时巡航",
    "parameters": {
      "max_sim_time_s": 20,
      "cruise_mach": 0.8
    },
    "modules": {
      "guidance": {
        "id": "phase_l1",
        "parameters": {
          "navigation_constant": 4,
          "l1_lookahead_factor": 5,
          "first_waypoint_distance_m": 20000,
          "pull_bias_angle_deg": 10
        }
      },
      "control": {
        "id": "p6dof_p",
        "parameters": {
          "gain_scale": 1,
          "rudder_limit_deg": 45
        }
      }
    }
  }
  ```

- `GET /api/runs?limit=50&model_id=BGM&status=succeeded`：历史列表与筛选
- `GET /api/runs/{run_id}`：详情、参数和日志
- `POST /api/runs/{run_id}/cancel`
- `GET /api/runs/{run_id}/result`
- `GET /api/runs/{run_id}/export?format=json|csv|raw`
- `POST /api/comparisons`：请求体如 `{ "run_ids": ["...", "..."], "max_points": 2000 }`

省略 `modules` 时，服务端会使用该模型当前登记的默认制导和控制模块。HTV2 默认保持“分阶段制导 + 末段拉偏覆盖层”，BGM 默认保持“分阶段 + L1 航路制导”，控制默认保持 `P6DOF-PI`。其中，BGM 的“首航点距目标”用于生成 L1 航路点，并不是运行时按距离触发切换；与内部制导切换耦合的 15 秒时序不作为独立参数开放。

服务只允许一个仿真任务同时运行。每个 C++ 进程使用独立工作目录，命令采用 `shell=False`，浏览器不能提交可执行文件路径、命令行参数名或结果路径。服务端会拒绝未知模块、未知模块参数、非数值、非有限、越界及发射点与目标点水平距离小于 1 m 的输入。这里的模块是已经编译进 HTV2/BGM、并由服务端注册的算法变体，不是任意 DLL 或脚本热插拔接口。

## 测试

```powershell
.\Production\modeldev-web\.venv\Scripts\python.exe -m pytest `
  .\Production\modeldev-web\backend\tests -q
node --check .\Production\modeldev-web\static\app.js
```

## 当前限制

- 当前是个人本机、单服务实例、单 worker；同时只能运行一个仿真。
- 历史记录暂未提供页面删除和自动保留策略。
- 服务异常崩溃时还没有使用 Windows Job Object 强制清理遗留子进程。
- 页面比较限制为同模型 2–4 条运行；曲线保留各运行的原生时间轴，不做插值对齐。
- 结果展示和 JSON/CSV 导出最多返回 5000 个采样点；raw 导出保留完整原始 `result.dat`。
