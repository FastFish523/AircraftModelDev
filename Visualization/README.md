# HTV2 离线轨迹与姿态回放

该工具读取 `HTV2::FileSaver` 生成的 48 列 `result.dat`，离线显示 ENU 三维轨迹，并使用红、绿、蓝三根箭头表示机体 X、Y、Z 轴。它不会修改或启动仿真程序。

## 安装依赖

当前机器的 Python 3.12 未加入 `PATH`，请在 PowerShell 中使用绝对路径：

```powershell
& 'C:\Users\17298\AppData\Local\Programs\Python\Python312\python.exe' -m pip install -r .\Visualization\requirements.txt
```

## 启动

在项目根目录执行：

```powershell
& 'C:\Users\17298\AppData\Local\Programs\Python\Python312\python.exe' .\Visualization\htv2_replay.py
```

也可以启动时直接指定文件：

```powershell
& 'C:\Users\17298\AppData\Local\Programs\Python\Python312\python.exe' .\Visualization\htv2_replay.py .\_Build\out\windows\Debug\Results\HTV2\result.dat
```

只验证文件能否读取、不打开窗口：

```powershell
& 'C:\Users\17298\AppData\Local\Programs\Python\Python312\python.exe' .\Visualization\htv2_replay.py --check .\_Build\out\windows\Debug\Results\HTV2\result.dat
```

## 操作

1. 先独立运行 `HTV2_Test.exe` 生成 `result.dat`。
2. 点击“加载 result.dat”选择实际输出文件。
3. 使用播放、暂停、复位、倍速和底部时间滑块进行回放。
4. Matplotlib 工具栏可用于旋转、缩放和恢复三维视角。

轨迹坐标显示为 East–North–Up。姿态计算复现项目的 231 约定：`Cbn = Ry(yaw) * Rz(pitch) * Rx(roll)`。
## STL model and curved Earth surface

The replay automatically loads `assets/Fighter_jet_concept.stl`, normalizes its dimensions, and displays a reduced 6,000-triangle rendering mesh while preserving the original STL file. Model and body axes use the front/up/right order. A spherical Earth replaces the Cartesian grid and coordinate panes.

Model source: [NASA X-59 3D Printing - No Stand](https://www.nasa.gov/stem-content/x-59-3d-printing/). Author/origin: NASA Aeronautics Research Mission Directorate, Zachary Gwennap and Liam Brinton.
