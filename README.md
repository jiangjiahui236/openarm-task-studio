# OpenArm Task Studio

这是一个 OpenArm 双臂机器人的可视化任务编辑器。可以在 Isaac Sim 里摆放 Move、Grasp、Place 任务点并按顺序执行，也可以通过 D435 或普通摄像头进行上肢动作示教。

项目最初用来串联自己的 OpenArm 仿真实验，目前仍是一个实验工具，不是成熟产品。

## 项目状态

本项目使用 AI 辅助开发。AI 参与了代码编写、重构、测试和文档整理；功能设计、运行验证和最终取舍由作者完成。

项目仍有不少问题：环境兼容性有限，部分错误提示不够清楚，普通 RGB 摄像头的深度估计不如 D435 稳定，避障也不是真正的整臂运动规划。README 无法覆盖所有机器的情况。复现失败时可以提交 Issue，并附上终端报错、系统版本和复现步骤。

本项目适合仿真、研究和学习。避障采用 TCP 对场景 AABB 的采样，请勿直接用于无人值守的真实机器人生产环境。

## 它现在能做什么

- 可视化创建 Move / Grasp / Place 任务点
- 左右臂独立任务与并行执行
- 任务 JSON 保存、加载和路径验证
- D435 RGB-D 与普通摄像头 RGB 动作捕捉切换
- MediaPipe 肩、肘、腕姿态跟随与中立姿态标定
- 动作捕捉监测日志与可选 ESP32-C3 记录按钮
- 参数调节 GUI 和纯 Python 单元测试

## 环境要求

- Ubuntu 22.04（当前仅验证 Linux）
- NVIDIA GPU、驱动以及能够正常启动 Isaac Sim 的图形环境
- Isaac Sim 5.1.0
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab) 2.3.0
- [OpenArm Isaac Lab](https://github.com/enactic/openarm_isaac_lab)
- Isaac Sim / Isaac Lab 环境使用 Python 3.11
- 动作捕捉进程使用系统 Python 3.10，并需要 OpenCV、MediaPipe 0.10.21、PyYAML
- 可选：Intel RealSense D435；没有 D435 时可使用普通 USB/笔记本摄像头

安装问题大多来自 Isaac Sim、Isaac Lab 和 OpenArm Isaac Lab 版本不匹配。请先运行 OpenArm Isaac Lab 自带示例，再安装 Task Studio。本文档基于 `Isaac Sim 5.1.0 + Isaac Lab 2.3.0`。

## 从零安装

### 1. 安装 Isaac Sim 和 Isaac Lab

按照 [Isaac Lab 官方本地安装文档](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html#local-installation) 完成安装。安装后确认下面两个命令能够运行：

```bash
/path/to/IsaacLab/isaaclab.sh -p -c "import isaaclab; print('Isaac Lab OK')"
/path/to/IsaacLab/isaaclab.sh -p -c "import isaacsim; print('Isaac Sim OK')"
```

`/path/to/IsaacLab` 需要替换成实际的 Isaac Lab 根目录。

### 2. 安装 OpenArm Isaac Lab

```bash
git clone https://github.com/enactic/openarm_isaac_lab.git
cd openarm_isaac_lab
/path/to/IsaacLab/isaaclab.sh -p -m pip install -e source/openarm
/path/to/IsaacLab/isaaclab.sh -p scripts/tools/list_envs.py
```

最后一个命令应列出 OpenArm 环境。如果失败，请先修复 OpenArm Isaac Lab 环境。

### 3. 克隆 Task Studio

```bash
git clone https://github.com/jiangjiahui236/openarm-task-studio.git
cd openarm-task-studio
```

### 4. 安装动作捕捉依赖

相机进程由 `/usr/bin/python3` 启动，与 Isaac Sim Python 环境相互独立：

```bash
sudo apt update
sudo apt install -y python3-pip python3-tk v4l-utils ripgrep
python3 -m pip install --user -r requirements-camera.txt
python3 -c "import cv2, mediapipe, yaml; print('Camera dependencies OK')"
```

使用 D435 时再检查：

```bash
python3 -c "import pyrealsense2; print('RealSense OK')"
rs-enumerate-devices
```

如果 `rs-enumerate-devices` 不存在或普通用户无法访问 D435，请先按照 Intel RealSense 官方文档安装 SDK 和 udev 规则。

### 5. 配置安装路径

```bash
export ISAACSIM_VENV=/path/to/isaacsim/venv
export ISAACLAB_ROOT=/path/to/IsaacLab
export OPENARM_ISAAC_REPO=/path/to/openarm_isaac_lab
```

变量含义：

- `ISAACSIM_VENV`：包含 `bin/activate` 的 Isaac Sim Python 环境
- `ISAACLAB_ROOT`：包含可执行文件 `isaaclab.sh` 的 Isaac Lab 根目录
- `OPENARM_ISAAC_REPO`：包含 `source/openarm` 的 OpenArm Isaac Lab 仓库根目录

可以先检查三个路径：

```bash
test -f "$ISAACSIM_VENV/bin/activate" && echo "Isaac Sim venv OK"
test -x "$ISAACLAB_ROOT/isaaclab.sh" && echo "Isaac Lab root OK"
test -d "$OPENARM_ISAAC_REPO/source/openarm" && echo "OpenArm repo OK"
```

需要长期使用时，可以将三个 `export` 加入 `~/.bashrc`，然后重新打开终端。

### 6. 启动

```bash
cd openarm-task-studio
./run_task_studio.sh
```

首次启动 Isaac Sim 可能需要等待一段时间。进入后应看到：

- 中央为 OpenArm 双臂仿真视口
- 左侧为 `OpenArm Task Studio`
- 右侧为 `Task Point Parameters`
- 右下角为 `Motion Capture`
- 中央下方有 Console / Content / Button Test 标签页

只验证能否启动时可运行：

```bash
./run_task_studio.sh --headless --duration 30
```

## 快速上手

### 创建并运行简单 Move 任务

1. 点击 `LEFT ARM` 或 `RIGHT ARM` 选择手臂。
2. 点击 `Move`，视口中会出现一个任务点。
3. 拖动任务点，或在右侧参数窗口修改 XYZ。
4. 点击 `Confirm Current Edit`。
5. 继续添加任务点后，点击 `Validate Scene Path` 检查路径。
6. 点击 `Preview Selected` 预览单点，或点击 `Run Full Task` 执行完整任务。
7. 点击 `Save Task JSON`，任务会保存到 `tasks/current_task.json`。

### 创建 Grasp / Place 任务

1. 设置工作台高度和距离。
2. 点击 `Grasp`，设置物体形状、尺寸、抓取方向、接近高度和安全间隙。
3. 将物体代理拖到抓取位置并确认。
4. 点击 `Place`，设置目标位置并确认。
5. 先执行 `Validate Scene Path`，确认无明显 TCP/工作台碰撞，再运行任务。

当前碰撞检查只覆盖 TCP 路径与场景 AABB，不保证机械臂所有连杆、夹爪和携带物绝对无碰撞。

## 动作捕捉

1. 启动 Task Studio。
2. 在右下角 `Motion Capture` 面板选择 `RGB-D (D435)` 或 `RGB (Webcam)`。
3. 选择 `TEACH LEFT`、`TEACH RIGHT` 或 `TEACH BOTH`。
4. 点击 `Start Motion Teaching`，让双肩、双肘和双腕进入画面并保持中立姿态约 45 帧。
5. 移动手臂后点击 `Record Current Point`，最后点击 `Stop Motion Teaching`。

普通摄像头默认使用 OpenCV 设备 `0`。命令行验证摄像头：

```bash
./run_d435_teaching_camera.sh --source rgb --camera-index 0
./run_d435_teaching_camera.sh --source rgbd
```

RGB-D 模式使用对齐深度反投影三维关键点；RGB 模式使用 MediaPipe 单目 world landmarks，深度精度和稳定性通常低于 D435。

若 RGB 摄像头不是设备 `0`，先运行：

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
./run_d435_teaching_camera.sh --source rgb --camera-index 1
```

相机被浏览器、会议软件或其他 OpenCV 程序占用时，先关闭占用程序。切换 RGB-D / RGB 后需要重新保持中立姿态完成标定。

## 参数调节

Task Studio 中点击 `Open Motion Mapping Tuner` 可调节动作映射。配置保存在 `config/openarm_d435_teleop_visual.yaml`，相机发送器和仿真接收器会读取同一文件。

也可以单独启动：

```bash
python3 d435_tuning_gui.py --config config/openarm_d435_teleop_visual.yaml
```

调参前建议备份配置。参数过大会导致动作跳变或触及关节限位。

## 测试

```bash
python3 -m pip install --user -r requirements-dev.txt
python3 -m pytest -q tests
OPENARM_TASK_STUDIO_SELF_TEST=1 ./run_task_studio.sh --headless --duration 30
```

纯 Python 测试不需要启动 Isaac Sim。Isaac Sim 自检必须在完整仿真环境中运行。

## 遇到问题先看这里

### 启动脚本提示缺少环境变量

重新执行第 5 步的三个 `export`，并用 `test` 命令确认路径存在。

### 提示 `No module named openarm`

使用 Isaac Lab Python 重新安装 OpenArm 包：

```bash
cd "$OPENARM_ISAAC_REPO"
"$ISAACLAB_ROOT/isaaclab.sh" -p -m pip install -e source/openarm
```

### 提示 `No module named carb` 或 `omni`

这些模块只存在于 Isaac Sim Python 环境。不要直接用系统 `python3 launch_task_studio.py`，请始终使用 `./run_task_studio.sh`。

### Motion Capture 一直黑屏

- 先点击 `Start Motion Teaching`
- 检查 `/dev/video*` 和摄像头权限
- 确认摄像头没有被其他程序占用
- 在终端单独运行相机验证命令查看错误
- D435 模式确认 USB 3 连接和 `rs-enumerate-devices` 输出

### 能看到画面但机械臂不动

- 双肩、双肘和双腕必须同时可见
- 保持中立姿态约 45 帧直到完成标定
- 检查选择的是 `TEACH LEFT`、`TEACH RIGHT` 还是 `TEACH BOTH`
- 观察终端是否持续收到 UDP 5010 数据

### 窗口布局没有自动停靠

先等几秒，让自动布局跑完。如果还是不正常，可以重启一次，并检查是否同时启用了会修改窗口布局的其他 Kit 扩展。

## 目录

| 路径 | 用途 |
| --- | --- |
| `launch_task_studio.py` | Isaac Lab 启动入口、仿真循环、双臂 IK 与任务执行 |
| `studio_scene.py` | OpenArm 双臂机器人、地面、灯光和工作台场景 |
| `openarm_task_studio/extension.py` | Kit UI、任务点编辑、窗口和动作捕捉交互 |
| `openarm_task_studio/task_model.py` | 任务 JSON 数据模型与保存/加载 |
| `openarm_task_studio/task_compiler.py` | 将 Move / Grasp / Place 编译为执行阶段 |
| `openarm_task_studio/stage_objects.py` | USD 任务点、物体代理、路径和碰撞几何 |
| `openarm_task_studio/runtime.py` | UI 与仿真循环共享的运行状态 |
| `openarm_task_studio/d435_teaching.py` | 动作捕捉 UDP 接收与关节映射 |
| `openarm_task_studio/motion_monitor.py` | 动作捕捉运行指标和日志 |
| `openarm_task_studio/remote_button.py` | ESP32-C3 无线记录按钮接收器 |
| `d435_rgbd_sender.py` | RGB-D / RGB 采集、MediaPipe 识别和 UDP 发送 |
| `d435_tuning_base.py` | 通用 Tkinter 参数调节器实现 |
| `d435_tuning_gui.py` | Task Studio 专用调参项和界面扩展 |
| `tests/` | 不依赖 Isaac Sim 启动的单元测试 |
| `config/` | Kit 扩展清单和动作映射配置 |
| `esp32c3_remote_button/` | 可选 ESP32-C3 固件与接线说明 |

这些 Python 文件按职责拆分，都是运行功能或测试所需文件；`__pycache__`、`.pytest_cache`、日志和个人任务数据由 `.gitignore` 排除，不会进入 GitHub。

## 贡献与许可

欢迎提交 Issue 和 Pull Request。报告问题时请附上系统版本、Isaac Sim / Isaac Lab 版本和完整报错。贡献代码前请阅读 `CONTRIBUTING.md`。

本项目采用 [GNU General Public License v3.0](LICENSE)，依赖的软件和库继续遵循各自的许可证。
